// Configuration message parser: collects the 16-byte little-endian header, validates it against the
// topology, waits for the compute pipeline to go idle, then routes the payload bytes to the selected
// layer writer (msg_sel / msg_type) and tracks which weight/threshold memories are loaded.
//   HEADER  : 16 bytes; a TLAST (or null item) on any header item discards the header collected so far.
//             Byte k (k < 12) is written at its one-hot position pos_q[k] (in every cycle there, the
//             handshake's byte last); the reserved bytes 12..15 are not stored. hdr_q is final after byte 11, so the field compares are registered every
//             cycle (stage 1) and combined into hdr_valid_q (stage 2); both are settled before the 16th
//             byte, which then selects: valid -> WAIT_IDLE, invalid -> DISCARD
//             (or straight back to HEADER when total_bytes == 0).
//   WAIT    : cfg_busy = 1, no bytes accepted; pipeline_idle is honored from the second cycle on.
//             Then msg_start is pulsed (registered, one cycle, no byte accepted in that cycle), msg_sel
//             (registered one-hot layer select) is loaded and the layer's load flag is cleared.
//   PAYLOAD : total_bytes bytes to pl_*; the last one asserts drop_rest and sets the load flag.
//             A TLAST before that (incl. a null item) truncates: msg_abort (registered: it pulses in the
//             cycle after that handshake), the byte carrying TLAST is not forwarded, the flag stays
//             cleared. The next msg_start is at least 16 header bytes + 2 WAIT cycles later.
//   DISCARD : consume total_bytes bytes (TLAST ends it early); drop_rest on the last one.
// The state is one-hot (st_q). The byte counter is loaded from the header in every HEADER cycle (the
// last one is the 16th byte's) and split: lo_q counts LO_W bits, hi_q takes the precomputed hi_m1_q one
// cycle after lo_q wraps (the next wrap or rem == 2 check is at least 2**LO_W - 3 decrements later).
// Its terminal count is a registered flag (last_q: one byte left).
// s_ready depends on registered state and pl_ready only (pl_ready = selected writer's registered s_ready).
module cfg_parser import bnn_fcc_pkg::*; #(
  parameter int TOTAL_LAYERS = 4,
  parameter int TOPOLOGY [TOTAL_LAYERS] = '{784, 256, 256, 10}
) (
  input  logic       clk,
  input  logic       rst,
  input  logic       s_valid,
  output logic       s_ready,
  input  logic [7:0] s_byte,
  input  logic       s_null,
  input  logic       s_beat_last,
  output logic       drop_rest,
  input  logic       pipeline_idle,
  output logic       cfg_busy,
  output logic       configured,
  output logic       msg_start,
  output logic       msg_abort,
  output logic [TOTAL_LAYERS-2:0] msg_sel,
  output logic       msg_type,
  output logic       pl_valid,
  input  logic       pl_ready,
  output logic [7:0] pl_byte
);
  localparam int L = TOTAL_LAYERS - 1;           // non-input layers
  localparam logic [L-1:0] T_NONE = L'(1) << (L - 1);   // output layer has no thresholds
  localparam int LO_W = 5;                       // low counter bits

  // one-hot state bits
  localparam int S_HEADER = 0, S_WAIT = 1, S_PAYLOAD = 2, S_DISCARD = 3;
  localparam logic [3:0] ST_HEADER  = 4'b0001, ST_WAIT = 4'b0010,
                         ST_PAYLOAD = 4'b0100, ST_DISCARD = 4'b1000;

  logic [3:0]   st_q;
  logic [15:0]  pos_q;         // one-hot header byte position; zero outside HEADER
  logic [95:0]  hdr_q;         // header bytes 0..11, byte k at [8k+7:8k]
  logic         pay_q;         // PAYLOAD and not the msg_start cycle
  logic [LO_W-1:0]  lo_q;      // payload / discard bytes remaining = {hi_q, lo_q}
  logic [31-LO_W:0] hi_q, hi_m1_q;
  logic         lo_z_q;        // lo_q == 0
  logic         hi_z_q;        // hi_q == 0 (free-running)
  logic         wrap_q;        // lo_q wrapped in the previous cycle: hi_q <= hi_m1_q
  logic         last_q;        // remaining == 1
  logic         abort_q;       // msg_abort
  logic         armed_q;       // second and later WAIT cycles
  logic         start_q;       // msg_start
  logic [L-1:0] sel_q;         // one-hot layer of the current message
  logic         type_q;
  logic [L-1:0] w_loaded, t_loaded;

  // ---------------- header fields and validation
  logic [7:0]  h_type, h_layer;
  logic [15:0] h_inputs, h_neurons, h_bpn;
  logic [31:0] h_total;

  assign h_type    = hdr_q[7:0];
  assign h_layer   = hdr_q[15:8];
  assign h_inputs  = hdr_q[31:16];
  assign h_neurons = hdr_q[47:32];
  assign h_bpn     = hdr_q[63:48];
  assign h_total   = hdr_q[95:64];

  // stage 1: field compares
  logic         c_type0_q, c_type1_q, c_bpn4_q, c_tot_nz_q, c_tot1_q;
  logic [L-1:0] c_layer_q, c_in_q, c_neu_q, c_bpn_q, c_wtot_q, c_ttot_q;
  // stage 2
  logic [L-1:0] w_match, t_match;
  logic         hdr_valid_q;

  always_ff @(posedge clk) begin
    c_type0_q  <= (h_type == 8'd0);
    c_type1_q  <= (h_type == 8'd1);
    c_bpn4_q   <= (h_bpn == 16'd4);
    c_tot_nz_q <= (h_total != '0);
    c_tot1_q   <= (h_total == 32'd1);
  end

  for (genvar l = 0; l < L; l++) begin : g_tab
    localparam int F_L   = TOPOLOGY[l];
    localparam int N_L   = TOPOLOGY[l+1];
    localparam int BPN_L = cdiv(F_L, 8);
    always_ff @(posedge clk) begin
      c_layer_q[l] <= (32'(h_layer) == 32'(l));
      c_in_q[l]    <= (32'(h_inputs) == 32'(F_L));
      c_neu_q[l]   <= (32'(h_neurons) == 32'(N_L));
      c_bpn_q[l]   <= (32'(h_bpn) == 32'(BPN_L));
      c_wtot_q[l]  <= (h_total == 32'(N_L * BPN_L));
      c_ttot_q[l]  <= (l < L - 1) && (h_total == 32'(4 * N_L));
    end
    assign w_match[l] = c_layer_q[l] && c_in_q[l] && c_neu_q[l] && c_bpn_q[l] && c_wtot_q[l];
    assign t_match[l] = c_layer_q[l] && c_neu_q[l] && c_bpn4_q && c_ttot_q[l];   // 0 for l = L-1
  end

  always_ff @(posedge clk) hdr_valid_q <= (c_type0_q && (|w_match)) || (c_type1_q && (|t_match));

  // ---------------- handshake-level control
  logic hs, tlast, hd_wr, done_now, dec;

  assign s_ready   = st_q[S_HEADER] || st_q[S_DISCARD] || (pay_q && pl_ready);
  assign hs        = s_valid && s_ready;
  assign tlast     = s_beat_last || s_null;  // a null item always carries TLAST
  assign hd_wr     = s_valid && st_q[S_HEADER];   // s_ready = 1 in HEADER
  assign done_now  = hs && !s_null && last_q;     // last payload / discard byte (in those states)
  assign dec       = hs && !s_null && (st_q[S_PAYLOAD] || st_q[S_DISCARD]);

  assign drop_rest = done_now && (st_q[S_PAYLOAD] || st_q[S_DISCARD]);
  assign msg_abort = abort_q;
  assign msg_start = start_q;
  assign msg_sel   = sel_q;
  assign msg_type  = type_q;
  assign pl_valid  = pay_q && s_valid && !s_null && (!s_beat_last || last_q);
  assign pl_byte   = s_byte;
  assign cfg_busy  = st_q[S_WAIT] || st_q[S_PAYLOAD];
  assign configured = (&w_loaded) && (&(t_loaded | T_NONE));

  // ---------------- header bytes
  // pos_q is zero outside HEADER and advances only on a handshake, so the last write at each position
  // is the handshake's byte; the enables are register outputs.
  always_ff @(posedge clk) begin
    for (int k = 0; k < 12; k++) if (pos_q[k]) hdr_q[8*k +: 8] <= s_byte;
  end

  // ---------------- byte counter
  always_ff @(posedge clk) begin
    hi_m1_q <= hi_q - 1'b1;
    hi_z_q  <= (hi_q == '0);
    wrap_q  <= dec && lo_z_q;
    if (st_q[S_HEADER]) begin
      lo_q   <= h_total[LO_W-1:0];
      hi_q   <= h_total[31:LO_W];
      lo_z_q <= (h_total[LO_W-1:0] == '0);
      last_q <= c_tot1_q;
    end else begin
      if (dec) begin
        lo_q   <= lo_q - 1'b1;
        lo_z_q <= (lo_q == LO_W'(1));
        last_q <= hi_z_q && (lo_q == LO_W'(2));
      end
      if (wrap_q) hi_q <= hi_m1_q;
    end
  end

  // ---------------- state
  always_ff @(posedge clk) begin
    if (rst) begin
      st_q     <= ST_HEADER;
      pos_q    <= 16'd1;
      pay_q    <= 1'b0;
      armed_q  <= 1'b0;
      start_q  <= 1'b0;
      abort_q  <= 1'b0;
      sel_q    <= '0;
      type_q   <= 1'b0;
      w_loaded <= '0;
      t_loaded <= '0;
    end else begin
      start_q <= 1'b0;
      armed_q <= st_q[S_WAIT];
      pay_q   <= st_q[S_PAYLOAD] && !(hs && (tlast || last_q));
      abort_q <= hs && st_q[S_PAYLOAD] && tlast && !done_now;
      if (start_q) begin
        for (int l = 0; l < L; l++) if (sel_q[l]) begin
          if (type_q) t_loaded[l] <= 1'b0;
          else        w_loaded[l] <= 1'b0;
        end
      end
      if (hd_wr) begin
        pos_q <= tlast ? 16'd1 : {pos_q[14:0], pos_q[15]};
        if (!tlast && pos_q[15]) begin
          if (hdr_valid_q) begin
            st_q  <= ST_WAIT;
            pos_q <= '0;
          end else if (c_tot_nz_q) begin
            st_q  <= ST_DISCARD;
            pos_q <= '0;
          end
        end
      end
      if (st_q[S_WAIT] && armed_q && pipeline_idle) begin
        st_q    <= ST_PAYLOAD;
        start_q <= 1'b1;
        sel_q   <= c_layer_q;
        type_q  <= h_type[0];
      end
      if (st_q[S_PAYLOAD] && hs) begin
        if (done_now) begin
          st_q  <= ST_HEADER;
          pos_q <= 16'd1;
          for (int l = 0; l < L; l++) if (sel_q[l]) begin
            if (type_q) t_loaded[l] <= 1'b1;
            else        w_loaded[l] <= 1'b1;
          end
        end else if (tlast) begin
          st_q  <= ST_HEADER;
          pos_q <= 16'd1;
        end
      end
      if (st_q[S_DISCARD] && hs && (done_now || tlast)) begin
        st_q  <= ST_HEADER;
        pos_q <= 16'd1;
      end
    end
  end
endmodule
