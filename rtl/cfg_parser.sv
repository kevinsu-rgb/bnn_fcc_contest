// Configuration message parser: collects the 16-byte little-endian header, validates it against the
// topology, waits for the compute pipeline to go idle, then routes the payload bytes to the selected
// layer writer (msg_sel / msg_type) and tracks which weight/threshold memories are loaded.
//   HEADER  : 16 bytes; a TLAST (or null item) on any header item discards the header collected so far.
//             The header is validated on its 16th byte from registered fields (bytes 0..11; the
//             reserved bytes 12..15 are not stored): valid -> WAIT_IDLE, invalid -> DISCARD
//             (or straight back to HEADER when total_bytes == 0).
//   WAIT    : cfg_busy = 1, no bytes accepted; pipeline_idle is honored from the second cycle on.
//             Then msg_start is pulsed (registered, one cycle, no byte accepted in that cycle), msg_sel
//             (registered one-hot layer select) is loaded and the layer's load flag is cleared.
//   PAYLOAD : total_bytes bytes to pl_*; the last one asserts drop_rest and sets the load flag.
//             A TLAST before that (incl. a null item) truncates: msg_abort (registered: it pulses in the
//             cycle after that handshake), the byte carrying TLAST is not forwarded, the flag stays
//             cleared. The next msg_start is at least 16 header bytes + 2 WAIT cycles later.
//   DISCARD : consume total_bytes bytes (TLAST ends it early); drop_rest on the last one.
// rem_q's terminal count is a registered flag (last_q: one byte left).
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

  typedef enum logic [1:0] {S_HEADER, S_WAIT, S_PAYLOAD, S_DISCARD} state_t;

  state_t       state_q;
  logic [3:0]   hcnt_q;        // header bytes collected
  logic [95:0]  hdr_q;         // header bytes 0..11, byte 0 at [7:0] once complete
  logic [31:0]  rem_q;         // payload / discard bytes remaining
  logic         last_q;        // rem_q == 1
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
  logic [L-1:0] w_match, t_match;
  logic        hdr_valid;

  assign h_type    = hdr_q[7:0];
  assign h_layer   = hdr_q[15:8];
  assign h_inputs  = hdr_q[31:16];
  assign h_neurons = hdr_q[47:32];
  assign h_bpn     = hdr_q[63:48];
  assign h_total   = hdr_q[95:64];

  for (genvar l = 0; l < L; l++) begin : g_tab
    localparam int F_L   = TOPOLOGY[l];
    localparam int N_L   = TOPOLOGY[l+1];
    localparam int BPN_L = cdiv(F_L, 8);
    assign w_match[l] = (32'(h_layer) == 32'(l)) && (32'(h_inputs) == 32'(F_L)) &&
                        (32'(h_neurons) == 32'(N_L)) && (32'(h_bpn) == 32'(BPN_L)) &&
                        (h_total == 32'(N_L * BPN_L));
    if (l < L - 1) begin : g_thr
      assign t_match[l] = (32'(h_layer) == 32'(l)) && (32'(h_neurons) == 32'(N_L)) &&
                          (h_bpn == 16'd4) && (h_total == 32'(4 * N_L));
    end else begin : g_nothr
      assign t_match[l] = 1'b0;
    end
  end

  assign hdr_valid = ((h_type == 8'd0) && (|w_match)) || ((h_type == 8'd1) && (|t_match));

  // ---------------- handshake-level control
  logic hs, tlast, last_cnt, done_now;

  always_comb begin
    unique case (state_q)
      S_HEADER, S_DISCARD: s_ready = 1'b1;
      S_PAYLOAD:           s_ready = pl_ready && !start_q;
      default:             s_ready = 1'b0;
    endcase
  end

  assign hs        = s_valid && s_ready;
  assign tlast     = s_beat_last || s_null;  // a null item always carries TLAST
  assign last_cnt  = last_q;
  assign done_now  = hs && !s_null && last_cnt;   // last payload / discard byte (in those states)

  assign drop_rest = done_now && (state_q == S_PAYLOAD || state_q == S_DISCARD);
  assign msg_abort = abort_q;
  assign msg_start = start_q;
  assign msg_sel   = sel_q;
  assign msg_type  = type_q;
  assign pl_valid  = (state_q == S_PAYLOAD) && !start_q && s_valid && !s_null && (!tlast || last_cnt);
  assign pl_byte   = s_byte;
  assign cfg_busy  = (state_q == S_WAIT) || (state_q == S_PAYLOAD);
  assign configured = (&w_loaded) && (&(t_loaded | T_NONE));

  // ---------------- state
  always_ff @(posedge clk) begin
    if (rst) begin
      state_q  <= S_HEADER;
      hcnt_q   <= '0;
      armed_q  <= 1'b0;
      start_q  <= 1'b0;
      abort_q  <= 1'b0;
      sel_q    <= '0;
      type_q   <= 1'b0;
      w_loaded <= '0;
      t_loaded <= '0;
    end else begin
      start_q <= 1'b0;
      abort_q <= hs && (state_q == S_PAYLOAD) && tlast && !done_now;
      if (start_q) begin
        for (int l = 0; l < L; l++) if (sel_q[l]) begin
          if (type_q) t_loaded[l] <= 1'b0;
          else        w_loaded[l] <= 1'b0;
        end
      end
      unique case (state_q)
        S_HEADER: if (hs) begin
          if (tlast) begin
            hcnt_q <= '0;
          end else begin
            hcnt_q <= hcnt_q + 1'b1;
            if (hcnt_q < 4'd12) hdr_q <= {s_byte, hdr_q[95:8]};
            if (hcnt_q == 4'd15) begin
              rem_q  <= h_total;
              last_q <= (h_total == 32'd1);
              if (hdr_valid) begin
                state_q <= S_WAIT;
                armed_q <= 1'b0;
              end else if (h_total != '0) begin
                state_q <= S_DISCARD;
              end
            end
          end
        end
        S_WAIT: begin
          armed_q <= 1'b1;
          if (armed_q && pipeline_idle) begin
            state_q <= S_PAYLOAD;
            start_q <= 1'b1;
            for (int l = 0; l < L; l++) sel_q[l] <= (h_layer == 8'(l));
            type_q  <= h_type[0];
          end
        end
        S_PAYLOAD: if (hs) begin
          if (!s_null) begin
            rem_q  <= rem_q - 1'b1;
            last_q <= (rem_q == 32'd2);
          end
          if (done_now) begin
            state_q <= S_HEADER;
            for (int l = 0; l < L; l++) if (sel_q[l]) begin
              if (type_q) t_loaded[l] <= 1'b1;
              else        w_loaded[l] <= 1'b1;
            end
          end else if (tlast) begin
            state_q <= S_HEADER;
          end
        end
        S_DISCARD: if (hs) begin
          if (!s_null) begin
            rem_q  <= rem_q - 1'b1;
            last_q <= (rem_q == 32'd2);
          end
          if (done_now || tlast) state_q <= S_HEADER;
        end
        default: state_q <= S_HEADER;
      endcase
    end
  end
endmodule
