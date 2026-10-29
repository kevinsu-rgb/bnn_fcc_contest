// Per-layer configuration writer: turns a payload byte stream into bnn_layer config writes.
// Every accepted byte is registered first (in_v_q, in_byte_q) and handled one cycle later; the
// byte-in-neuron counter bn_q advances on the registered byte, so nothing upstream drives it.
// Weights: each neuron is ceil(F/8) bytes (bit i at byte i/8 bit i%8). Byte b is stored at fixed
// position stage[8b +: 8] (bits >= F dropped). After the neuron's last byte the writer emits PASSES
// words, one per cycle: word p = stage bits [p*PW, (p+1)*PW), bits >= F padded with 1, written to
// lane (neuron % PN) at group*PASSES + p.
//   PW >= 8 (OVERLAP): emission overlaps the next neuron's bytes (byte b < word p never overlaps word p,
//            and PASSES <= ceil(F/8)), so s_ready is constant 1.
//   PW <  8: s_ready drops from the cycle after a neuron's last byte until its emission ends.
// Thresholds: 4 bytes little-endian, signed 32-bit, clamped to [0, F+1].
// The write ports are registered. s_ready depends on registered state only.
// idle: low from the cycle after start until the message's last write (weights: last word of neuron
// N-1; thresholds: neuron N-1), high from the cycle after it; abort and rst force it high.
module cfg_layer_writer import bnn_fcc_pkg::*; #(
  parameter int F       = 8,
  parameter int N       = 8,
  parameter int PW      = 8,
  parameter int PN      = 8,
  parameter bit HAS_THR = 1'b1
) (
  input  logic                                    clk,
  input  logic                                    rst,
  input  logic                                    start,
  /* verilator lint_off SYMRSVDWORD */
  input  logic                                    abort,
  /* verilator lint_on SYMRSVDWORD */
  input  logic                                    msg_type,   // 0 = weights, 1 = thresholds
  input  logic                                    s_valid,
  output logic                                    s_ready,
  input  logic [7:0]                              s_byte,
  output logic                                    w_we,
  output logic [bits(PN)-1:0]                     w_lane,
  output logic [bits(cdiv(N,PN)*cdiv(F,PW))-1:0]  w_addr,
  output logic [PW-1:0]                           w_data,
  output logic                                    t_we,
  output logic [bits(PN)-1:0]                     t_lane,
  output logic [bits(cdiv(N,PN))-1:0]             t_addr,
  output logic [bits(F+2)-1:0]                    t_data,
  output logic                                    idle
);
  localparam int GROUPS  = cdiv(N, PN);
  localparam int PASSES  = cdiv(F, PW);
  localparam int THR_W   = bits(F + 2);
  localparam int LANE_W  = bits(PN);
  localparam int WADDR_W = bits(GROUPS * PASSES);
  localparam int TADDR_W = bits(GROUPS);
  localparam int BPN     = cdiv(F, 8);
  localparam int BN_W    = bits(BPN);
  localparam int P_W     = bits(PASSES);
  localparam int PADW    = PASSES * PW - F;
  localparam bit OVERLAP = (PW >= 8);
  localparam logic [LANE_W-1:0]  LAST_LANE  = LANE_W'((N - 1) % PN);        // lane of neuron N-1
  localparam logic [WADDR_W-1:0] LAST_WADDR = WADDR_W'(GROUPS * PASSES - 1); // its last word
  localparam logic [TADDR_W-1:0] LAST_GRP   = TADDR_W'(GROUPS - 1);

  // Message type latched at start; the parser never presents a byte in the start cycle.
  logic mt_q;
  always_ff @(posedge clk) if (start) mt_q <= msg_type;

  logic clr;
  assign clr = start || abort;

  logic acc;
  assign acc = s_valid && s_ready;

  // ---------------- input register
  logic            in_v_q;
  logic [7:0]      in_byte_q;
  logic [BN_W-1:0] bn_q;        // byte index (within its neuron) of the registered byte
  logic            wr_byte, last_byte;

  assign wr_byte   = in_v_q && !mt_q;
  assign last_byte = (bn_q == BN_W'(BPN - 1));

  always_ff @(posedge clk) begin
    if (rst || clr) begin
      in_v_q <= 1'b0;
      bn_q   <= '0;
    end else begin
      in_v_q <= acc;
      if (wr_byte) bn_q <= last_byte ? '0 : bn_q + 1'b1;
    end
  end

  always_ff @(posedge clk) if (acc) in_byte_q <= s_byte;

  // ---------------- weights: neuron staging register (fixed byte positions)
  logic [F-1:0] stage;

  for (genvar c = 0; c < BPN; c++) begin : g_byte
    localparam int NB = imin(8, F - 8 * c);
    always_ff @(posedge clk)
      if (wr_byte && bn_q == BN_W'(c)) stage[8*c +: NB] <= in_byte_q[NB-1:0];
  end

  // Word view of the padded neuron; rows >= PASSES are never selected.
  logic [PASSES*PW-1:0] padded;
  logic [PW-1:0]        words [2**P_W];

  if (PADW > 0) begin : g_pad
    assign padded = {{PADW{1'b1}}, stage};
  end else begin : g_nopad
    assign padded = stage;
  end

  always_comb begin
    for (int r = 0; r < 2**P_W; r++)
      words[r] = (r < PASSES) ? padded[r*PW +: PW] : '1;
  end

  // ---------------- weights: emission
  logic               em_q, em_start, em_last;
  logic [P_W-1:0]     p_q;
  logic [LANE_W-1:0]  lane_q;
  logic [WADDR_W-1:0] waddr_q, gbase_q;

  assign em_start = wr_byte && last_byte;
  assign em_last  = (p_q == P_W'(PASSES - 1));
  assign s_ready  = mt_q || OVERLAP || (!em_start && !em_q);

  always_ff @(posedge clk) begin
    if (rst || clr) begin
      em_q    <= 1'b0;
      p_q     <= '0;
      lane_q  <= '0;
      waddr_q <= '0;
      gbase_q <= '0;
    end else if (em_q) begin
      if (em_last) begin
        em_q <= em_start;   // the next neuron may complete in the cycle this emission ends
        p_q  <= '0;
        if (lane_q == LANE_W'(PN - 1)) begin
          lane_q  <= '0;
          gbase_q <= WADDR_W'(int'(gbase_q) + PASSES);
          waddr_q <= WADDR_W'(int'(gbase_q) + PASSES);
        end else begin
          lane_q  <= lane_q + 1'b1;
          waddr_q <= gbase_q;
        end
      end else begin
        p_q     <= p_q + 1'b1;
        waddr_q <= waddr_q + 1'b1;
      end
    end else if (em_start) begin
      em_q <= 1'b1;
    end
  end

  // registered write port
  always_ff @(posedge clk) begin
    if (rst || clr) w_we <= 1'b0;
    else            w_we <= em_q;
    w_lane <= lane_q;
    w_addr <= waddr_q;
    w_data <= words[p_q];
  end

  // ---------------- idle (message active until its last write)
  logic active_q, w_done, t_done;
  assign w_done = w_we && (w_lane == LAST_LANE) && (w_addr == LAST_WADDR);
  assign t_done = mt_q && (HAS_THR ? (t_we && t_lane == LAST_LANE && t_addr == LAST_GRP) : 1'b1);
  assign idle   = !active_q;

  always_ff @(posedge clk) begin
    if (rst || abort)          active_q <= 1'b0;
    else if (start)            active_q <= 1'b1;
    else if (w_done || t_done) active_q <= 1'b0;
  end

  // ---------------- thresholds (from the input register)
  logic [1:0]         tb_q;
  logic [23:0]        tsh_q;
  logic [LANE_W-1:0]  tlane_q;
  logic [TADDR_W-1:0] tgrp_q;
  logic signed [31:0] tval;
  logic [THR_W-1:0]   tclamp;

  assign tval = {in_byte_q, tsh_q};
  always_comb begin
    if (tval < 0)                          tclamp = '0;
    else if (tval > 32'(F + 1))            tclamp = THR_W'(F + 1);
    else                                   tclamp = THR_W'(tval);
  end

  if (HAS_THR) begin : g_thr
    always_ff @(posedge clk) begin
      t_we <= 1'b0;
      if (rst || clr) begin
        tb_q    <= '0;
        tsh_q   <= '0;
        tlane_q <= '0;
        tgrp_q  <= '0;
      end else if (in_v_q && mt_q) begin
        tsh_q <= {in_byte_q, tsh_q[23:8]};
        tb_q  <= tb_q + 1'b1;
        if (tb_q == 2'd3) begin
          t_we   <= 1'b1;
          t_lane <= tlane_q;
          t_addr <= tgrp_q;
          t_data <= tclamp;
          if (tlane_q == LANE_W'(PN - 1)) begin
            tlane_q <= '0;
            tgrp_q  <= tgrp_q + 1'b1;
          end else begin
            tlane_q <= tlane_q + 1'b1;
          end
        end
      end
    end
  end else begin : g_nothr
    assign t_we   = 1'b0;
    assign t_lane = '0;
    assign t_addr = '0;
    assign t_data = '0;
    /* verilator lint_off UNUSEDSIGNAL */
    logic unused;
    assign unused = &{1'b0, tclamp, tb_q, tsh_q, tlane_q, tgrp_q};
    /* verilator lint_on UNUSEDSIGNAL */
    assign tb_q = '0; assign tsh_q = '0; assign tlane_q = '0; assign tgrp_q = '0;
  end
endmodule
