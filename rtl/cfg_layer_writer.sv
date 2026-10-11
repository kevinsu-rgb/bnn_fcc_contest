// Per-layer configuration writer: turns a payload byte stream into bnn_layer config writes.
// Weights: each neuron is ceil(F/8) bytes (bit i at byte i/8 bit i%8); bytes go through a bit_packer
// (8 -> PW bits, pad 1) and each emitted word is written to lane (neuron % PN) at
// group*PASSES + pass. Thresholds: 4 bytes little-endian, signed 32-bit, clamped to [0, F+1].
// s_ready depends on registered state only. Writes are single-cycle pulses.
// idle: low from the cycle after start until the message's last write (weights: flush of neuron N-1;
// thresholds: write of neuron N-1), high from the cycle after it; abort and rst force it high.
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
  localparam int LAST_BITS = F - 8 * (BPN - 1);
  localparam logic [LANE_W-1:0]  LAST_LANE  = LANE_W'((N - 1) % PN);        // lane of neuron N-1
  localparam logic [WADDR_W-1:0] LAST_GBASE = WADDR_W'((GROUPS - 1) * PASSES);
  localparam logic [TADDR_W-1:0] LAST_GRP   = TADDR_W'(GROUPS - 1);

  // Message type is latched at start so a writer's tail flush does not depend on the
  // parser's (shared) msg_type staying stable after the message's last byte.
  logic mt_q, mt;
  always_ff @(posedge clk) if (start) mt_q <= msg_type;
  assign mt = start ? msg_type : mt_q;

  logic clr;
  assign clr = start || abort;

  logic acc;
  assign acc = s_valid && s_ready;

  // ---------------- weights
  logic            pk_ready, pk_valid, pk_flush_done;
  logic [PW-1:0]   pk_word;
  logic [BN_W-1:0] bn_q;
  logic            last_byte;
  logic [3:0]      nbits;
  logic [LANE_W-1:0]  lane_q;
  logic [WADDR_W-1:0] waddr_q, gbase_q;

  assign last_byte = (bn_q == BN_W'(BPN - 1));
  assign nbits     = last_byte ? 4'(LAST_BITS) : 4'd8;

  bit_packer #(.IN_W(8), .OUT_W(PW), .PAD(1'b1)) u_pk (
    .clk(clk), .rst(rst), .clear(clr),
    .in_valid(s_valid && !mt), .in_ready(pk_ready),
    .in_bits(s_byte), .in_count(nbits), .in_flush(last_byte),
    .out_valid(pk_valid), .out_word(pk_word), .flush_done(pk_flush_done)
  );

  assign s_ready = mt ? 1'b1 : pk_ready;
  assign w_we    = pk_valid && !mt;
  assign w_lane  = lane_q;
  assign w_addr  = waddr_q;
  assign w_data  = pk_word;

  always_ff @(posedge clk) begin
    if (rst || clr) begin
      bn_q    <= '0;
      lane_q  <= '0;
      waddr_q <= '0;
      gbase_q <= '0;
    end else if (!mt) begin
      if (acc) bn_q <= last_byte ? '0 : bn_q + 1'b1;
      if (pk_flush_done) begin
        if (lane_q == LANE_W'(PN - 1)) begin
          lane_q  <= '0;
          gbase_q <= WADDR_W'(int'(gbase_q) + PASSES);
          waddr_q <= WADDR_W'(int'(gbase_q) + PASSES);
        end else begin
          lane_q  <= lane_q + 1'b1;
          waddr_q <= gbase_q;
        end
      end else if (pk_valid) begin
        waddr_q <= waddr_q + 1'b1;
      end
    end
  end

  // ---------------- idle (message active until its last write)
  logic active_q, w_done, t_done;
  assign w_done = !mt && pk_flush_done && (lane_q == LAST_LANE) && (gbase_q == LAST_GBASE);
  assign t_done = mt && (HAS_THR ? (t_we && t_lane == LAST_LANE && t_addr == LAST_GRP) : 1'b1);
  assign idle   = !active_q;

  always_ff @(posedge clk) begin
    if (rst || abort)          active_q <= 1'b0;
    else if (start)            active_q <= 1'b1;
    else if (w_done || t_done) active_q <= 1'b0;
  end

  // ---------------- thresholds
  logic [1:0]         tb_q;
  logic [23:0]        tsh_q;
  logic [LANE_W-1:0]  tlane_q;
  logic [TADDR_W-1:0] tgrp_q;
  logic signed [31:0] tval;
  logic [THR_W-1:0]   tclamp;

  assign tval = {s_byte, tsh_q};
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
      end else if (mt && acc) begin
        tsh_q <= {s_byte, tsh_q[23:8]};
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
