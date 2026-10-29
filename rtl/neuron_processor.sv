// Neuron processor: XNOR register -> pipelined popcount -> accumulate over passes -> threshold compare.
// out_valid, out_y and out_acc are valid together np_latency(PW, POPCOUNT_LEVELS_PER_STAGE) cycles
// after an in_valid && last_pass input.
// Contract: the passes of one neuron arrive in consecutive cycles (in_valid gaps only between
// neurons). The accumulator therefore loads every cycle without an enable: first_pass restarts it,
// and whatever it holds between neurons is overwritten by the next first pass.
// thr is presented in the same cycle as x/w (delayed internally to align with the accumulator).
// Only the valid pipeline is reset; the accumulator and data are not.
module neuron_processor import bnn_fcc_pkg::*; #(
  parameter int PW = 8,
  parameter int ACC_W = 4,
  parameter int THR_W = 4,
  parameter int POPCOUNT_LEVELS_PER_STAGE = 3
) (
  input  logic             clk,
  input  logic             rst,
  input  logic             in_valid,
  input  logic             first_pass,
  input  logic             last_pass,
  input  logic [PW-1:0]    x,
  input  logic [PW-1:0]    w,
  input  logic [THR_W-1:0] thr,
  output logic             out_valid,
  output logic             out_y,
  output logic [ACC_W-1:0] out_acc
);
  localparam int PC_LAT = popcount_latency(PW, POPCOUNT_LEVELS_PER_STAGE);
  localparam int PC_W   = bits(PW + 1);
  localparam int CMP_W  = imax(ACC_W, THR_W);
  localparam int SB     = PC_LAT + 1;   // side-band delay: XNOR register + popcount

  logic [PW-1:0]   xn_q;
  logic [PC_W-1:0] pc;

  always_ff @(posedge clk) xn_q <= ~(x ^ w);

  popcount #(.W(PW), .LEVELS_PER_STAGE(POPCOUNT_LEVELS_PER_STAGE)) u_pc (
    .clk(clk), .in_bits(xn_q), .count(pc)
  );

  // Side-band delay line, SB stages; f_d is kept per lane (it selects every accumulator bit).
  logic                                 v_d [SB];
  (* keep = "true" *) logic             f_d [SB];
  logic                                 l_d [SB];
  logic [THR_W-1:0]                     t_d [SB];

  always_ff @(posedge clk) begin
    f_d[0] <= first_pass;
    l_d[0] <= last_pass;
    t_d[0] <= thr;
    for (int i = 1; i < SB; i++) begin
      f_d[i] <= f_d[i-1];
      l_d[i] <= l_d[i-1];
      t_d[i] <= t_d[i-1];
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < SB; i++) v_d[i] <= 1'b0;
    end else begin
      v_d[0] <= in_valid;
      for (int i = 1; i < SB; i++) v_d[i] <= v_d[i-1];
    end
  end

  // Accumulate stage (no enable), then compare stage.
  logic [ACC_W-1:0] acc, acc_o;
  logic [THR_W-1:0] t_a;
  logic             v_a;

  always_ff @(posedge clk) begin
    acc <= f_d[SB-1] ? ACC_W'(pc) : (acc + ACC_W'(pc));
    t_a <= t_d[SB-1];
  end

  always_ff @(posedge clk) begin
    if (rst) v_a <= 1'b0;
    else     v_a <= v_d[SB-1] & l_d[SB-1];
  end

  always_ff @(posedge clk) begin
    out_y <= (CMP_W'(acc) >= CMP_W'(t_a));
    acc_o <= acc;
  end

  always_ff @(posedge clk) begin
    if (rst) out_valid <= 1'b0;
    else     out_valid <= v_a;
  end

  assign out_acc = acc_o;
endmodule
