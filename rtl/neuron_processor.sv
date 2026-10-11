// Neuron processor: XNOR -> pipelined popcount -> accumulate over passes -> threshold compare.
// out_valid pulses np_latency(PW, POPCOUNT_LEVELS_PER_STAGE) cycles after an in_valid&&last_pass input.
// thr is presented in the same cycle as x/w (delayed internally to align with the popcount result).
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

  logic [PW-1:0]   xn;
  logic [PC_W-1:0] pc;
  assign xn = ~(x ^ w);

  popcount #(.W(PW), .LEVELS_PER_STAGE(POPCOUNT_LEVELS_PER_STAGE)) u_pc (
    .clk(clk), .in_bits(xn), .count(pc)
  );

  // Side-band delay line, PC_LAT stages (PC_LAT >= 1).
  logic             v_d [PC_LAT];
  logic             f_d [PC_LAT];
  logic             l_d [PC_LAT];
  logic [THR_W-1:0] t_d [PC_LAT];

  always_ff @(posedge clk) begin
    f_d[0] <= first_pass;
    l_d[0] <= last_pass;
    t_d[0] <= thr;
    for (int i = 1; i < PC_LAT; i++) begin
      f_d[i] <= f_d[i-1];
      l_d[i] <= l_d[i-1];
      t_d[i] <= t_d[i-1];
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < PC_LAT; i++) v_d[i] <= 1'b0;
    end else begin
      v_d[0] <= in_valid;
      for (int i = 1; i < PC_LAT; i++) v_d[i] <= v_d[i-1];
    end
  end

  logic [ACC_W-1:0] acc;
  logic [ACC_W-1:0] sum;
  assign sum = f_d[PC_LAT-1] ? ACC_W'(pc) : (acc + ACC_W'(pc));

  always_ff @(posedge clk) begin
    if (v_d[PC_LAT-1]) begin
      acc   <= sum;
      out_y <= (CMP_W'(sum) >= CMP_W'(t_d[PC_LAT-1]));
    end
  end

  always_ff @(posedge clk) begin
    if (rst) out_valid <= 1'b0;
    else     out_valid <= v_d[PC_LAT-1] & l_d[PC_LAT-1];
  end

  assign out_acc = acc;
endmodule
