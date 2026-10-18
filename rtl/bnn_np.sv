// Neuron processor: XNOR-popcount -> accumulator -> registered threshold compare.
//
// A neuron is presented as C contiguous cycles (in_first on the first, in_last
// on the last, in_thr valid with in_last). out_count (total popcount) and
// out_bit (out_count >= thr) are valid np_latency(PW) cycles after the in_last
// cycle, both from registers. Neurons may follow back-to-back. No valid, no
// reset: the pipeline is pure datapath.
//
// Pipeline (t = in_last cycle):
//   t .. t+PL-1  bnn_popcount (PL = popcount_latency(PW)); in_first/in_thr are
//                delayed alongside it.
//   t+PL         pc of the last chunk is presented; acc <= first ? pc : acc + pc
//   t+PL+1       acc holds the neuron total; out_count <= acc, out_bit <= acc >= thr
//   t+PL+2       outputs valid (= np_latency(PW))
module bnn_np import bnn_fcc_pkg::*; #(
    parameter int PW = 16,
    parameter int AW = 10,
    parameter int TW = 10
) (
    input  logic          clk,
    input  logic          in_first,
    // in_last is not needed: in_thr is delayed unconditionally, so the threshold
    // sampled at the neuron's last cycle reaches the compare without a capture enable.
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic          in_last,
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [PW-1:0] in_x,
    input  logic [PW-1:0] in_w,
    input  logic [TW-1:0] in_thr,
    output logic          out_bit,
    output logic [AW-1:0] out_count
);

  localparam int PL = popcount_latency(PW);
  localparam int CW = width_for(PW);
  localparam int CMPW = max_int(AW, TW);

  logic [CW-1:0] pc;

  bnn_popcount #(
      .WIDTH(PW)
  ) u_pop (
      .clk  (clk),
      .a    (in_x),
      .b    (in_w),
      .count(pc)
  );

  // Delay in_first and in_thr by PL cycles (plain FFs, no SRL inference), plus one
  // more stage for the threshold so it lines up with the registered accumulator.
  (* shreg_extract = "no" *) logic            first_sr[PL];
  (* shreg_extract = "no" *) logic [TW-1:0]   thr_sr  [PL+1];

  always_ff @(posedge clk) begin
    first_sr[0] <= in_first;
    thr_sr[0]   <= in_thr;
    for (int i = 1; i < PL; i++) first_sr[i] <= first_sr[i-1];
    for (int i = 1; i <= PL; i++) thr_sr[i] <= thr_sr[i-1];
  end

  logic first_d;
  logic [TW-1:0] thr_d;
  assign first_d = first_sr[PL-1];
  assign thr_d   = thr_sr[PL];

  // Accumulator.
  logic [AW-1:0] acc;
  always_ff @(posedge clk) acc <= first_d ? AW'(pc) : acc + AW'(pc);

  // Registered outputs, aligned on the same cycle.
  logic [CMPW-1:0] acc_x;
  logic [CMPW-1:0] thr_x;
  assign acc_x = CMPW'(acc);
  assign thr_x = CMPW'(thr_d);

  always_ff @(posedge clk) begin
    out_count <= acc;
    out_bit   <= (acc_x >= thr_x);
  end

endmodule
