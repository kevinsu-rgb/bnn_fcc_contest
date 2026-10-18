// Replicated-register broadcast tree: dout[i] = din delayed by bcast_depth(N_OUT)
// cycles, with every register driving at most BCAST_FANOUT loads.
//
// Stage k (0-based, d = bcast_depth(N_OUT) stages) holds
// ceil(N_OUT / BCAST_FANOUT**(d-1-k)) registers; register j of stage k is
// loaded from register j / BCAST_FANOUT of stage k-1 (stage 0 from din). The
// last stage has one register per output. keep/shreg_extract stop Vivado from
// merging the replicas or turning the chain into shift registers.
module bnn_bcast import bnn_fcc_pkg::*; #(
    parameter int WIDTH = 8,
    parameter int N_OUT = 128
) (
    input  logic             clk,
    input  logic [WIDTH-1:0] din,
    output logic [WIDTH-1:0] dout[N_OUT]
);

  localparam int D = bcast_depth(N_OUT);

  // Registers in stage k.
  function automatic int stage_size(int k);
    int s;
    s = N_OUT;
    for (int i = 0; i < D - 1 - k; i++) s = ceil_div(s, BCAST_FANOUT);
    return s;
  endfunction

  for (genvar k = 0; k < D; k++) begin : g_stage
    localparam int S = stage_size(k);
    (* keep = "true", shreg_extract = "no" *) logic [WIDTH-1:0] r[S];
    for (genvar j = 0; j < S; j++) begin : g_r
      if (k == 0) begin : g_first
        always_ff @(posedge clk) r[j] <= din;
      end else begin : g_next
        always_ff @(posedge clk) r[j] <= g_stage[k-1].r[j/BCAST_FANOUT];
      end
    end
  end

  for (genvar i = 0; i < N_OUT; i++) begin : g_o
    assign dout[i] = g_stage[D-1].r[i];
  end

endmodule
