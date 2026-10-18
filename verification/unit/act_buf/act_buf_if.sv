// Port bundle of bnn_act_buf (clock supplied by the testbench top).
interface act_buf_if #(
    parameter int WIDTH = 16,
    parameter int AW    = 6
) (
    input logic clk
);
  logic             rst;
  logic             wr_can_claim;
  logic             wr_claim;
  logic             wr_en;
  logic [   AW-1:0] wr_addr;
  logic [WIDTH-1:0] wr_data;
  logic             wr_commit;
  logic             rd_full;
  logic             rd_en;
  logic [   AW-1:0] rd_addr;
  logic [WIDTH-1:0] rd_data;
  logic             rd_release;
  logic             idle;
endinterface
