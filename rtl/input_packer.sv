// AXI4-Stream image input -> layer-0 ping-pong buffer (position-based TKEEP).
// Pixel i is lane i % K of beat i / K. Each accepted beat writes its K binarized lanes (pixel MSB,
// 0 where TKEEP = 0) as ping-pong chunk wr_idx++, and the TLAST beat commits the bank in the same
// cycle. The chunk index is a one-hot pointer (wr_oh). Overlong images are clipped: beats after
// chunk NCH-1 are not written (ovf_q), and bits >= F0 in the last chunk are not stored by the
// ping-pong buffer.
// One beat per cycle; s_ready depends on registered state only.
module input_packer import bnn_fcc_pkg::*; #(
  parameter int DATA_W = 8,
  parameter int BUS_W  = 64,
  parameter int F0     = 784
) (
  input  logic                       clk,
  input  logic                       rst,
  input  logic                       s_valid,
  output logic                       s_ready,
  input  logic [BUS_W-1:0]           s_data,
  input  logic [BUS_W/8-1:0]         s_keep,
  input  logic                       s_last,
  input  logic                       accept_new,
  input  logic [1:0]                 buf_full,
  output logic                       wr_en,
  output logic                       wr_bank,
  output logic [cdiv(F0,BUS_W/DATA_W)-1:0] wr_oh,
  output logic [BUS_W/DATA_W-1:0]    wr_data,
  output logic                       commit_en,
  output logic                       commit_bank,
  output logic                       mid_image
);
  localparam int K   = BUS_W / DATA_W;
  localparam int KB  = DATA_W / 8;
  localparam int NCH = cdiv(F0, K);

  logic           mid_q, bank_q, ovf_q;
  logic [NCH-1:0] oh_q;   // one-hot chunk pointer
  logic           gate, accept;

  for (genvar j = 0; j < K; j++) begin : g_lane
    assign wr_data[j] = s_data[j*DATA_W + DATA_W - 1] && (&s_keep[j*KB +: KB]);
  end

  assign gate    = mid_q || (accept_new && !buf_full[bank_q]);
  assign s_ready = gate;
  assign accept  = s_valid && gate;

  assign wr_en       = accept && !ovf_q;
  assign wr_bank     = bank_q;
  assign wr_oh       = oh_q;
  assign commit_en   = accept && s_last;
  assign commit_bank = bank_q;
  assign mid_image   = mid_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      mid_q  <= 1'b0;
      bank_q <= 1'b0;
      oh_q   <= NCH'(1);
      ovf_q  <= 1'b0;
    end else if (commit_en) begin
      mid_q  <= 1'b0;
      bank_q <= ~bank_q;
      oh_q   <= NCH'(1);
      ovf_q  <= 1'b0;
    end else if (accept) begin
      mid_q <= 1'b1;
      if (!ovf_q) begin
        oh_q <= oh_q << 1;
        if (oh_q[NCH-1]) ovf_q <= 1'b1;
      end
    end
  end
endmodule
