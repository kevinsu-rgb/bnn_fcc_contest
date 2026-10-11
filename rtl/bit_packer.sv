// Packs variable-width LSB-first bit groups into fixed OUT_W-bit words.
// Consumer always accepts. in_ready depends on registered state only.
module bit_packer import bnn_fcc_pkg::*; #(
  parameter int IN_W  = 8,
  parameter int OUT_W = 8,
  parameter bit PAD   = 1'b0
) (
  input  logic                     clk,
  input  logic                     rst,
  input  logic                     clear,
  input  logic                     in_valid,
  output logic                     in_ready,
  input  logic [IN_W-1:0]          in_bits,
  input  logic [bits(IN_W+1)-1:0]  in_count,
  input  logic                     in_flush,
  output logic                     out_valid,
  output logic [OUT_W-1:0]         out_word,
  output logic                     flush_done
);
  localparam int BUF_W = OUT_W + IN_W;
  localparam int CNT_W = bits(BUF_W + 1);

  logic [BUF_W-1:0] buf_q;
  logic [CNT_W-1:0] cnt_q;
  logic             pend_q;

  logic             emit;
  logic [CNT_W-1:0] post_cnt;
  logic [BUF_W-1:0] post_buf;
  logic             pad_emit;
  logic             accept;
  logic [IN_W-1:0]  in_masked;
  logic [BUF_W-1:0] app_buf;

  always_comb begin
    emit     = (int'(cnt_q) >= OUT_W);
    post_cnt = emit ? CNT_W'(int'(cnt_q) - OUT_W) : cnt_q;
    post_buf = emit ? (buf_q >> OUT_W) : buf_q;
    pad_emit = pend_q && !emit && (cnt_q != '0);

    in_ready   = (int'(post_cnt) < OUT_W) && !pend_q;
    accept     = in_valid && in_ready;
    out_valid  = emit || pad_emit;
    flush_done = pend_q && (emit ? (post_cnt == '0) : 1'b1);

    for (int i = 0; i < OUT_W; i++)
      out_word[i] = (emit || int'(cnt_q) > i) ? buf_q[i] : PAD;

    for (int i = 0; i < IN_W; i++)
      in_masked[i] = in_bits[i] && (i < int'(in_count));
    app_buf = BUF_W'(in_masked) << post_cnt;
  end

  always_ff @(posedge clk) begin
    if (rst || clear) begin
      buf_q  <= '0;
      cnt_q  <= '0;
      pend_q <= 1'b0;
    end else begin
      if (accept) begin
        buf_q  <= post_buf | app_buf;
        cnt_q  <= post_cnt + CNT_W'(in_count);
        pend_q <= in_flush;
      end else begin
        buf_q <= pad_emit ? '0 : post_buf;
        cnt_q <= pad_emit ? '0 : post_cnt;
        if (flush_done) pend_q <= 1'b0;
      end
    end
  end
endmodule
