// AXI4-Stream image input -> layer-0 ping-pong buffer.
// Binarizes each valid pixel (MSB), compacts valid bits to the LSBs, packs into PW0-bit words
// (bit_packer), writes chunk wr_idx++ and commits on flush_done after TLAST.
// Overlong images (> F0 pixels) are clipped at word level: words after chunk NCH-1 are not written
// (ovf_q), and bits >= F0 in the last chunk are not stored by the ping-pong buffer.
// s_ready depends on registered state only.
module input_packer import bnn_fcc_pkg::*; #(
  parameter int DATA_W = 8,
  parameter int BUS_W  = 64,
  parameter int F0     = 784,
  parameter int PW0    = 66
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
  output logic [bits(cdiv(F0,PW0))-1:0] wr_idx,
  output logic [PW0-1:0]             wr_data,
  output logic                       commit_en,
  output logic                       commit_bank,
  output logic                       mid_image
);
  localparam int K   = BUS_W / DATA_W;
  localparam int KB  = DATA_W / 8;
  localparam int NCH = cdiv(F0, PW0);

  logic            mid_q, bank_q, ovf_q;
  logic [bits(NCH)-1:0] idx_q;

  logic            gate, pk_ready, pk_valid, pk_flush_done, accept;
  logic [PW0-1:0]  pk_word;
  logic [K-1:0]    c_bits;
  int              n_valid;
  logic [bits(K+1)-1:0] take_w;

  always_comb begin
    c_bits  = '0;
    n_valid = 0;
    for (int j = 0; j < K; j++) begin
      if (&s_keep[j*KB +: KB]) begin
        c_bits[n_valid] = s_data[j*DATA_W + DATA_W - 1];
        n_valid++;
      end
    end
    take_w  = bits(K+1)'(n_valid);
  end

  assign gate    = mid_q || (accept_new && !buf_full[bank_q]);
  assign s_ready = pk_ready && gate;
  assign accept  = s_valid && gate && pk_ready;

  // c_bits is zero above take_w, so the packer's input mask is redundant
  bit_packer #(.IN_W(K), .OUT_W(PW0), .PAD(1'b0), .MASK_IN(1'b0)) pk (
    .clk(clk), .rst(rst), .clear(1'b0),
    .in_valid(s_valid && gate), .in_ready(pk_ready),
    .in_bits(c_bits), .in_count(take_w), .in_flush(s_last),
    .out_valid(pk_valid), .out_word(pk_word), .flush_done(pk_flush_done)
  );

  assign wr_en       = pk_valid && !ovf_q;
  assign wr_bank     = bank_q;
  assign wr_idx      = idx_q;
  assign wr_data     = pk_word;
  assign commit_en   = pk_flush_done;
  assign commit_bank = bank_q;
  assign mid_image   = mid_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      mid_q  <= 1'b0;
      bank_q <= 1'b0;
      idx_q  <= '0;
      ovf_q  <= 1'b0;
    end else begin
      if (accept) mid_q <= 1'b1;
      if (pk_valid) begin
        idx_q <= idx_q + 1'b1;
        if (int'(idx_q) == NCH - 1) ovf_q <= 1'b1;
      end
      if (pk_flush_done) begin
        mid_q  <= 1'b0;
        bank_q <= ~bank_q;
        idx_q  <= '0;
        ovf_q  <= 1'b0;
      end
    end
  end
endmodule
