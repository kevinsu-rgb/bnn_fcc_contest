// AXI4-Stream image input -> layer-0 ping-pong buffer.
// Binarizes each valid pixel (MSB), compacts valid bits to the LSBs, clips to F0 bits per image,
// packs into PW0-bit words (bit_packer), writes chunk wr_idx++ and commits on flush_done after TLAST.
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
  localparam int K  = BUS_W / DATA_W;
  localparam int KB = DATA_W / 8;
  localparam int PC_W = bits(F0 + 1);

  logic            mid_q, bank_q;
  logic [bits(cdiv(F0,PW0))-1:0] idx_q;
  logic [PC_W-1:0] pix_q;

  logic            gate, pk_ready, pk_valid, pk_flush_done, accept;
  logic [PW0-1:0]  pk_word;
  logic [K-1:0]    c_bits;
  int              n_valid, n_room;
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
    n_room = F0 - int'(pix_q);
    take_w = bits(K+1)'((n_valid < n_room) ? n_valid : n_room);
  end

  assign gate    = mid_q || (accept_new && !buf_full[bank_q]);
  assign s_ready = pk_ready && gate;
  assign accept  = s_valid && gate && pk_ready;

  bit_packer #(.IN_W(K), .OUT_W(PW0), .PAD(1'b0)) pk (
    .clk(clk), .rst(rst), .clear(1'b0),
    .in_valid(s_valid && gate), .in_ready(pk_ready),
    .in_bits(c_bits), .in_count(take_w), .in_flush(s_last),
    .out_valid(pk_valid), .out_word(pk_word), .flush_done(pk_flush_done)
  );

  assign wr_en       = pk_valid;
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
      pix_q  <= '0;
    end else begin
      if (accept) begin
        mid_q <= 1'b1;
        pix_q <= pix_q + PC_W'(take_w);
      end
      if (pk_valid) idx_q <= idx_q + 1'b1;
      if (pk_flush_done) begin
        mid_q  <= 1'b0;
        bank_q <= ~bank_q;
        idx_q  <= '0;
        pix_q  <= '0;
      end
    end
  end
endmodule
