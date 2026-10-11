// Splits AXI4-Stream config beats into a stream of single bytes (lowest kept lane first).
// Holds one beat; a new beat is accepted only when the holder is empty (s_ready is a function of
// registered state only). Config throughput is 1 byte per cycle.
//   m_end_of_beat : this item is the last kept byte of the beat (or the null item)
//   m_beat_last   : s_last of the beat, presented on its last item
//   m_null        : beat had TKEEP == 0 and TLAST == 1 -> single item carrying only TLAST
//   drop_rest     : with an m_valid && m_ready handshake, discards the remainder of the current beat
module cfg_byte_serializer #(
  parameter int BUS_W = 64
) (
  input  logic                clk,
  input  logic                rst,

  input  logic                s_valid,
  output logic                s_ready,
  input  logic [BUS_W-1:0]    s_data,
  input  logic [BUS_W/8-1:0]  s_keep,
  input  logic                s_last,

  output logic                m_valid,
  input  logic                m_ready,
  output logic [7:0]          m_byte,
  output logic                m_null,
  output logic                m_end_of_beat,
  output logic                m_beat_last,
  input  logic                drop_rest
);
  localparam int NB = BUS_W / 8;

  logic [BUS_W-1:0] data_q;
  logic [NB-1:0]    keep_q;    // bytes of the held beat not yet emitted
  logic             last_q;
  logic             full_q;

  logic [NB-1:0]    keep_rest;
  logic [NB-1:0]    keep_next;
  logic [7:0]       sel_byte;

  assign keep_rest = keep_q & (keep_q - 1'b1);   // clear lowest set bit
  assign keep_next = keep_rest;

  always_comb begin
    sel_byte = '0;
    for (int i = NB - 1; i >= 0; i--) begin
      if (keep_q[i]) sel_byte = data_q[8*i +: 8];   // lowest set lane wins (last assignment)
    end
  end

  assign s_ready       = !full_q;
  assign m_valid       = full_q;
  assign m_null        = (keep_q == '0);
  assign m_byte        = sel_byte;
  assign m_end_of_beat = (keep_rest == '0);
  assign m_beat_last   = last_q && m_end_of_beat;

  always_ff @(posedge clk) begin
    if (rst) begin
      full_q <= 1'b0;
      keep_q <= '0;
      last_q <= 1'b0;
      data_q <= '0;
    end else if (full_q) begin
      if (m_ready) begin
        if (m_end_of_beat || drop_rest) full_q <= 1'b0;
        else keep_q <= keep_next;
      end
    end else if (s_valid && (s_keep != '0 || s_last)) begin
      full_q <= 1'b1;
      data_q <= s_data;
      keep_q <= s_keep;
      last_q <= s_last;
    end
  end
endmodule
