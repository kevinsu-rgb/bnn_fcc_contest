// Splits AXI4-Stream config beats into a stream of single bytes (lowest kept lane first).
// Holds one beat; a new beat is accepted only when the holder is empty (s_ready is a function of
// registered state only). Config throughput is 1 byte per cycle.
//   m_end_of_beat : this item is the last kept byte of the beat (or the null item)
//   m_beat_last   : s_last of the beat, presented on its last item
//   m_null        : beat had TKEEP == 0 and TLAST == 1 -> single item carrying only TLAST
//   drop_rest     : with an m_valid && m_ready handshake, discards the remainder of the current beat
// The item outputs (m_byte, m_null, m_end_of_beat, m_beat_last) are registers loaded with the values
// of the next held state, so their timing is identical to a combinational decode of the held beat.
// The candidates for "advance" (keep_q with its lowest lane cleared) and "load" (the incoming beat)
// are decoded in parallel from registers and inputs; the handshake only selects between them.
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

  typedef struct packed {
    logic [7:0] b;      // byte of the lowest kept lane
    logic       nul;    // no kept lane
    logic       eob;    // at most one kept lane
    logic       blast;  // last && eob
  } item_t;

  function automatic item_t decode(input logic [NB-1:0] keep, input logic [BUS_W-1:0] data,
                                   input logic last);
    item_t it;
    it.b = '0;
    for (int i = NB - 1; i >= 0; i--) if (keep[i]) it.b = data[8*i +: 8];   // lowest set lane wins
    it.nul   = (keep == '0);
    it.eob   = ((keep & (keep - 1'b1)) == '0);
    it.blast = last && it.eob;
    return it;
  endfunction

  logic [BUS_W-1:0] data_q;
  logic [NB-1:0]    keep_q;    // bytes of the held beat not yet emitted
  logic             last_q;
  logic             full_q;
  item_t            it_q;      // decoded outputs of the held state

  logic [NB-1:0]    keep_rest;
  logic             load;
  item_t            it_adv, it_load;

  assign keep_rest = keep_q & (keep_q - 1'b1);   // clear lowest set bit
  assign it_adv    = decode(keep_rest, data_q, last_q);
  assign it_load   = decode(s_keep, s_data, s_last);

  assign load      = !full_q && s_valid && (s_keep != '0 || s_last);

  assign s_ready       = !full_q;
  assign m_valid       = full_q;
  assign m_null        = it_q.nul;
  assign m_byte        = it_q.b;
  assign m_end_of_beat = it_q.eob;
  assign m_beat_last   = it_q.blast;

  always_ff @(posedge clk) begin
    if (rst) begin
      full_q <= 1'b0;
      keep_q <= '0;
      last_q <= 1'b0;
      data_q <= '0;
      it_q   <= '{b: '0, nul: 1'b1, eob: 1'b1, blast: 1'b0};
    end else if (full_q) begin
      if (m_ready) begin
        if (it_q.eob || drop_rest) full_q <= 1'b0;
        else begin
          keep_q <= keep_rest;
          it_q   <= it_adv;
        end
      end
    end else if (load) begin
      full_q <= 1'b1;
      data_q <= s_data;
      keep_q <= s_keep;
      last_q <= s_last;
      it_q   <= it_load;
    end
  end
endmodule
