// Self-checking testbench for cfg_byte_serializer over several bus widths.
// Random beats (25% full keep, 25% zero keep, 50% sparse keep), random last, random s_valid gaps,
// random m_ready, random drop_rest on 10% of output handshakes. A reference model builds the expected
// item queue at each input handshake (honouring drops at output handshakes). A directed phase checks
// that s_ready has no combinational dependence on s_valid.
module cbs_case #(
  parameter int BUS_W = 64,
  parameter int NBEATS = 2000
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int NB = BUS_W / 8;

  logic              rst = 1'b1;
  logic              s_valid = 1'b0, s_ready;
  logic [BUS_W-1:0]  s_data = '0;
  logic [NB-1:0]     s_keep = '0;
  logic              s_last = 1'b0;
  logic              m_valid, m_ready = 1'b0;
  logic [7:0]        m_byte;
  logic              m_null, m_end_of_beat, m_beat_last;
  logic              drop_rest = 1'b0;

  cfg_byte_serializer #(.BUS_W(BUS_W)) dut (.*);

  typedef struct {
    bit [7:0] b;
    bit       null_i;
    bit       eob;
    bit       bl;
    int       beat;
  } item_t;

  item_t q [$];
  int    beat_id = 0;
  int    sent = 0;
  int    n_items_out = 0;
  bit    directed = 1'b0;
  bit    mon_on = 1'b0;
  bit    rand_ok = 1'b1;       // random driver may issue new beats

  function automatic logic [NB-1:0] rnd_keep();
    int r;
    r = $urandom_range(3, 0);
    if (r == 0) return '1;
    if (r == 1) return '0;
    return NB'($urandom());
  endfunction

  // ---------------- driver + monitor (all in one clocked block; sample pre-edge, drive with NBA)
  always @(posedge clk) begin : mon
    item_t it;
    int    kcnt;
    if (!rst && mon_on) begin
      // ---- input handshake -> push expected items
      if (s_valid && s_ready) begin
        kcnt = 0;
        for (int i = 0; i < NB; i++) if (s_keep[i]) kcnt++;
        if (kcnt == 0) begin
          if (s_last) begin
            it.b = 8'h00; it.null_i = 1'b1; it.eob = 1'b1; it.bl = 1'b1; it.beat = beat_id;
            q.push_back(it);
          end
        end else begin
          int seen;
          seen = 0;
          for (int i = 0; i < NB; i++) if (s_keep[i]) begin
            seen++;
            it.b = s_data[8*i +: 8]; it.null_i = 1'b0; it.eob = (seen == kcnt);
            it.bl = s_last && (seen == kcnt); it.beat = beat_id;
            q.push_back(it);
          end
        end
        beat_id++;
        sent++;
      end
      // ---- output handshake -> compare
      if (m_valid && m_ready) begin
        if (q.size() == 0) begin
          errors++; $error("BUS_W=%0d: unexpected output item byte=%02h", BUS_W, m_byte);
        end else begin
          it = q.pop_front();
          n_items_out++;
          if (m_null !== it.null_i || m_end_of_beat !== it.eob || m_beat_last !== it.bl ||
              (!it.null_i && m_byte !== it.b)) begin
            errors++;
            $error("BUS_W=%0d: item mismatch got byte=%02h null=%b eob=%b bl=%b exp byte=%02h null=%b eob=%b bl=%b",
                   BUS_W, m_byte, m_null, m_end_of_beat, m_beat_last, it.b, it.null_i, it.eob, it.bl);
          end
          if (drop_rest) while (q.size() != 0 && q[0].beat == it.beat) void'(q.pop_front());
        end
      end
      // ---- m_valid must not drop without handshake, m_byte stable (checked via item compare)
    end
    // ---- drivers
    if (!directed) begin
      if (!s_valid || s_ready) begin
        if (rand_ok && sent < NBEATS && ($urandom_range(3, 0) != 0)) begin
          s_valid <= 1'b1;
          s_keep  <= rnd_keep();
          s_last  <= ($urandom_range(2, 0) == 0);
          for (int i = 0; i < NB; i++) s_data[8*i +: 8] <= 8'($urandom());
        end else begin
          s_valid <= 1'b0;
          s_keep  <= NB'($urandom());   // garbage while invalid
          s_data  <= {BUS_W{1'b1}} ^ s_data;
        end
      end
      m_ready   <= ($urandom_range(2, 0) != 0);
      drop_rest <= ($urandom_range(9, 0) == 0);
    end
  end

  // post-handshake sanity: a beat with sent count stops when NBEATS reached
  initial begin
    done = 1'b0;
    errors = 0;
    repeat (4) @(posedge clk);
    rst <= 1'b0;
    mon_on = 1'b1;    // the monitor itself ignores edges while rst is high
    wait (sent >= NBEATS);
    // drain
    repeat (4) @(posedge clk);
    m_ready <= 1'b1;
    begin : drain
      int guard;
      guard = 0;
      while ((q.size() != 0 || m_valid) && guard < 100000) begin
        @(posedge clk);
        guard++;
        m_ready <= 1'b1; drop_rest <= 1'b0;
      end
    end
    // random driver still toggling m_ready/drop; freeze it for directed phase
    @(posedge clk);
    if (q.size() != 0 || m_valid) begin
      errors++; $error("BUS_W=%0d: not drained, %0d items left", BUS_W, q.size());
    end
    directed = 1'b1;
    rand_ok = 1'b0;
    s_valid <= 1'b0; m_ready <= 1'b0; drop_rest <= 1'b0;
    @(posedge clk);
    // ---- directed: s_ready independent of s_valid
    for (int c = 0; c < 200; c++) begin : dir
      logic r0, r1;
      @(negedge clk);
      s_valid = 1'b0;
      #1 r0 = s_ready;
      s_keep = rnd_keep(); s_last = ($urandom_range(1, 0) == 1);
      for (int i = 0; i < NB; i++) s_data[8*i +: 8] = 8'($urandom());
      s_valid = 1'b1;
      #1 r1 = s_ready;
      if (r0 !== r1) begin
        errors++; $error("BUS_W=%0d: s_ready depends combinationally on s_valid (cycle %0d)", BUS_W, c);
      end
      m_ready = ($urandom_range(2, 0) == 0);
      drop_rest = ($urandom_range(9, 0) == 0);
    end
    @(negedge clk);
    s_valid = 1'b0;
    // drain again
    begin : drain2
      int guard;
      guard = 0;
      m_ready = 1'b1; drop_rest = 1'b0;
      while ((q.size() != 0 || m_valid) && guard < 100000) begin
        @(posedge clk);
        guard++;
      end
    end
    @(posedge clk);
    if (q.size() != 0 || m_valid) begin
      errors++; $error("BUS_W=%0d: not drained after directed phase", BUS_W);
    end
    if (n_items_out == 0) begin errors++; $error("BUS_W=%0d: no items seen", BUS_W); end
    $display("tb_cfg_byte_serializer: BUS_W=%0d: %0d beats, %0d items, %0d errors", BUS_W, sent, n_items_out, errors);
    done = 1'b1;
  end
endmodule

module tb_cfg_byte_serializer;
  localparam int NC = 4;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NC-1:0] done;
  int            errs [NC];

  cbs_case #(.BUS_W(8))   c0 (.clk(clk), .done(done[0]), .errors(errs[0]));
  cbs_case #(.BUS_W(32))  c1 (.clk(clk), .done(done[1]), .errors(errs[1]));
  cbs_case #(.BUS_W(64))  c2 (.clk(clk), .done(done[2]), .errors(errs[2]));
  cbs_case #(.BUS_W(128)) c3 (.clk(clk), .done(done[3]), .errors(errs[3]));

  initial begin
    #50ms;
    $error("tb_cfg_byte_serializer: timeout");
    $finish;
  end

  int total;
  initial begin
    wait (&done);
    @(posedge clk);
    total = 0;
    for (int i = 0; i < NC; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_cfg_byte_serializer");
    else $display("tb_cfg_byte_serializer: %0d mismatches", total);
    $finish;
  end
endmodule
