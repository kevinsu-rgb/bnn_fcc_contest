// Self-checking testbench for axis_out: order, no loss/dup, AXIS hold, keep/last.
module ao_case import bnn_fcc_pkg::*; #(
  parameter int DATA_W = 4,
  parameter int BUS_W  = 8,
  parameter int DEPTH  = 4,
  parameter int PPUSH  = 50,   // push probability, percent
  parameter int PREADY = 50    // m_ready probability, percent
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int NITEMS = 3000;

  logic                    rst = 1'b1;
  logic                    push = 1'b0;
  logic [DATA_W-1:0]       push_data = '0;
  logic [bits(DEPTH+1)-1:0] count;
  logic                    m_valid;
  logic                    m_ready = 1'b0;
  logic [BUS_W-1:0]        m_data;
  logic [BUS_W/8-1:0]      m_keep;
  logic                    m_last;

  axis_out #(.DATA_W(DATA_W), .BUS_W(BUS_W), .DEPTH(DEPTH)) dut (.*);

  localparam logic [BUS_W/8-1:0] KEEP_EXP = (BUS_W/8)'((1 << cdiv(DATA_W, 8)) - 1);

  logic [DATA_W-1:0] exp_q [$];
  int  sent = 0, got = 0;
  bit  checking = 1'b0;
  bit  prev_stall = 1'b0;
  logic [BUS_W-1:0] prev_data;
  int  occ = 0;              // model of entries held

  always @(posedge clk) if (checking) begin
    // AXIS hold rule
    if (prev_stall) begin
      if (m_valid !== 1'b1 || m_data !== prev_data) begin
        errors++;
        $error("DEPTH=%0d: m_valid/m_data changed without handshake", DEPTH);
      end
    end
    if (m_valid === 1'b1) begin
      if (m_keep !== KEEP_EXP || m_last !== 1'b1) begin
        errors++;
        $error("DEPTH=%0d: keep=%h last=%b", DEPTH, m_keep, m_last);
      end
      if (m_ready) begin
        if (exp_q.size() == 0) begin
          errors++;
          $error("DEPTH=%0d: output with empty model", DEPTH);
        end else begin
          if (m_data !== BUS_W'(exp_q[0])) begin
            errors++;
            $error("DEPTH=%0d: item %0d exp=%h got=%h", DEPTH, got, exp_q[0], m_data);
          end
          void'(exp_q.pop_front());
        end
        got++;
        occ--;
      end
      prev_stall = !m_ready;
      prev_data  = m_data;
    end else begin
      prev_stall = 1'b0;
    end
    if (push) occ++;
  end

  // count check: sampled just before the posedge, model occ reflects state after previous edge
  always @(negedge clk) if (checking) begin
    if (count !== (bits(DEPTH+1))'(occ)) begin
      errors++;
      $error("DEPTH=%0d: count=%0d exp=%0d", DEPTH, count, occ);
    end
    if (count > DEPTH) begin
      errors++;
      $error("DEPTH=%0d: count above DEPTH", DEPTH);
    end
  end

  initial begin
    done = 1'b0;
    errors = 0;
    repeat (3) @(negedge clk);
    rst = 1'b0;
    @(negedge clk);
    checking = 1'b1;
    while (got < NITEMS) begin
      // drive for the coming posedge
      m_ready = ($urandom_range(1, 100) <= PREADY);
      if (sent < NITEMS && count < DEPTH && ($urandom_range(1, 100) <= PPUSH)) begin
        push = 1'b1;
        push_data = DATA_W'($urandom);
        exp_q.push_back(push_data);
        sent++;
      end else begin
        push = 1'b0;
        push_data = DATA_W'($urandom);
      end
      @(negedge clk);
    end
    push = 1'b0;
    repeat (5) @(negedge clk);
    if (exp_q.size() != 0 || m_valid !== 1'b0 || count !== '0) begin
      errors++;
      $error("DEPTH=%0d: leftover %0d valid=%b count=%0d", DEPTH, exp_q.size(), m_valid, count);
    end
    done = 1'b1;
  end
endmodule

module tb_axis_out;
  localparam int NC = 6;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NC-1:0] done;
  int            errs [NC];

  ao_case #(.DATA_W(4), .BUS_W(8), .DEPTH(1), .PPUSH(50), .PREADY(50)) c0 (.clk(clk), .done(done[0]), .errors(errs[0]));
  ao_case #(.DATA_W(4), .BUS_W(8), .DEPTH(4), .PPUSH(50), .PREADY(50)) c1 (.clk(clk), .done(done[1]), .errors(errs[1]));
  ao_case #(.DATA_W(4), .BUS_W(8), .DEPTH(4), .PPUSH(90), .PREADY(20)) c2 (.clk(clk), .done(done[2]), .errors(errs[2]));
  ao_case #(.DATA_W(4), .BUS_W(8), .DEPTH(4), .PPUSH(20), .PREADY(90)) c3 (.clk(clk), .done(done[3]), .errors(errs[3]));
  ao_case #(.DATA_W(10), .BUS_W(16), .DEPTH(3), .PPUSH(70), .PREADY(60)) c4 (.clk(clk), .done(done[4]), .errors(errs[4]));
  ao_case #(.DATA_W(4), .BUS_W(8), .DEPTH(1), .PPUSH(100), .PREADY(100)) c5 (.clk(clk), .done(done[5]), .errors(errs[5]));

  int total;
  initial begin
    fork
      wait (&done);
      begin #50_000_000; $error("tb_axis_out: watchdog timeout"); $finish; end
    join_any
    @(posedge clk);
    total = 0;
    for (int i = 0; i < NC; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_axis_out");
    else $display("tb_axis_out: %0d mismatches", total);
    $finish;
  end
endmodule
