// Self-checking testbench for cfg_layer_writer over several (F, N, PW, PN) configurations.
// Feeds weight/threshold payload bytes (per-neuron ceil(F/8) bytes LSB-first with pad bits 1; thresholds
// 4 bytes little-endian) with random s_valid gaps, captures the config writes into shadow memories and
// checks them against the bnn_layer layout contract. Also exercises abort mid-message.
// idle: must be low from the cycle after start until the message's last write, high from the cycle
// after that write, and high after an abort.
module cw_case import bnn_fcc_pkg::*; #(
  parameter int F  = 8,
  parameter int N  = 8,
  parameter int PW = 8,
  parameter int PN = 8
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int GROUPS  = cdiv(N, PN);
  localparam int PASSES  = cdiv(F, PW);
  localparam int GP      = GROUPS * PASSES;
  localparam int THR_W   = bits(F + 2);
  localparam int LANE_W  = bits(PN);
  localparam int WADDR_W = bits(GP);
  localparam int TADDR_W = bits(GROUPS);
  localparam int BPN     = cdiv(F, 8);

  logic        rst = 1'b1;
  logic        start = 1'b0, abort = 1'b0, msg_type = 1'b0;
  logic        s_valid = 1'b0, s_ready;
  logic [7:0]  s_byte = '0;
  logic                w_we, t_we;
  logic [LANE_W-1:0]   w_lane, t_lane;
  logic [WADDR_W-1:0]  w_addr;
  logic [PW-1:0]       w_data;
  logic [TADDR_W-1:0]  t_addr;
  logic [THR_W-1:0]    t_data;
  logic                idle;

  cfg_layer_writer #(.F(F), .N(N), .PW(PW), .PN(PN), .HAS_THR(1'b1)) dut (.*);

  // ---------------- reference data
  logic wt [N][F];
  int   thr [N];
  byte unsigned bq [$];              // payload bytes of the message being sent

  // ---------------- shadow memories + monitors
  logic [PW-1:0]    wmem [PN][GP];
  bit               wseen [PN][GP];
  logic [THR_W-1:0] tmem [PN][GROUPS];
  bit               tseen [PN][GROUPS];
  int               n_wr, n_tr;
  bit               mon_on = 1'b0;
  bit               idle_chk = 1'b0;    // armed: idle must stay low until the last expected write
  bit               idle_after = 1'b0;  // idle must be high in the cycle after the last write
  int               idle_target;        // writes (weights + thresholds) expected for the armed message

  function automatic int clampf(int v);
    if (v < 0) return 0;
    if (v > F + 1) return F + 1;
    return v;
  endfunction

  always @(posedge clk) if (mon_on) begin : mon
    int g, p, n;
    if (idle_after && idle !== 1'b1) begin
      errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: idle not high after the last write", F, N, PW, PN);
    end
    idle_after = 1'b0;
    if (idle_chk && idle !== 1'b0) begin
      errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: idle high mid-message (%0d/%0d writes)", F, N, PW, PN, n_wr + n_tr, idle_target);
    end
    if (w_we) begin
      g = int'(w_addr) / PASSES; p = int'(w_addr) % PASSES; n = g * PN + int'(w_lane);
      if (int'(w_lane) >= PN || int'(w_addr) >= GP) begin
        errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: weight write out of range lane=%0d addr=%0d", F, N, PW, PN, w_lane, w_addr);
      end else begin
        if (n >= N) begin
          errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: weight write for nonexistent neuron lane=%0d addr=%0d", F, N, PW, PN, w_lane, w_addr);
        end
        if (wseen[w_lane][w_addr]) begin
          errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: duplicate weight write lane=%0d addr=%0d", F, N, PW, PN, w_lane, w_addr);
        end
        wseen[w_lane][w_addr] = 1'b1;
        wmem[w_lane][w_addr] = w_data;
      end
      n_wr++;
    end
    if (t_we) begin
      n = int'(t_addr) * PN + int'(t_lane);
      if (int'(t_lane) >= PN || int'(t_addr) >= GROUPS || n >= N) begin
        errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: threshold write out of range lane=%0d addr=%0d", F, N, PW, PN, t_lane, t_addr);
      end else begin
        if (tseen[t_lane][t_addr]) begin
          errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: duplicate threshold write lane=%0d addr=%0d", F, N, PW, PN, t_lane, t_addr);
        end
        tseen[t_lane][t_addr] = 1'b1;
        tmem[t_lane][t_addr] = t_data;
      end
      n_tr++;
    end
    if (idle_chk && n_wr + n_tr == idle_target) begin
      idle_chk   = 1'b0;
      idle_after = 1'b1;
    end
  end

  task automatic clear_seen();
    for (int l = 0; l < PN; l++) begin
      for (int a = 0; a < GP; a++) wseen[l][a] = 1'b0;
      for (int a = 0; a < GROUPS; a++) tseen[l][a] = 1'b0;
    end
    n_wr = 0; n_tr = 0;
  endtask

  // ---------------- driver
  task automatic pulse(input bit is_abort);
    @(negedge clk);
    s_valid <= 1'b0;
    if (is_abort) abort <= 1'b1; else start <= 1'b1;
    @(negedge clk);
    start <= 1'b0; abort <= 1'b0;
  endtask

  // arm the idle check for a complete message (call right after pulse(0))
  task automatic arm_idle(input int target);
    idle_target = target;
    idle_chk    = 1'b1;
  endtask

  task automatic check_idle_high(string what);
    if (idle !== 1'b1) begin
      errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): idle not high", F, N, PW, PN, what);
    end
    if (idle_chk) begin
      errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): last write never seen (%0d/%0d)", F, N, PW, PN, what, n_wr + n_tr, idle_target);
      idle_chk = 1'b0;
    end
  endtask

  // send first `cnt` bytes of bq with random valid gaps; checks s_ready is independent of s_valid
  task automatic send(input int cnt);
    int  i;
    bit  r0, r1, v;
    i = 0;
    while (i < cnt) begin
      @(negedge clk);
      s_valid <= 1'b0; s_byte <= bq[i];
      #1; r0 = s_ready;
      s_valid <= 1'b1;
      #1; r1 = s_ready;
      if (r0 !== r1) begin
        errors++; $error("F=%0d N=%0d PW=%0d PN=%0d: s_ready depends on s_valid", F, N, PW, PN);
      end
      v = ($urandom_range(3, 0) != 0);
      s_valid <= v;
      @(posedge clk);
      if (v && s_ready) i++;
    end
    @(negedge clk);
    s_valid <= 1'b0; s_byte <= $urandom;
  endtask

  task automatic build_weights();
    bit [7:0] b;
    bq.delete();
    for (int n = 0; n < N; n++)
      for (int k = 0; k < BPN; k++) begin
        b = 8'hFF;
        for (int j = 0; j < 8; j++) if (k * 8 + j < F) b[j] = wt[n][k * 8 + j];
        bq.push_back(b);
      end
  endtask

  task automatic build_thr();
    bit [31:0] v;
    bq.delete();
    for (int n = 0; n < N; n++) begin
      v = 32'(thr[n]);
      for (int k = 0; k < 4; k++) bq.push_back(v[8*k +: 8]);
    end
  endtask

  task automatic check_weights(string what);
    int g, p, n;
    logic [PW-1:0] e;
    if (n_wr != N * PASSES) begin
      errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): %0d weight writes, exp %0d", F, N, PW, PN, what, n_wr, N * PASSES);
    end
    for (int l = 0; l < PN; l++)
      for (int a = 0; a < GP; a++) begin
        g = a / PASSES; p = a % PASSES; n = g * PN + l;
        if (n < N) begin
          for (int k = 0; k < PW; k++) e[k] = (p * PW + k < F) ? wt[n][p * PW + k] : 1'b1;
          if (!wseen[l][a]) begin
            errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): missing weight write lane=%0d addr=%0d", F, N, PW, PN, what, l, a);
          end else if (wmem[l][a] !== e) begin
            errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): lane=%0d addr=%0d exp=%h got=%h", F, N, PW, PN, what, l, a, e, wmem[l][a]);
          end
        end
      end
  endtask

  task automatic check_thr(string what);
    int n;
    if (n_tr != N) begin
      errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): %0d threshold writes, exp %0d", F, N, PW, PN, what, n_tr, N);
    end
    for (int l = 0; l < PN; l++)
      for (int g = 0; g < GROUPS; g++) begin
        n = g * PN + l;
        if (n < N) begin
          if (!tseen[l][g]) begin
            errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): missing threshold write lane=%0d grp=%0d", F, N, PW, PN, what, l, g);
          end else if (tmem[l][g] !== THR_W'(clampf(thr[n]))) begin
            errors++; $error("F=%0d N=%0d PW=%0d PN=%0d (%s): thr n=%0d in=%0d exp=%0d got=%0d", F, N, PW, PN, what, n, thr[n], clampf(thr[n]), tmem[l][g]);
          end
        end
      end
  endtask

  task automatic rnd_weights();
    for (int n = 0; n < N; n++) for (int i = 0; i < F; i++) wt[n][i] = $urandom_range(1, 0);
  endtask

  task automatic rnd_thr(input int off);
    for (int n = 0; n < N; n++) begin
      case ((n + off) % 9)
        0: thr[n] = -5;
        1: thr[n] = 0;
        2: thr[n] = F;
        3: thr[n] = F + 1;
        4: thr[n] = F + 7;
        5: thr[n] = 32'h7FFF_FFFF;
        6: thr[n] = 32'h8000_0000;
        7: thr[n] = $urandom_range(F + 1, 0);
        default: thr[n] = int'($urandom);
      endcase
    end
  endtask

  int cut;
  initial begin
    done = 1'b0; errors = 0;
    for (int l = 0; l < PN; l++) begin
      for (int a = 0; a < GP; a++) wmem[l][a] = '0;
      for (int a = 0; a < GROUPS; a++) tmem[l][a] = '0;
    end
    clear_seen();
    repeat (4) @(negedge clk);
    rst <= 1'b0;
    repeat (2) @(negedge clk);
    check_idle_high("after reset");
    mon_on = 1'b1;

    // weights: clean message
    msg_type <= 1'b0;
    rnd_weights(); build_weights();
    clear_seen();
    pulse(0);
    arm_idle(N * PASSES);
    send(bq.size());
    repeat (12) @(negedge clk);   // drain: up to 8 words from the last byte
    check_idle_high("weights");
    check_weights("clean");

    // weights: aborted message (several cut points), then a full message into the same memories
    for (int r = 0; r < 4; r++) begin
      rnd_weights(); build_weights();
      cut = (r == 0) ? 1 : (r == 1) ? bq.size() - 1 : $urandom_range(bq.size() - 1, 1);
      if (cut < 1) cut = 1;
      clear_seen();
      pulse(0);
      send(cut);
      pulse(1);
      check_idle_high("after weight abort");
      repeat (3) @(negedge clk);
      rnd_weights(); build_weights();
      clear_seen();
      pulse(0);
      arm_idle(N * PASSES);
      send(bq.size());
      repeat (12) @(negedge clk);   // drain: up to 8 words from the last byte
      check_idle_high("weights after abort");
      check_weights($sformatf("after abort@%0d", cut));
    end

    // thresholds: several rounds with rotating value patterns
    msg_type <= 1'b1;
    for (int r = 0; r < 9; r++) begin
      rnd_thr(r); build_thr();
      clear_seen();
      pulse(0);
      arm_idle(N);
      send(bq.size());
      repeat (3) @(negedge clk);
      check_idle_high("thresholds");
      check_thr($sformatf("round %0d", r));
    end
    // thresholds: abort mid-message (cut inside a threshold word), then full
    rnd_thr(1); build_thr();
    clear_seen();
    pulse(0);
    send(bq.size() > 2 ? bq.size() - 2 : 1);
    pulse(1);
    check_idle_high("after threshold abort");
    rnd_thr(3); build_thr();
    clear_seen();
    pulse(0);
    arm_idle(N);
    send(bq.size());
    repeat (3) @(negedge clk);
    check_idle_high("thresholds after abort");
    check_thr("after abort");

    // a weight message after thresholds still works (msg_type switch)
    msg_type <= 1'b0;
    rnd_weights(); build_weights();
    clear_seen();
    pulse(0);
    arm_idle(N * PASSES);
    send(bq.size());
    repeat (12) @(negedge clk);   // drain: up to 8 words from the last byte
    check_idle_high("weights");
    check_weights("after thr");
    if (n_tr != 0) begin errors++; $error("threshold write during weight message"); end

    $display("tb_cfg_layer_writer: F=%0d N=%0d PW=%0d PN=%0d: %0d errors", F, N, PW, PN, errors);
    done = 1'b1;
  end
endmodule

module tb_cfg_layer_writer;
  localparam int NC = 6;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NC-1:0] done;
  int            errs [NC];

  cw_case #(.F(784), .N(256), .PW(66), .PN(32)) c0 (.clk(clk), .done(done[0]), .errors(errs[0]));
  cw_case #(.F(13),  .N(7),   .PW(3),  .PN(2))  c1 (.clk(clk), .done(done[1]), .errors(errs[1]));
  cw_case #(.F(7),   .N(5),   .PW(1),  .PN(4))  c2 (.clk(clk), .done(done[2]), .errors(errs[2]));
  cw_case #(.F(8),   .N(8),   .PW(8),  .PN(8))  c3 (.clk(clk), .done(done[3]), .errors(errs[3]));
  cw_case #(.F(256), .N(10),  .PW(66), .PN(1))  c4 (.clk(clk), .done(done[4]), .errors(errs[4]));
  cw_case #(.F(20),  .N(9),   .PW(64), .PN(9))  c5 (.clk(clk), .done(done[5]), .errors(errs[5]));

  initial begin
    #50ms;
    $error("tb_cfg_layer_writer: timeout");
    $finish;
  end

  int total;
  initial begin
    wait (&done);
    @(posedge clk);
    total = 0;
    for (int i = 0; i < NC; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_cfg_layer_writer");
    else $display("tb_cfg_layer_writer: %0d mismatches", total);
    $finish;
  end
endmodule
