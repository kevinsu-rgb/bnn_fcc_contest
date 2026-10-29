// Self-checking testbench for input_packer (position-based TKEEP). A pingpong_buf captures the writes;
// the TB checks every committed bank against a reference and releases banks after a (random) delay.
// The write index is a one-hot chunk pointer (wr_oh); the buffer stores writes one cycle after the
// handshake, so a commit's content is checked two cycles after commit_en.
// Position rule: pixel i is lane i % K of beat i / K; a TKEEP = 0 lane is stored as a 0 pixel in place.
// MODE 0: back-to-back full beats, immediate release (throughput check)
// MODE 1: random valid gaps, full beats + TKEEP-trimmed last beat, kinds: normal/overlong/early
// MODE 2: random valid gaps, random TKEEP = 0 lanes inside beats, sometimes a trailing TKEEP = 0 TLAST beat
module ip_case import bnn_fcc_pkg::*; #(
  parameter int BUS_W = 64,
  parameter int F0    = 784,
  parameter int MODE  = 1,
  parameter int NIMG  = 30
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int DATA_W = 8;
  localparam int K      = BUS_W / DATA_W;
  localparam int NCH    = cdiv(F0, K);

  logic rst = 1'b1;
  logic s_valid = 1'b0, s_last = 1'b0, s_ready;
  logic [BUS_W-1:0]   s_data = '0;
  logic [BUS_W/8-1:0] s_keep = '0;
  logic accept_new = 1'b0;
  logic [1:0] buf_full;
  logic wr_en, wr_bank, commit_en, commit_bank, mid_image;
  logic [NCH-1:0] wr_oh;
  logic [K-1:0] wr_data;
  logic rls_en = 1'b0, rls_bank = 1'b0;

  input_packer #(.DATA_W(DATA_W), .BUS_W(BUS_W), .F0(F0)) dut (
    .clk(clk), .rst(rst),
    .s_valid(s_valid), .s_ready(s_ready), .s_data(s_data), .s_keep(s_keep), .s_last(s_last),
    .accept_new(accept_new), .buf_full(buf_full),
    .wr_en(wr_en), .wr_bank(wr_bank), .wr_oh(wr_oh), .wr_data(wr_data),
    .commit_en(commit_en), .commit_bank(commit_bank), .mid_image(mid_image)
  );

  pingpong_buf #(.NBITS(F0), .WR_W(K), .RD_W(K)) pp (
    .clk(clk), .rst(rst),
    .wr_en(wr_en), .wr_bank(wr_bank), .wr_oh(wr_oh), .wr_data(wr_data),
    .commit_en(commit_en), .commit_bank(commit_bank),
    .rd_adv(1'b0), .rd_bank(1'b0), .rd_data(),
    .rls_en(rls_en), .rls_bank(rls_bank), .full(buf_full)
  );

  string tag;
  initial tag = $sformatf("BUS_W=%0d F0=%0d MODE=%0d", BUS_W, F0, MODE);

  // ---------------- stimulus generation ----------------
  logic [BUS_W-1:0]   bd [$];
  logic [K-1:0]       bk [$];
  bit                 bl [$];
  logic [F0-1:0]      eimg [$];   // expected stored bits (0 at TKEEP = 0 positions)
  int                 enb  [$];   // number of checked bits (positions covered by the beats sent, <= F0)
  logic [F0-1:0]      eimg_v;
  logic [BUS_W-1:0]   gd;
  logic [K-1:0]       gk;
  logic [7:0]         px;
  int                 npos, nbeats, kind, pos;

  initial begin
    bd.delete(); bk.delete(); bl.delete(); eimg.delete(); enb.delete();
    for (int im = 0; im < NIMG; im++) begin
      kind = (MODE == 0) ? 0 : im % 5;
      if (kind == 2)                npos = F0 + 1 + $urandom_range(0, K + 2);   // overlong
      else if (kind == 3 && F0 > 1) npos = $urandom_range(1, F0 - 1);           // early TLAST
      else                          npos = F0;
      nbeats = cdiv(npos, K);
      eimg_v = '0;
      for (int t = 0; t < nbeats; t++) begin
        gd = {BUS_W/32 + 1{$urandom}} ^ {BUS_W/32 + 1{$urandom}};
        gk = '0;
        for (int l = 0; l < K; l++) begin
          pos = t * K + l;
          if (pos < npos && (MODE != 2 || $urandom_range(0, 99) < 60)) begin
            gk[l] = 1'b1;
            px = 8'($urandom);
            gd[l*DATA_W +: DATA_W] = px;
            if (pos < F0) eimg_v[pos] = px[7];
          end
        end
        bd.push_back(gd); bk.push_back(gk); bl.push_back(t == nbeats - 1);
      end
      if (MODE == 2 && $urandom_range(0, 3) == 0) begin
        // trailing beat with TKEEP = 0 carries TLAST; its positions are stored as 0
        bl[bl.size()-1] = 1'b0;
        bd.push_back({BUS_W/32 + 1{$urandom}}); bk.push_back('0); bl.push_back(1'b1);
        nbeats++;
      end
      eimg.push_back(eimg_v);
      enb.push_back(imin(nbeats * K, F0));
    end
  end

  // ---------------- driver + handshake / ready checks ----------------
  int  bi = 0;
  int  stalls = 0;
  bit  img_active = 1'b0;
  bit  hs, rdy_a, rdy_b;

  initial begin
    errors = 0;
    repeat (4) @(posedge clk);
    #1 rst = 1'b0;
  end

  always @(posedge clk) begin : drv
    if (!rst) begin
      hs = s_valid && s_ready;
      if (mid_image !== img_active) begin
        errors++; $error("%s: mid_image=%b expected %b", tag, mid_image, img_active);
      end
      if (s_ready !== (mid_image || (accept_new && !buf_full[wr_bank]))) begin
        errors++; $error("%s: s_ready=%b, expected mid_image | accept_new & !full (%b %b %b)",
                         tag, s_ready, mid_image, accept_new, buf_full);
      end
      if (commit_en !== (s_valid && s_ready && s_last)) begin
        errors++; $error("%s: commit_en=%b, expected accepted-TLAST %b", tag, commit_en, s_valid && s_ready && s_last);
      end
      if (s_valid && !s_ready) begin
        stalls++;
        if (mid_image) begin errors++; $error("%s: mid-image stall", tag); end
      end
      if (hs) begin
        img_active = 1'b1;
        bi++;
      end
      if (commit_en) img_active = 1'b0;
    end else hs = 1'b0;

    #1;
    if (!rst) begin
      accept_new = (MODE == 0) ? 1'b1 : ($urandom_range(0, 99) < 70);
      if (!s_valid || hs) begin
        if (bi < bd.size() && (MODE == 0 || $urandom_range(0, 99) < 70)) begin
          s_valid = 1'b1; s_data = bd[bi]; s_keep = bk[bi]; s_last = bl[bi];
        end else begin
          s_valid = 1'b0; s_data = {BUS_W/32 + 1{$urandom}}; s_keep = K'($urandom); s_last = $urandom;
        end
      end
    end
    #1 rdy_a = s_ready;
    #1 s_valid = ~s_valid;
    #1 rdy_b = s_ready;
    #1 s_valid = ~s_valid;
    if (!rst && rdy_a !== rdy_b) begin
      errors++; $error("%s: s_ready depends combinationally on s_valid", tag);
    end
  end

  // ---------------- commit / content checks and bank release ----------------
  int  chk_idx = 0;
  int  ncommit = 0;
  int  rel_t [2] = '{-1, -1};
  bit  cm_d = 1'b0, cm_d2 = 1'b0;
  bit  cm_bank_d = 1'b0, cm_bank_d2 = 1'b0;
  bit  exp_bank = 1'b0;
  int  settle = 0;

  always @(posedge clk) begin : chk
    if (rst) begin
      rel_t[0] = -1; rel_t[1] = -1; cm_d = 1'b0; cm_d2 = 1'b0; rls_en <= 1'b0;
    end else begin
      if (wr_en && buf_full[wr_bank]) begin
        errors++; $error("%s: write into a full bank", tag);
      end
      if (wr_en && !$onehot(wr_oh)) begin
        errors++; $error("%s: wr_oh %b not one-hot", tag, wr_oh);
      end
      // delayed content check of the commit seen two cycles ago (its last write is stored by now)
      if (cm_d2) begin
        if (chk_idx >= NIMG) begin
          errors++; $error("%s: extra commit", tag);
        end else begin
          for (int j = 0; j < enb[chk_idx]; j++)
            if (pp.mem[cm_bank_d2][j] !== eimg[chk_idx][j]) begin
              errors++;
              $error("%s: image %0d bit %0d exp=%b got=%b", tag, chk_idx, j,
                     eimg[chk_idx][j], pp.mem[cm_bank_d2][j]);
              break;
            end
          chk_idx++;
        end
        rel_t[cm_bank_d2] = (MODE == 0) ? 0 : $urandom_range(0, 12);
      end
      cm_d2 = cm_d;
      cm_bank_d2 = cm_bank_d;
      cm_d = commit_en;
      cm_bank_d = commit_bank;
      if (commit_en) begin
        ncommit++;
        if (commit_bank !== exp_bank) begin
          errors++; $error("%s: commit to bank %b, expected %b", tag, commit_bank, exp_bank);
        end
        exp_bank = ~exp_bank;
      end
      // release scheduling (one release per cycle)
      rls_en <= 1'b0;
      for (int b = 0; b < 2; b++) if (rel_t[b] > 0) rel_t[b]--;
      for (int b = 0; b < 2; b++)
        if (rel_t[b] == 0) begin
          rls_en <= 1'b1; rls_bank <= 1'(b); rel_t[b] = -1; break;
        end
    end
  end

  // ---------------- completion ----------------
  initial done = 1'b0;
  always @(posedge clk) begin : fin
    if (!rst && !done && chk_idx == NIMG && bi == bd.size()) begin
      settle++;
      if (settle == 30) begin
        if (ncommit != NIMG) begin
          errors++; $error("%s: %0d commits, expected %0d", tag, ncommit, NIMG);
        end
        // back-to-back images only stall at boundaries, and only when a bank is still being released
        if (MODE == 0 && NCH > 2 && stalls > 0) begin
          errors++; $error("%s: %0d stall cycles for %0d images (throughput)", tag, stalls, NIMG);
        end
        done = 1'b1;
      end
    end
  end
endmodule

module tb_input_packer;
  localparam int NCFG = 6;
  localparam int CB [NCFG] = '{64, 8, 128, 64, 32, 64};
  localparam int CF [NCFG] = '{784, 13, 100, 5, 20, 64};   // F0 = 64, BUS 64: NCH = 8 (power of two)

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NCFG*3-1:0] done;
  int                errs [NCFG*3];

  for (genvar c = 0; c < NCFG; c++) begin : g_cfg
    for (genvar m = 0; m < 3; m++) begin : g_mode
      ip_case #(.BUS_W(CB[c]), .F0(CF[c]), .MODE(m), .NIMG(m == 0 ? 12 : 30)) u (
        .clk(clk), .done(done[c*3+m]), .errors(errs[c*3+m]));
    end
  end

  int total;
  initial begin
    wait (&done);
    total = 0;
    for (int i = 0; i < NCFG*3; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_input_packer");
    else $display("FAILED: tb_input_packer with %0d errors", total);
    $finish;
  end

  initial begin
    #50ms;
    $error("tb_input_packer: timeout");
    $finish;
  end
endmodule
