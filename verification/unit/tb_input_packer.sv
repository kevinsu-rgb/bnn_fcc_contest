// Self-checking testbench for input_packer. A pingpong_buf captures the writes; the TB checks every
// committed bank against a reference and releases banks after a (random) delay.
// MODE 0: back-to-back full beats, immediate release (throughput check)
// MODE 1: random valid gaps, full beats + TKEEP-trimmed last beat, kinds: normal/extra/early
// MODE 2: random valid gaps, sparse TKEEP inside beats
module ip_case import bnn_fcc_pkg::*; #(
  parameter int BUS_W = 64,
  parameter int F0    = 784,
  parameter int PW0   = 66,
  parameter int MODE  = 1,
  parameter int NIMG  = 30
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int DATA_W = 8;
  localparam int K      = BUS_W / DATA_W;
  localparam int NCH    = cdiv(F0, PW0);

  logic rst = 1'b1;
  logic s_valid = 1'b0, s_last = 1'b0, s_ready;
  logic [BUS_W-1:0]   s_data = '0;
  logic [BUS_W/8-1:0] s_keep = '0;
  logic accept_new = 1'b0;
  logic [1:0] buf_full;
  logic wr_en, wr_bank, commit_en, commit_bank, mid_image;
  logic [bits(NCH)-1:0] wr_idx;
  logic [PW0-1:0] wr_data;
  logic rls_en = 1'b0, rls_bank = 1'b0;

  input_packer #(.DATA_W(DATA_W), .BUS_W(BUS_W), .F0(F0), .PW0(PW0)) dut (
    .clk(clk), .rst(rst),
    .s_valid(s_valid), .s_ready(s_ready), .s_data(s_data), .s_keep(s_keep), .s_last(s_last),
    .accept_new(accept_new), .buf_full(buf_full),
    .wr_en(wr_en), .wr_bank(wr_bank), .wr_idx(wr_idx), .wr_data(wr_data),
    .commit_en(commit_en), .commit_bank(commit_bank), .mid_image(mid_image)
  );

  pingpong_buf #(.NBITS(F0), .WR_W(PW0), .RD_W(PW0)) pp (
    .clk(clk), .rst(rst),
    .wr_en(wr_en), .wr_bank(wr_bank), .wr_idx(wr_idx), .wr_data(wr_data),
    .commit_en(commit_en), .commit_bank(commit_bank),
    .rd_bank(1'b0), .rd_idx('0), .rd_data(),
    .rls_en(rls_en), .rls_bank(rls_bank), .full(buf_full)
  );

  string tag;
  initial tag = $sformatf("BUS_W=%0d F0=%0d PW0=%0d MODE=%0d", BUS_W, F0, PW0, MODE);

  // ---------------- stimulus generation ----------------
  logic [BUS_W-1:0]   bd [$];
  logic [K-1:0]       bk [$];
  bit                 bl [$];
  logic [F0-1:0]      eimg [$];
  int                 enb  [$];
  logic [7:0]         pixq [$];
  logic [F0-1:0]      eimg_v;
  logic [BUS_W-1:0]   gd;
  logic [K-1:0]       gk;
  int                 npix, nb, pi, kind;

  initial begin
    bd.delete(); bk.delete(); bl.delete(); eimg.delete(); enb.delete();
    for (int im = 0; im < NIMG; im++) begin
      kind = (MODE == 0) ? 0 : im % 5;
      if (kind == 2)                npix = F0 + 1 + $urandom_range(0, K + 2);
      else if (kind == 3 && F0 > 1) npix = $urandom_range(1, F0 - 1);
      else                          npix = F0;
      pixq.delete();
      for (int i = 0; i < npix; i++) pixq.push_back(8'($urandom));
      eimg_v = '0;
      nb = (npix < F0) ? npix : F0;
      for (int i = 0; i < nb; i++) eimg_v[i] = pixq[i][7];
      eimg.push_back(eimg_v);
      enb.push_back(nb);
      pi = 0;
      while (pi < npix) begin
        gd = {BUS_W/32 + 1{$urandom}} ^ {BUS_W/32 + 1{$urandom}};
        gk = '0;
        for (int l = 0; l < K; l++) begin
          if (pi < npix && (MODE != 2 || $urandom_range(0, 99) < 60)) begin
            gk[l] = 1'b1;
            gd[l*DATA_W +: DATA_W] = pixq[pi];
            pi++;
          end
        end
        bd.push_back(gd); bk.push_back(gk); bl.push_back(pi >= npix);
      end
      if (MODE == 2 && $urandom_range(0, 3) == 0) begin
        bl[bl.size()-1] = 1'b0;
        bd.push_back({BUS_W/32 + 1{$urandom}}); bk.push_back('0); bl.push_back(1'b1);
      end
    end
  end

  // ---------------- driver + handshake / ready checks ----------------
  int  bi = 0;
  int  stalls = 0;
  int  accnew0_ready = 0;
  bit  img_active = 1'b0;
  bit  hs, rdy_a, rdy_b;
  logic pk_ready;
  assign pk_ready = dut.pk_ready;

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
      if (s_valid && !s_ready) stalls++;
      if (!mid_image && !accept_new && s_ready) begin
        errors++; $error("%s: s_ready high at image boundary with accept_new=0", tag);
      end
      if (!mid_image && s_ready && buf_full[wr_bank]) begin
        errors++; $error("%s: s_ready high at image boundary with bank full", tag);
      end
      if (mid_image && s_ready !== pk_ready) begin
        errors++; $error("%s: mid-image s_ready=%b differs from packer ready=%b", tag, s_ready, pk_ready);
      end
      if (mid_image && !accept_new && s_ready) accnew0_ready++;
      if (hs) begin
        if (!img_active && !(accept_new && !buf_full[wr_bank])) begin
          errors++; $error("%s: image started with accept_new=%b full=%b", tag, accept_new, buf_full);
        end
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
  bit  cm_d = 1'b0;
  bit  cm_bank_d = 1'b0;
  bit  exp_bank = 1'b0;
  int  nbits_chk;
  int  settle = 0;

  always @(posedge clk) begin : chk
    if (rst) begin
      rel_t[0] = -1; rel_t[1] = -1; cm_d = 1'b0; rls_en <= 1'b0;
    end else begin
      if (wr_en && buf_full[wr_bank]) begin
        errors++; $error("%s: write into a full bank", tag);
      end
      if (wr_en && int'(wr_idx) >= NCH) begin
        errors++; $error("%s: wr_idx %0d out of range", tag, wr_idx);
      end
      // delayed content check of the commit seen last cycle
      if (cm_d) begin
        if (chk_idx >= NIMG) begin
          errors++; $error("%s: extra commit", tag);
        end else begin
          nbits_chk = imin(cdiv(enb[chk_idx], PW0) * PW0, F0);
          for (int j = 0; j < F0; j++)
            if (j < nbits_chk && pp.mem[cm_bank_d][j] !== eimg[chk_idx][j]) begin
              errors++;
              $error("%s: image %0d bit %0d exp=%b got=%b", tag, chk_idx, j,
                     eimg[chk_idx][j], pp.mem[cm_bank_d][j]);
              break;
            end
          chk_idx++;
        end
        rel_t[cm_bank_d] = (MODE == 0) ? 0 : $urandom_range(0, 12);
      end
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
        if (MODE == 0 && PW0 >= K && stalls > NIMG) begin
          errors++; $error("%s: %0d stall cycles for %0d images (throughput)", tag, stalls, NIMG);
        end
        if (MODE != 0 && accnew0_ready == 0)
          $display("NOTE %s: no mid-image cycle with accept_new=0 and s_ready=1 observed", tag);
        done = 1'b1;
      end
    end
  end
endmodule

module tb_input_packer;
  localparam int NCFG = 5;
  localparam int CB [NCFG] = '{64, 8, 128, 64, 32};
  localparam int CF [NCFG] = '{784, 13, 100, 5, 20};
  localparam int CP [NCFG] = '{66, 3, 7, 1, 64};

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic done [NCFG*3];
  int   errs [NCFG*3];

  for (genvar c = 0; c < NCFG; c++) begin : g_cfg
    for (genvar m = 0; m < 3; m++) begin : g_mode
      ip_case #(.BUS_W(CB[c]), .F0(CF[c]), .PW0(CP[c]), .MODE(m), .NIMG(m == 0 ? 12 : 30)) u (
        .clk(clk), .done(done[c*3+m]), .errors(errs[c*3+m]));
    end
  end

  int total;
  initial begin
    wait (&{done[0], done[1], done[2], done[3], done[4], done[5], done[6], done[7], done[8],
            done[9], done[10], done[11], done[12], done[13], done[14]});
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
