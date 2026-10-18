// Double-buffered activation buffer with a claim / commit / release slot handshake.
//
// Two slots, each EMPTY -> CLAIMED -> FULL -> EMPTY, with round-robin claim (cp), write (wp)
// and read (rp) pointers:
//   wr_can_claim : slot[cp] is EMPTY            wr_claim   : slot[cp] -> CLAIMED, cp toggles
//   wr_en/commit : target slot[wp] (the oldest CLAIMED slot)
//                                               wr_commit  : slot[wp] -> FULL, wp toggles
//   rd_full      : slot[rp] is FULL             rd_release : slot[rp] -> EMPTY, rp toggles
//   rd_full_next : the other slot (read pointer + 1) is FULL, so a consumer releasing slot A
//                  on its last read already knows whether slot B can be read next cycle
//   idle         : no slot FULL
// Two claims may be outstanding (slot 1 claimed while slot 0 is still CLAIMED).
//
// Timing contract:
//   * wr_commit may coincide with the image's final wr_en (that write lands in the slot being
//     committed: the write address uses the pre-commit wp) or come any cycle later.
//   * wr_en never occurs in the cycle of the claim of its own slot.
//   * rd_release may coincide with the image's last rd_en (the read uses the pre-release rp).
//   * wr_can_claim / rd_full / rd_full_next / idle are registered: an event sampled at edge k is visible from
//     edge k+1. Reset (synchronous, active high) clears control state only; storage has none.
//   * rd_data appears READ_LATENCY cycles after rd_en / rd_addr.
//
// Storage: 2*DEPTH <= 4 -> registers (asynchronous read mux followed by READ_LATENCY pipeline
// registers); otherwise a bnn_ram "SDP" memory. The address is the concatenation {slot, addr}
// (slot bit = MSB, no adder), so the memory depth is 2**(addr_w(DEPTH)+1); the words
// [DEPTH, 2**addr_w(DEPTH)) of each slot are simply never addressed.
module bnn_act_buf import bnn_fcc_pkg::*; #(
    parameter int    WIDTH        = 16,
    parameter int    DEPTH        = 49,
    parameter string RAM_STYLE    = "distributed",
    parameter int    READ_LATENCY = 1
) (
    input  logic                     clk,
    input  logic                     rst,
    output logic                     wr_can_claim,
    input  logic                     wr_claim,
    input  logic                     wr_en,
    input  logic [addr_w(DEPTH)-1:0] wr_addr,
    input  logic [    WIDTH-1:0]     wr_data,
    input  logic                     wr_commit,
    output logic                     rd_full,
    output logic                     rd_full_next,
    // rd_en is part of the interface contract; the read port is free-running (reads are only
    // observed when the consumer samples rd_data), so it has no function here.
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic                     rd_en,
    /* verilator lint_on UNUSEDSIGNAL */
    input  logic [addr_w(DEPTH)-1:0] rd_addr,
    output logic [    WIDTH-1:0]     rd_data,
    input  logic                     rd_release,
    output logic                     idle
);

  localparam int AW = addr_w(DEPTH);

  if (WIDTH < 1) begin : g_chk_width
    $error("bnn_act_buf: WIDTH must be >= 1 (got %0d)", WIDTH);
  end
  if (DEPTH < 1) begin : g_chk_depth
    $error("bnn_act_buf: DEPTH must be >= 1 (got %0d)", DEPTH);
  end
  if (READ_LATENCY < 1) begin : g_chk_lat
    $error("bnn_act_buf: READ_LATENCY must be >= 1 (got %0d)", READ_LATENCY);
  end

  // ---------------------------------------------------------------------------------------
  // Slot control state
  // ---------------------------------------------------------------------------------------
  logic [1:0] claimed;  // slot is CLAIMED
  logic [1:0] full;  // slot is FULL
  logic cp, wp, rp;  // claim / write / read pointers

  logic [1:0] claimed_n, full_n;
  logic cp_n, wp_n, rp_n;

  always_comb begin
    for (int s = 0; s < 2; s++) begin
      claimed_n[s] = (claimed[s] && !(wr_commit && wp == 1'(s))) || (wr_claim && cp == 1'(s));
      full_n[s]    = (full[s] && !(rd_release && rp == 1'(s))) || (wr_commit && wp == 1'(s));
    end
    cp_n = cp ^ wr_claim;
    wp_n = wp ^ wr_commit;
    rp_n = rp ^ rd_release;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      claimed      <= '0;
      full         <= '0;
      cp           <= 1'b0;
      wp           <= 1'b0;
      rp           <= 1'b0;
      wr_can_claim <= 1'b1;
      rd_full      <= 1'b0;
      rd_full_next <= 1'b0;
      idle         <= 1'b1;
    end else begin
      claimed      <= claimed_n;
      full         <= full_n;
      cp           <= cp_n;
      wp           <= wp_n;
      rp           <= rp_n;
      wr_can_claim <= !(claimed_n[cp_n] || full_n[cp_n]);
      rd_full      <= full_n[rp_n];
      rd_full_next <= full_n[~rp_n];
      idle         <= !(full_n[0] || full_n[1]);
    end
  end

  // ---------------------------------------------------------------------------------------
  // Replicated slot selects: wp / rp stay as the control copy; the data path (write decode,
  // read mux, RAM slot address bit) is driven from NC copies, each serving one SLICE-bit slice
  // of the word (fanout ~SLICE), all loaded with the same next value (behavior unchanged).
  // ---------------------------------------------------------------------------------------
  localparam int SLICE = 16;
  localparam int NC = ceil_div(WIDTH, SLICE);

  (* keep = "true", shreg_extract = "no" *) logic [NC-1:0] wp_c;
  (* keep = "true", shreg_extract = "no" *) logic [NC-1:0] rp_c;

  always_ff @(posedge clk) begin
    if (rst) begin
      wp_c <= '0;
      rp_c <= '0;
    end else begin
      wp_c <= {NC{wp_n}};
      rp_c <= {NC{rp_n}};
    end
  end

  // ---------------------------------------------------------------------------------------
  // Storage; the write slot is the pre-commit wp, the read slot the pre-release rp.
  // ---------------------------------------------------------------------------------------
  for (genvar c = 0; c < NC; c++) begin : g_slice
    localparam int LO = c * SLICE;
    localparam int SW = min_int(SLICE, WIDTH - LO);

    if (2 * DEPTH <= 4) begin : g_regs
      // Word index space 2**AW per slot (DEPTH=1 still has a 1-bit address).
      logic [SW-1:0] mem[2][2**AW];
      (* shreg_extract = "no" *) logic [SW-1:0] rd_pipe[READ_LATENCY];

      always_ff @(posedge clk) begin
        if (wr_en) mem[wp_c[c]][wr_addr] <= wr_data[LO+:SW];
        rd_pipe[0] <= mem[rp_c[c]][rd_addr];
        for (int i = 1; i < READ_LATENCY; i++) rd_pipe[i] <= rd_pipe[i-1];
      end

      assign rd_data[LO+:SW] = rd_pipe[READ_LATENCY-1];
    end else begin : g_ram
      bnn_ram #(
          .WIDTH       (SW),
          .DEPTH       (2 ** (AW + 1)),
          .RAM_STYLE   (RAM_STYLE),
          .READ_LATENCY(READ_LATENCY),
          .PORTS       ("SDP")
      ) u_ram (
          .clk    (clk),
          .we     (wr_en),
          .wr_addr({wp_c[c], wr_addr}),
          .wr_data(wr_data[LO+:SW]),
          .rd_addr({rp_c[c], rd_addr}),
          .rd_data(rd_data[LO+:SW])
      );
    end
  end

endmodule
