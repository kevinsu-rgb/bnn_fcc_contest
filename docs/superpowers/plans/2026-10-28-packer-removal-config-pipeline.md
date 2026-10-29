# Packer Removal and Config-Path Pipelining Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Delete `bit_packer`, make the image input position-based, pipeline the config path and clean up the ping-pong read/write logic, raising fMax and cutting LUTs without changing judged throughput.

**Architecture:** The image path writes each beat's K bits straight into the layer-0 ping-pong (WR_W = K). Each config writer stages one neuron in an F-bit register at fixed byte positions and emits PASSES words through a registered write port. The parser drives a registered one-hot layer select. `pingpong_buf` keeps its behaviour with a plain-mux read and shared write enables.

**Tech Stack:** SystemVerilog (IEEE 1800-2012), Vivado 2025.2 xsim (`sim/*.sh`), Verilator lint (`sim/lint.sh`), openflex + Vivado for synthesis (XCU250-FIGD2104-2L-E).

**Spec:** `docs/superpowers/specs/2026-10-28-packer-removal-config-pipeline-design.md`

## Global Constraints

- Do not edit `verification/bnn_fcc_tb.sv`, `verification/bnn_fcc_tb_pkg.sv`, `verification/axi4_stream_if.sv`, `sim/cfg/*`, `sim/*.sh`, `README.md`, `rtl/README.md`, `verification/README.md`, `openflex/README.md`, `doc/`, or earlier specs/plans.
- Allowed outside `rtl/`: `verification/unit/*` (our TBs); the source lists in `openflex/bnn_fcc_timing.yml` and `openflex/bnn_fcc_verify.yml`; `sim/README.md` "Latest results" section, plus removing `tb_bit_packer` from its "Unit testbenches" line.
- Judged defaults stay: `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 3, 1}`, `RAM_STYLE = "distributed"`.
- Do not change `neuron_processor.sv`, `popcount.sv`, `bnn_layer.sv`, `argmax.sv`, `axis_out.sv`, `cfg_byte_serializer.sv`.
- Every `s_ready` stays a function of registered state only (TBs check this).
- Code style: match the surrounding RTL (2-space indent in leaf modules, 4-space in `bnn_fcc.sv`; header comment block stating behaviour; `bits()` / `cdiv()` from `bnn_fcc_pkg`).
- Gate set (run at the end of every task unless a task says otherwise): `sim/lint.sh` clean; `sim/run_unit.sh` exit 0; for tasks 2–5 also `sim/run_regress.sh` exit 0.
- Commits end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01AgTzjKWb29sXbysykb5tmg
  ```

## Review Focus

1. Abort while a writer's emission tail is still running must produce no further weight writes. Test: `tb_cfg_layer_writer` abort at `cut = BPN` plus a no-write-after-abort check (Task 4).
2. A message following one whose last-neuron emission outlasts header + WAIT (PI = 1, PASSES = F0 = 20 > 18 cycles) must not cut the tail. Test: new `tb_cfg_sequences` instance `topo_pi1` (Task 4).
3. Gapless weight bytes at PW ≥ 8 must stay correct while emission overlaps the next neuron. Test: gapless `send` in `tb_cfg_layer_writer` (Task 4).
4. Overlong image when NCH is a power of two must not wrap `idx_q` back onto chunk 0. Test: `tb_input_packer` config F0 = 64, BUS 64 (NCH = 8) with the overlong kind (Task 2).
5. Single-beat images (F0 ≤ K) back to back with a full bank must hold `s_ready` low until release, and commit without `mid_q` ever set. Test: `tb_input_packer` config F0 = 5, BUS 64 in all modes (Task 2).

---

### Task 1: `pingpong_buf` plain-mux read and shared write enables

**Files:**
- Modify: `rtl/pingpong_buf.sv`
- Test: `verification/unit/tb_pingpong_buf.sv`
- Scratch (not committed): `<scratchpad>/pp_ooc/pp_ooc.tcl`

**Interfaces:**
- Consumes: nothing new.
- Produces: `pingpong_buf` with unchanged parameters (`NBITS`, `WR_W`, `RD_W`), ports and timing.

- [ ] **Step 1: Add the layer-0 shape to the unit TB.** In `tb_pingpong_buf`, set `NC = 11` and add `pp_case #(.NBITS(784), .WR_W(8), .RD_W(66)) c10`.

- [ ] **Step 2: Run it against the current RTL.** Run `sim/run_unit.sh tb_pingpong_buf`. Expected: `PASS tb_pingpong_buf`. This is a characterization test: it pins the behaviour to keep.

- [ ] **Step 3: Record a baseline OOC synthesis.** Write `pp_ooc.tcl`: `read_verilog -sv rtl/bnn_fcc_pkg.sv rtl/pingpong_buf.sv`, `synth_design -top pingpong_buf -mode out_of_context -part xcu250-figd2104-2L-e -generic NBITS=784 -generic WR_W=8 -generic RD_W=66`, a 1.0 ns clock on `clk`, `opt_design; place_design; route_design`, then print the LUT/FF totals, WNS, and the count of `FD*` cells whose D pin is driven by a LUT (same query as in brainstorming). Run it on the current RTL and record the four numbers.

- [ ] **Step 4: Rewrite `rtl/pingpong_buf.sv` (spec §3.6).**
  - **Read:** `localparam int RIDX_W = bits(NRD)` and an array `logic [RD_W-1:0] words [2][2**RIDX_W]`, filled combinationally: bit k of row r of bank b is `mem[b][r*RD_W + k]` when `r*RD_W + k < NBITS`, else `1'b0`; rows ≥ NRD are `'0`. Then `rd_nxt = words[rd_bank][rd_idx]`.
  - **Write:** a vector `logic [NWR-1:0] we [2]` with `NWR = cdiv(NBITS, WR_W)`, `we[b][c] = wr_en && wr_bank == 1'(b) && int'(wr_idx) == c`. Each bit flop does `if (we[b][j / WR_W]) mem[b][j] <= wr_data[j % WR_W]`.
  - Keep the header comment, the full flags, the commit-wins rule and the registered `rd_data`.

- [ ] **Step 5: Run the unit TB.** Run `sim/run_unit.sh tb_pingpong_buf`. Expected: PASS.

- [ ] **Step 6: Measure.** Re-run `pp_ooc.tcl`. If LUT-driven D flops are still ≥ 90% of the mem flops, add `(* direct_enable = "yes" *)` to `we` and re-run. Keep the variant with the lowest LUT count whose WNS is no worse than the baseline. If neither the read rewrite nor the write rewrite beats the baseline, revert that half (spec §3.6). Record all numbers for Task 6.

- [ ] **Step 7: Run the gates.** Run `sim/lint.sh`, `sim/run_unit.sh` and `sim/run_regress.sh`. All must pass.

- [ ] **Step 8: Commit.** `git add rtl/pingpong_buf.sv verification/unit/tb_pingpong_buf.sv`, message `pingpong_buf: plain-mux read, shared write enables; layer-0 8->66 unit case`.

---

### Task 2: Position-based `input_packer`

**Files:**
- Modify: `rtl/input_packer.sv` (rewrite), `rtl/bnn_fcc.sv` (u_in instance, `ip_*` widths, `WRW_L` for l = 0)
- Test: `verification/unit/tb_input_packer.sv` (rewrite `ip_case` checks and stimulus)

**Interfaces:**
- Consumes: `pingpong_buf` (Task 1), unchanged.
- Produces: `input_packer #(DATA_W, BUS_W, F0)` (`PW0` removed). Same port names. `wr_idx` is `[bits(cdiv(F0, BUS_W/DATA_W))-1:0]` and `wr_data` is `[BUS_W/DATA_W-1:0]`. `commit_en = accept && s_last`. `s_ready = gate`.

- [ ] **Step 1: Rewrite `tb_input_packer` for the position rule.**
  - **Instances:** `ip_case #(BUS_W, F0, MODE, NIMG)` (drop `PW0`). The internal `pingpong_buf` is `NBITS(F0)`, `WR_W(K)`, `RD_W(K)`.
  - **Configs (BUS_W, F0):** `'{64,784}`, `'{8,13}`, `'{128,100}`, `'{64,5}`, `'{32,20}`, `'{64,64}` (NCH = 8, power of two; Review Focus 4). Run 3 modes each, so NCFG = 6 and 18 instances; update the `wait` to `&` all 18.
  - **Stimulus:**
    - Kinds are as today (normal, overlong `F0+1..F0+K+2` pixels, early `1..F0-1` pixels).
    - Pixels always fill lanes in order, one pixel per lane position.
    - MODE 2 clears TKEEP on random lanes (40%) **without** compaction: the pixel at that position still advances, and its expected bit is 0.
    - The last beat of each image has TKEEP = 0 on lanes beyond the image.
    - The trailing all-zero-TKEEP TLAST beat of MODE 2 stays, as a beat whose positions are beyond the image.
  - **Expected image:** bit i, for i < min(positions sent, F0), = lane kept ? pixel MSB : 0.
  - **Content check:** `nbits_chk = imin(positions_sent_rounded_up_to_K, F0)` over `pp.mem` at commit.
  - **New handshake checks:**
    - `s_ready === (mid_image || (accept_new && !buf_full[wr_bank]))` every cycle.
    - `commit_en === (s_valid && s_ready && s_last)` every cycle.
    - Mid-image `s_valid && !s_ready` never occurs (all modes).
    - `wr_en` never with `wr_idx >= NCH` (where NCH = cdiv(F0, K)).
  - **Remove:** the `pk_ready` reference and the throughput-stall criterion. The zero-stall check replaces it.
  - **Keep:** the `s_ready`-independent-of-`s_valid` check, the bank alternation check, the write-into-full-bank check, and random release.

- [ ] **Step 2: Run it against the old RTL.** Run `sim/run_unit.sh tb_input_packer`. Expected: FAIL. Compile fails on `.PW0` removal, or MODE 2 content mismatches because the old packer compacts.

- [ ] **Step 3: Rewrite `rtl/input_packer.sv` (spec §3.4).** Keep the registers `mid_q`, `bank_q`, `ovf_q` and `idx_q [bits(NCH)-1:0]`. Update the header comment to state the position rule, the clipping and the commit-on-TLAST behaviour. No `bit_packer` instance.

- [ ] **Step 4: Wire the top (`rtl/bnn_fcc.sv`).**
  - Add `localparam int K_IN = INPUT_BUS_WIDTH / INPUT_DATA_WIDTH`.
  - Set `ip_wr_idx [bits(cdiv(F0, K_IN))-1:0]` and `ip_wr_data [K_IN-1:0]`.
  - Drop `.PW0(PW0)` from `u_in`.
  - In `g_layer`, set `WRW_L = (l == 0) ? K_IN : ...`, so the chunk width matches `ip_wr_data`. `pingpong_buf` already supports `WR_W > NBITS` (`tb_pingpong_buf` c6: NBITS 5, WR_W 8): it gives one chunk, and the upper `wr_data` bits are unused.
  - Update the header comment line for the image path.

- [ ] **Step 5: Run the unit TB.** Run `sim/run_unit.sh tb_input_packer`. Expected: `PASS tb_input_packer`.

- [ ] **Step 6: Run the gates.** Run `sim/lint.sh`, `sim/run_unit.sh` and `sim/run_regress.sh`, all passing. Note the `sfc_perf` cycles/output (expected 100.5) and the `xsim_tb_fix` latency (expected ≤ 249) from the regress output.

- [ ] **Step 7: Commit.** `git add rtl/input_packer.sv rtl/bnn_fcc.sv verification/unit/tb_input_packer.sv`, message `input_packer: position-based TKEEP, direct K-bit ping-pong writes, commit on TLAST`.

---

### Task 3: Parser one-hot layer select

**Files:**
- Modify: `rtl/cfg_parser.sv`, `rtl/bnn_fcc.sv` (parser instance, `cw_sel`)
- Test: `verification/unit/tb_cfg_parser.sv`

**Interfaces:**
- Consumes: nothing new.
- Produces: `cfg_parser` output `msg_sel [TOTAL_LAYERS-2:0]` (registered one-hot, loaded at WAIT → PAYLOAD from `h_layer`, held until the next load). Output `msg_layer` is removed. The top uses `cw_sel = msg_sel`.

- [ ] **Step 1: Update `tb_cfg_parser`.**
  - Replace `logic [7:0] msg_layer` with `logic [L-1:0] msg_sel` (`.*` connects it).
  - In the payload and `msg_start` checks, compare `msg_sel !== (L'(1) << p.layer)` (and `st.layer`). Print `msg_sel` in the messages.
  - Add: `msg_sel` is one-hot (`$onehot(msg_sel)`) at every `msg_start`.

- [ ] **Step 2: Run it against the old RTL.** Run `sim/run_unit.sh tb_cfg_parser`. Expected: FAIL (no port `msg_sel`).

- [ ] **Step 3: Implement it in `rtl/cfg_parser.sv`.**
  - Replace `layer_q` with `sel_q [L-1:0]`, loaded in `S_WAIT` alongside `type_q` with bit l = `(h_layer == 8'(l))`.
  - The `w_loaded`/`t_loaded` updates use `sel_q[l]` instead of `32'(layer_q) == 32'(l)`.
  - `assign msg_sel = sel_q`.
  - Update the header comment (`msg_sel` instead of `msg_layer`).

- [ ] **Step 4: Wire the top (`rtl/bnn_fcc.sv`).** Remove `msg_layer`. Declare `logic [L-1:0] msg_sel`, connect `.msg_sel(msg_sel)`, and set `assign cw_sel[l] = msg_sel[l]`.

- [ ] **Step 5: Run the unit TB.** Run `sim/run_unit.sh tb_cfg_parser`. Expected: PASS.

- [ ] **Step 6: Run the gates.** Lint, all unit TBs, regress: all pass.

- [ ] **Step 7: Commit.** `git add rtl/cfg_parser.sv rtl/bnn_fcc.sv verification/unit/tb_cfg_parser.sv`, message `cfg_parser: registered one-hot msg_sel replaces msg_layer decode`.

---

### Task 4: Staging-register `cfg_layer_writer` and writer-idle WAIT

**Files:**
- Modify: `rtl/cfg_layer_writer.sv` (rewrite, same ports), `rtl/bnn_fcc.sv` (`pipeline_idle_q`, header comment)
- Test: `verification/unit/tb_cfg_layer_writer.sv`, `verification/unit/tb_cfg_sequences.sv`

**Interfaces:**
- Consumes: `msg_sel`-based `cw_sel` (Task 3).
- Produces: `cfg_layer_writer` with unchanged parameters and ports. It no longer instantiates `bit_packer`. Write ports are registered. With `OVERLAP` (PW ≥ 8), `s_ready` is constant 1. `pipeline_idle_q` now includes `&cw_idle`.

- [ ] **Step 1: Extend `tb_cfg_layer_writer`.**
  - **Drain waits:** replace every weight drain `repeat (12)` with `repeat (PASSES + 6)` and every threshold `repeat (3)` with `repeat (5)`.
  - **`send(cnt, gapless)`:** when `gapless` is set, `v = 1` every cycle. When `PW >= 8`, error if `s_ready !== 1'b1` in any sampled cycle.
  - **Gapless message:** after the clean weight message, send one more full weight message gapless and check it (Review Focus 3).
  - **Abort loop:**
    - Make round `r == 2` use `cut = BPN`, so a neuron's emission is in flight at the abort.
    - After `pulse(1)`, record `n_wr`, wait `PASSES + 6` cycles, and error if `n_wr` changed (Review Focus 1).
  - **New instances** (set `NC = 8`):
    - `cw_case #(.F(256), .N(10), .PW(256), .PN(3)) c6`: PASSES = 1, the judged layer-1 shape.
    - `cw_case #(.F(13), .N(7), .PW(13), .PN(2)) c7`: F % 8 ≠ 0 with OVERLAP; the last byte carries 3 bits that must be dropped.

- [ ] **Step 2: Add `topo_pi1` to `tb_cfg_sequences`.**
  - New harness `topo_pi1`: `TOTAL_LAYERS 4`, `TOPOLOGY '{20, 12, 9, 5}`, `PARALLEL_INPUTS 1`, `PARALLEL_NEURONS '{3, 4, 2}`, `SEED 67`.
  - Run it fifth, after `topo_hid` (`go8`/`done8`/`nerr8`).
  - Add it to the SUCCESS condition, the error print and the header comment (Review Focus 2).

- [ ] **Step 3: Run both against the old RTL.** Run `sim/run_unit.sh tb_cfg_layer_writer tb_cfg_sequences`. Expected: `tb_cfg_layer_writer` FAILS on the `PW >= 8` constant-`s_ready` check (the old packer drops ready for a pad-emit cycle). `tb_cfg_sequences` is expected to PASS (the old tail is ≤ 8 cycles); record the result either way.

- [ ] **Step 4: Rewrite `rtl/cfg_layer_writer.sv` (spec §3.2).** Spec §3.2 fixes the behaviour. These are the names and decisions it leaves open:
  - **Latched type:** `mt_q` (latched on `start`). There is no `mt` bypass, and `clr = start || abort` as today.
  - **Input register:** `in_v_q`, `in_byte_q [7:0]`, `in_bn_q [BN_W-1:0]`, `in_last_q`, loaded on `s_valid && s_ready` and cleared by `rst || clr`. `bn_q` counts accepted weight bytes and wraps at `BPN-1`.
  - **Staging:** `logic [F-1:0] stage`, byte c written when `in_v_q && !mt_q && in_bn_q == c`, storing only the bits `< F`. Not reset.
  - **Emission:**
    - Registers `em_q`, `p_q [bits(PASSES)-1:0]`, `lane_q`, `gbase_q`, `waddr_q`.
    - Start emitting the cycle after `in_v_q && in_last_q && !mt_q`.
    - Word: `localparam int PADW = PASSES * PW - F;`, `logic [PASSES*PW-1:0] padded = {{PADW{1'b1}}, stage};` (when `PADW == 0`, `padded = stage`), word = `padded[p_q*PW +: PW]`.
    - Advance `lane_q`/`gbase_q` after the last pass, with today's wrap rules.
  - **Registered write port:** `w_we`, `w_lane`, `w_addr`, `w_data` loaded from the emission state each cycle (`w_we <= em_q`). `rst`/`clr` clear `w_we`.
  - **Ready:** `localparam bit OVERLAP = (PW >= 8)`. `s_ready = mt_q || OVERLAP || !(in_v_q && in_last_q) && !em_q`.
  - **Thresholds:** the existing `tb_q`/`tsh_q`/`tlane_q`/`tgrp_q` logic, driven by `in_v_q && mt_q` and `in_byte_q` instead of `acc`/`s_byte`. `t_*` stay registered.
  - **Idle:** `w_done` = registered `w_we` with `w_lane == LAST_LANE` and `w_addr == LAST_GBASE + PASSES - 1`. `t_done` = registered `t_we` with `t_lane == LAST_LANE && t_addr == LAST_GRP` (`HAS_THR = 0`: `mt_q`). `active_q` as today.
  - **Header comment:** rewritten to describe staging, emission, overlap and stall.

- [ ] **Step 5: Run the writer TB.** Run `sim/run_unit.sh tb_cfg_layer_writer`. Expected: PASS.

- [ ] **Step 6: Show that `topo_pi1` needs the writer-idle WAIT.** With `pipeline_idle_q` still unchanged, run `sim/run_unit.sh tb_cfg_sequences`. Expected: FAIL in `topo_pi1`. A threshold or next-layer `start` clears layer 0's 20-cycle emission tail, so its last neuron's high words are missing and outputs mismatch. If it passes, report the fact and continue; the guard is still required by spec §3.5.

- [ ] **Step 7: Add the writer-idle term.** In `rtl/bnn_fcc.sv`, `pipeline_idle_q <= ... && !am_busy && (&cw_idle)`. Update the header comment: pipeline_idle means no image anywhere and no writer active.

- [ ] **Step 8: Run the sequences TB.** Run `sim/run_unit.sh tb_cfg_sequences`. Expected: PASS for all five harnesses.

- [ ] **Step 9: Run the gates.** Lint, all unit TBs (including `tb_mnist`) and regress all pass. Record `sfc_perf` cycles/output and latency.

- [ ] **Step 10: Commit.** `git add rtl/cfg_layer_writer.sv rtl/bnn_fcc.sv verification/unit/tb_cfg_layer_writer.sv verification/unit/tb_cfg_sequences.sv`, message `cfg_layer_writer: neuron staging register, registered write port; parser waits for writer idle`.

---

### Task 5: Delete `bit_packer`

**Files:**
- Delete: `rtl/bit_packer.sv`, `verification/unit/tb_bit_packer.sv`
- Modify: `openflex/bnn_fcc_timing.yml`, `openflex/bnn_fcc_verify.yml` (remove the `../rtl/bit_packer.sv` line), `sim/README.md` (remove `tb_bit_packer` from the "Unit testbenches" line only)

**Interfaces:**
- Consumes: Tasks 2 and 4 (no remaining instances).
- Produces: a design without `bit_packer`.

- [ ] **Step 1: Confirm nothing outside the deleted files and the YAML lists still references it.** Run `grep -rn bit_packer rtl verification openflex/*.yml sim/README.md`. Expected: hits only in the two files being deleted, the two YAML lines and the README unit line. (Old specs/plans may mention it; leave them.)

- [ ] **Step 2: Delete and edit.** `git rm` the two files and edit the three listed lines.

- [ ] **Step 3: Run the gates.** Lint, all unit TBs (one fewer) and regress all pass. Then run `grep -rn bit_packer rtl verification openflex/*.yml`. Expected: no output.

- [ ] **Step 4: Commit.** Message `Remove bit_packer (no remaining users)`.

---

### Task 6: Synthesis, measurement and results

**Files:**
- Modify: `sim/README.md` ("Latest results" section only)

**Interfaces:**
- Consumes: Tasks 1–5.
- Produces: measured results recorded in the README.

- [ ] **Step 1: Run full openflex synthesis.** `source ~/envs/openflex/bin/activate; source /tools/AMD/2025.2/Vivado/settings64.sh; cd openflex && openflex bnn_fcc_timing.yml -c bnn_fcc.csv`. Expected: completes; a new row in `bnn_fcc.csv`.

- [ ] **Step 2: Hierarchical utilization.** Open `openflex/build_vivado/outputs/post_route.dcp` in batch Vivado, run `report_utilization -hierarchical -hierarchical_depth 4`, and extract the module rows (`u_ser`, `u_parser`, `g_layer[*].u_cw`, `g_layer[*].u_pp`, `u_in`, `g_layer[*].u_layer`).

- [ ] **Step 3: Critical-path classes.** Group `outputs/route_paths.rpt` by normalized source/destination cell (same script as brainstorming) and build the before/after table for the seven classes of spec §1.3, plus the new top three classes.

- [ ] **Step 4: Update `sim/README.md` "Latest results".** Add one bullet for this change:
  - lint, unit (count), regress (count), `tb_mnist` results
  - `sfc_perf` cycles/output and latency
  - fMax, LUT (logic/mem), REG, CARRY8 and BRAM vs the baseline (424.27 MHz, LUT 21,834, REG 9,860)
  - per-module LUT deltas for packers/writers/ping-pong
  - the new worst path classes
  - a note that single-run fMax is noisy by several percent

  Leave the other bullets unchanged.

- [ ] **Step 5: Commit.** `git add sim/README.md`, message `sim/README: results for packer removal and config pipelining`.
