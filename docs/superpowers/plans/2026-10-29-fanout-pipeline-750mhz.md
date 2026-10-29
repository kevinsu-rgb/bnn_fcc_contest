# Fanout Localization and Datapath Pipelining (750 MHz) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raise fMax from 555 MHz toward 750 MHz by making every broadcast hop local (register copies, a config write tree, a rotating ping-pong read) and splitting the datapath into shorter stages, keeping steady-state II at 99 cycles.

**Architecture:**
- **Neuron processor:** registers the XNOR, loads `acc` every cycle, and has its own compare stage.
- **`pingpong_buf`:** one-hot registered write port, a bank that rotates while it is read (2-cycle read, operand copies per 8 lanes), and a port-level `full` that keeps today's timing.
- **`bnn_layer`:** per-lane read-address copies and a two-level config write tree.
- **Config path:** serializer registers its outputs; parser registers `msg_abort` and a last-byte flag; writer counts from its input register.

**Tech Stack:** SystemVerilog (IEEE 1800-2012), Vivado 2025.2 xsim (`sim/*.sh`), Verilator lint (`sim/lint.sh`), openflex + Vivado for synthesis (XCU250-FIGD2104-2L-E).

**Spec:** `docs/superpowers/specs/2026-10-29-fanout-pipeline-750mhz-design.md`

## Global Constraints

- Branch `feature/bnn-fcc-750mhz`.
- Do not edit `verification/bnn_fcc_tb.sv`, `verification/bnn_fcc_tb_pkg.sv`, `verification/axi4_stream_if.sv`, `sim/cfg/*`, `sim/*.sh`, `README.md`, `rtl/README.md`, `verification/README.md`, `openflex/README.md`, `doc/`, or earlier specs/plans.
- Allowed outside `rtl/`: `verification/unit/*`; the openflex source lists and PARALLEL_* defaults; the `sim/README.md` "Latest results" section.
- Judged defaults stay: `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 3, 1}`, `RAM_STYLE = "distributed"`.
- Every `s_ready` stays a function of registered state only (TBs check this).
- AXI behaviour at the DUT ports, message formats, weight/threshold memory layout and sequencer issue order are unchanged.
- Replica attribute: `(* keep = "true" *)` unless Task 1 rules otherwise; the attribute Task 1 records is used everywhere this plan says `keep`.
- Code style: match the surrounding RTL (2-space indent in leaf modules, 4-space in `bnn_fcc.sv`; the header comment block states behaviour and is updated with it; `bits()` / `cdiv()` / `imin()` from `bnn_fcc_pkg`). Multi-copy ports are packed 2-D (`logic [COPIES-1:0][W-1:0]`).
- Gate set (end of every RTL task): `sim/lint.sh` clean; `sim/run_unit.sh` exit 0 (includes `tb_mnist`); `sim/run_regress.sh` exit 0 (record `sfc_perf` cycles/output and latency).
- Throughput gate (Tasks 2–5): the scratch judged-configuration wrapper. Create it once (it stays outside the repo):
  ```bash
  S=<scratchpad>/perf; mkdir -p $S
  sed -e 's/cfg_sfc_perf/cfg_judged_perf/' -e "s/'{32,11,1}/'{32,3,1}/" sim/cfg/sfc_perf.sv > $S/judged_perf.sv
  cat > $S/run.sh <<EOF
  source $PWD/sim/common.sh
  run_xsim judged_perf cfg_judged_perf -i $PWD/sim/cfg "\${RTL_FILES[@]}" "\${TBPKG_FILES[@]}" $PWD/verification/bnn_fcc_tb.sv $S/judged_perf.sv
  grep -E 'xsim_tb_fix: Avg latency|cycles/output|SUCCESS|FAIL' $PWD/sim/build/judged_perf/run.log | tail -4
  EOF
  bash $S/run.sh
  ```
  Baseline: `SUCCESS`, 101.1 cycles/output, latency 310.0. Pass condition: `SUCCESS` and cycles/output ≤ 101.3 (steady-state II 99).
- Commits end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01AgTzjKWb29sXbysykb5tmg
  ```

## Review Focus

1. A read that starts in the first cycle `full` shows, when `commit_en` came in the same cycle as the last write (as `input_packer` does on the TLAST beat), must return that last write. Test: `tb_pingpong_buf` same-cycle commit plus an immediate advance (Task 3).
2. A hidden layer must never start into an output bank whose commit is still in the buffer's write register (the cycle after `pending` clears). Test: `tb_bnn_layer` GP = 1 cases c6/c7 back to back with the existing no-bubble phase, plus a `tb_pingpong_buf` check that `full` rises the cycle after `commit_en` (Task 3).
3. Rotation must stay aligned with gaps between advances, with GROUPS × NRD = 1, and when a producer writes in the cycle right after release. Test: `tb_pingpong_buf` read task with random gaps, a GROUPS = 1 image, and a write immediately after release (Task 3).
4. With `acc` loading every cycle, idle cycles carrying garbage between neurons must not corrupt the next neuron. Test: `tb_neuron_processor` gaps between neurons only, garbage on every input during gaps (Task 2).
5. `last_q` must handle `total_bytes` 1 and 2, and a null-item TLAST on the last payload byte. Test: `tb_cfg_parser` DISCARD messages with totals 1 and 2, and the null-TLAST case (Task 5).

---

### Task 1: Probe whether replica attributes survive synthesis (scratch only)

**Files:**
- Scratch (not committed): `<scratchpad>/keep_probe/probe.sv`, `<scratchpad>/keep_probe/probe.tcl`

**Interfaces:**
- Consumes: nothing.
- Produces: a ruling on which attribute (`keep` or `dont_touch`) prevents Vivado from merging equivalent registers. Later tasks use it wherever they say `keep`.

- [ ] **Step 1: Write the probe.** In module `probe` (inputs `clk`, `d [7:0]`, `sel [3:0]`; output `q [31:0]`):
  - `src_q <= d`.
  - Eight copies `ka_q[i] <= src_q` marked `(* keep = "true" *)`.
  - Eight copies `kd_q[i] <= src_q` marked `(* dont_touch = "true" *)`.
  - Eight unmarked copies `un_q[i] <= src_q`.
  - Each copy drives its own 4-bit slice of an AND with `sel`, so all copies have loads.
- [ ] **Step 2: Synthesize.** In `probe.tcl`: `read_verilog -sv probe.sv; synth_design -top probe -mode out_of_context -part xcu250-figd2104-2L-e; opt_design`. Then print `llength [get_cells -hier -filter {REF_NAME =~ FD* && NAME =~ *ka_q*}]`, and the same for `kd_q*` and `un_q*`. Run: `source /tools/AMD/2025.2/Vivado/settings64.sh; vivado -mode batch -nojournal -nolog -source probe.tcl`.
  Expected: `ka_q` 64 and `kd_q` 64 (8 copies × 8 bits); `un_q` fewer than 64 (merged).
- [ ] **Step 3: Rule.** If `ka_q` is 64, the plan's attribute is `keep`. Else if `kd_q` is 64, it is `dont_touch`. Else stop and report: neither prevents merging, and the replication design needs another mechanism. Record `Task 1: Ruling: replica attribute = <keep|dont_touch> — probe counts ka/kd/un = <a>/<b>/<c>` in the ledger.

---

### Task 2: Neuron processor stages and popcount levels default

**Files:**
- Modify: `rtl/neuron_processor.sv`, `rtl/bnn_fcc_pkg.sv` (`np_latency`), `rtl/bnn_fcc.sv` (default `POPCOUNT_LEVELS_PER_STAGE = 2`)
- Test: `verification/unit/tb_neuron_processor.sv`

**Interfaces:**
- Consumes: Task 1 attribute.
- Produces:
  - `neuron_processor`: same ports and parameters. `out_valid`, `out_y` and `out_acc` are valid together `np_latency(PW, LPS)` cycles after an `in_valid && last_pass` input.
  - `np_latency(w, lps) = popcount_latency(w, lps) + 3`.
  - Contract: the passes of one neuron arrive in consecutive cycles; gaps (`in_valid = 0`) only occur between neurons.

- [ ] **Step 1: Update the TB to the new contract and latency.**
  - In `tb_neuron_processor`, set `LAT = popcount_latency(PW, 3) + 3`. Write the formula out, not `np_latency`, so the test fails on the old RTL.
  - Draw a gap only between neurons: after `last_pass`, before the next `first_pass`. Inside a neuron, passes go back to back.
  - Keep the garbage inputs (`x`, `w`, `thr`, `first_pass`, `last_pass`) during gaps.
  - Update the TB header comment.
- [ ] **Step 2: Run it against the current RTL.** `sim/run_unit.sh tb_neuron_processor`. Expected: FAIL (acc/y mismatches at the new LAT).
- [ ] **Step 3: Implement (spec §3.3).**
  - `xn_q <= ~(x ^ w)` feeds the popcount.
  - Side-band delay lines get one extra stage for the XNOR register, so their length becomes `PC_LAT + 1`.
  - `acc <= f ? ACC_W'(pc) : acc + ACC_W'(pc)` every cycle, where `f` is the aligned first flag.
  - Compare stage, one cycle later: `out_y <= CMP_W'(acc) >= CMP_W'(t)` with the aligned threshold, `out_acc_q <= acc`, and `out_valid <= v && l`, all aligned.
  - `out_acc = out_acc_q`.
  - Mark the `f_d` array with the Task 1 attribute.
  - Valid pipeline reset unchanged.
  - Then set `np_latency` to `+ 3` and the top default LPS to 2.
  - Update the header comments (NP contract, latency).
- [ ] **Step 4: Run the NP and layer TBs.** `sim/run_unit.sh tb_neuron_processor tb_bnn_layer tb_popcount`. Expected: PASS for all three (`tb_bnn_layer` takes `NPL` from `np_latency`).
- [ ] **Step 5: Gates + throughput.** Run the gate set and the throughput gate. Expected: all pass; judged latency ≈ 319 (310 + 3 per layer); cycles/output ≤ 101.3.
- [ ] **Step 6: Commit.** `git add rtl/neuron_processor.sv rtl/bnn_fcc_pkg.sv rtl/bnn_fcc.sv verification/unit/tb_neuron_processor.sv`, message `neuron_processor: registered XNOR, free-running accumulate, separate compare stage; LPS default 2`.

---

### Task 3: Rotating one-hot `pingpong_buf`, `input_packer` pointer, `bnn_layer` read side, top wiring

**Files:**
- Modify: `rtl/pingpong_buf.sv`, `rtl/input_packer.sv`, `rtl/bnn_layer.sv`, `rtl/bnn_fcc.sv`
- Test: `verification/unit/tb_pingpong_buf.sv`, `verification/unit/tb_input_packer.sv`, `verification/unit/tb_bnn_layer.sv`
- Scratch: `<scratchpad>/pp_ooc/pp_ooc.tcl`

**Interfaces:**
- Consumes: Task 2 `np_latency`; Task 1 attribute.
- Produces:
  - `pingpong_buf #(NBITS, WR_W, RD_W, RD_COPIES = 1)`, ports:
    - `wr_en`, `wr_bank`, `wr_oh [cdiv(NBITS,WR_W)]`, `wr_data [WR_W]`
    - `commit_en`, `commit_bank`
    - `rd_adv`, `rd_bank`, `rd_data [RD_COPIES-1:0][RD_W-1:0]`
    - `rls_en`, `rls_bank`, `full [1:0]`
  - Timing: the read is 2 cycles. `full` rises in the cycle after `commit_en`.
  - `input_packer`: `wr_oh [cdiv(F0, BUS_W/DATA_W)]` replaces `wr_idx`; all other ports unchanged.
  - `bnn_layer`:
    - `in_rd_adv` replaces `in_rd_idx`.
    - `in_rd_data [cdiv(PN,8)-1:0][PW-1:0]`.
    - `out_wr_oh [cdiv(N,PN)]` replaces `out_wr_idx`.
    - Lane k uses operand copy `k / 8`.

- [ ] **Step 1: Rewrite `tb_pingpong_buf` for the new contract (spec §3.1, §4).**
  - **Parameters:** `pp_case` gains `RD_COPIES`. Writes drive `wr_oh = 1 << chunk`.
  - **`write_bank`:**
    - Writes chunks in random order with random gaps, as today.
    - Commit either in the cycle of the last write (half the images) or later.
    - Checks that `full[b]` is 0 in the commit cycle and 1 in the following cycle.
  - **`read_bank(b, groups)`:**
    - Issues `groups × NRD` advances with random gaps (one advance per cycle at most).
    - In half the images the first advance comes in the first cycle `full[b]` reads 1.
    - Releases with the last advance.
    - The k-th advance's data must appear on every copy exactly 2 cycles later: word `k % NRD`, with bits ≥ NBITS reading 0.
    - In the cycle in between, the copies must still hold the previous response (latency exact).
  - **Release and simultaneous cases:**
    - In half the iterations, the next `write_bank` of the same bank starts in the cycle right after release.
    - Keep the overlap fork (write the other bank while reading).
    - Keep the simultaneous commit/release-on-different-banks case.
  - **Case list:** c0–c10, with `groups` drawn from 1..3, plus:
    - c11 `(784, 8, 66, RD_COPIES 4)`
    - c12 `(256, 3, 256, RD_COPIES 1)`
    - c13 `(5, 8, 8)` (NRD = 1, single chunk)
  - Update the header comment.
- [ ] **Step 2: Run it against the current RTL.** `sim/run_unit.sh tb_pingpong_buf`. Expected: FAIL (compile error: `wr_oh`/`rd_adv`/`RD_COPIES` don't exist).
- [ ] **Step 3: Implement `pingpong_buf` (spec §3.1).**
  - Write register stage: `en_q[2]`, `oh_q`, and per-bank data copies `dat_q[2]` (attribute).
  - `cm_q` and `cm_bank_q`; `full_q` set and release; `full = full_q | ({1'b0, cm_q} << cm_bank_q)`.
  - Banks are `LEN = NRD*RD_W` bits, still named `mem[2]` (the unit TBs read it hierarchically); pad bits `[NBITS, LEN)` are reset to 0 by `rst`.
  - Registered advance: `adv_q[b][r]` (attribute, one per row) and `bsel_q[c]` (attribute, one per copy).
  - Rotate and capture when `adv_q`, per spec. For NRD = 1, one `adv_q[b]` per bank and no rotation.
  - Write enable for bit j: `en_q[b] && oh_q[j / WR_W]`.
  - Update the header comment.
- [ ] **Step 4: Run it.** `sim/run_unit.sh tb_pingpong_buf`. Expected: PASS for all 14 cases.
- [ ] **Step 5: Check the copies survive.** In `pp_ooc.tcl`: `synth_design -top pingpong_buf -mode out_of_context -part xcu250-figd2104-2L-e -generic NBITS=784 -generic WR_W=8 -generic RD_W=66 -generic RD_COPIES=4; opt_design`, then count FD cells matching `*adv_q*`, `*bsel_q*`, `*dat_q*`. Expected: 24, 4 and 16 (2 × 8).
- [ ] **Step 6: Update `tb_input_packer`.**
  - Instantiate the new `pingpong_buf` (`wr_oh`; `rd_adv = 0`).
  - Replace the `wr_idx` range check with: `wr_oh` is one-hot when `wr_en`, and is never above bit NCH-1.
  - Run the content check one cycle later (writes land a cycle after the handshake). Read through the buffer's bank storage at its new hierarchical name.
  - Run `sim/run_unit.sh tb_input_packer`. Expected: FAIL (compile error: `input_packer` has no `wr_oh`).
- [ ] **Step 7: Implement `input_packer` (spec §3.2).**
  - `oh_q [NCH]`: bit 0 set on reset and on commit; shift left on an accepted, non-overflow beat.
  - `ovf_q` sets when an accepted beat writes chunk NCH-1 (`oh_q[NCH-1]`).
  - `wr_oh = oh_q`.
  - Run `sim/run_unit.sh tb_input_packer`. Expected: PASS (all 18 config/mode instances).
- [ ] **Step 8: Update `tb_bnn_layer`.**
  - The input and output `pingpong_buf` use the new ports. The TB producer drives `i_wr_oh = 1 << chunk`.
  - The output buffer takes `out_wr_oh`, with `RD_COPIES = 1` and `rd_adv` driven by the consumer (one advance per read, data checked 2 cycles later).
  - `in_rd_data` connects as `[cdiv(PN,8)][PW]`.
  - Run `sim/run_unit.sh tb_bnn_layer`. Expected: FAIL (compile error).
- [ ] **Step 9: Implement the `bnn_layer` read and output side (spec §3.5).**
  - `in_rd_adv = issue`.
  - Per-lane `la_q[k] <= waddr_r` and `lg_q[k] <= g_r` (attribute); `w_rd <= wmem[la_q[k]]` and `t_rd <= tmem[lg_q[k]]`.
  - `s1_first`/`s1_last` and `vpipe` get one extra stage so NP inputs align in t+2.
  - Lane k: `.x(in_rd_data[k / 8])`.
  - `g_o` becomes `oh_o [GROUPS]` (bit 0 on reset; rotate left on `np_valid`; `last_o = oh_o[GROUPS-1]`); `o_idx` becomes `o_oh`; `out_wr_oh = o_oh`.
  - Update the header comment.
  - Run `sim/run_unit.sh tb_bnn_layer`. Expected: PASS for all 17 cases.
- [ ] **Step 10: Wire the top (spec §3.9).**
  - `ip_wr_oh`, `pp_wr_oh` (layer 0 from `u_in`, layer l from layer l-1's `out_wr_oh`).
  - `rd_adv`, and `rd_data` as `[cdiv(PN_L,8)][PW_L]`; `RD_COPIES(cdiv(PN_L, 8))`.
- [ ] **Step 11: Gates + throughput.** Run the gate set and the throughput gate. Expected: all pass; judged latency ≈ 322; cycles/output ≤ 101.3.
- [ ] **Step 12: Mid-point synthesis.**
  - Run openflex: `source ~/envs/openflex/bin/activate; source /tools/AMD/2025.2/Vivado/settings64.sh; cd openflex && openflex bnn_fcc_timing.yml -c bnn_fcc.csv`.
  - Then run `<scratchpad>/sta/dump.tcl` (worst path per failing endpoint from `outputs/post_route.dcp`) and `python3 <scratchpad>/sta/group.py paths.csv`.
  - Record fMax, LUT, REG and the per-family table in the ledger. This is information for later tasks, not a gate.
- [ ] **Step 13: Commit.** `git add rtl/pingpong_buf.sv rtl/input_packer.sv rtl/bnn_layer.sv rtl/bnn_fcc.sv verification/unit/tb_pingpong_buf.sv verification/unit/tb_input_packer.sv verification/unit/tb_bnn_layer.sv`, message `pingpong_buf: one-hot registered write, rotating 2-cycle read with operand copies; per-lane read addresses`.

---

### Task 4: Config write tree and `cfg_wr_busy`

**Files:**
- Modify: `rtl/bnn_layer.sv`, `rtl/bnn_fcc.sv`
- Test: `verification/unit/tb_bnn_layer.sv`, `verification/unit/tb_cfg_sequences.sv`

**Interfaces:**
- Consumes: Task 3 `bnn_layer`.
- Produces:
  - `bnn_layer` output `cfg_wr_busy`: 1 while any write-tree stage holds a weight or threshold write.
  - Writes reach `wmem`/`tmem` 2 cycles after they appear on the `w_*`/`t_*` ports.
  - Top idle term `&cw_idle && !(|l_wr_busy)`.

- [ ] **Step 1: Extend `tb_bnn_layer`.**
  - Every cycle, model the expected busy as "a `w_we` or `t_we` was driven in one of the previous 2 cycles" and check `dut.cfg_wr_busy` against it.
  - Before starting images, the TB waits until `cfg_wr_busy` is 0 (in addition to its current wait).
  - Run `sim/run_unit.sh tb_bnn_layer`. Expected: FAIL (compile error: no `cfg_wr_busy`).
- [ ] **Step 2: Implement the tree (spec §3.5 write side).**
  - `LG = 8`, `NG = cdiv(PN, LG)`.
  - Level 1: `gw_q`, `glane_q`, `gaddr_q`, `gdata_q` and `gt_q`, `gtlane_q`, `gtaddr_q`, `gtdata_q`.
  - Level 2: `lw_q`, `laddr_q`, `lt_q`, `ltaddr_q`.
  - The memories write from level 2 with group-level data.
  - `cfg_wr_busy = |{gw_q, gt_q, lw_q, lt_q}`.
  - Update the header comment.
  - Run `sim/run_unit.sh tb_bnn_layer`. Expected: PASS.
- [ ] **Step 3: Add the system check to `tb_cfg_sequences`.**
  - In `cfgseq_harness`'s monitor, fail if an image beat is accepted (`data_in_valid && data_in_ready`) while any `dut.g_layer[l].cfg_wr_busy` is 1.
  - Add a test step to the existing flow that sends an image with `zero_gap` immediately after the last config message is accepted.
- [ ] **Step 4: Prove the check catches the bug.** Temporarily wire the top idle term without `!(|l_wr_busy)` and run `sim/run_unit.sh tb_cfg_sequences`.
  - Expected: FAIL, from the new monitor check, in at least one harness.
  - If it passes, the check can't see the hazard on these topologies. Record a ruling saying so (the term stays as a structural guarantee) and keep the check.
  - Restore the term.
- [ ] **Step 5: Wire the top.**
  - Collect `cfg_wr_busy` from each layer into `l_wr_busy[L-1:0]`.
  - Use `&cw_idle && !(|l_wr_busy)` in `accept_new_q` and `pipeline_idle_q`.
  - Run `sim/run_unit.sh tb_cfg_sequences`. Expected: PASS for all five harnesses.
- [ ] **Step 6: Gates + throughput.** Run the gate set and the throughput gate. Expected: all pass; judged latency unchanged from Task 3.
- [ ] **Step 7: Commit.** Message `bnn_layer: two-level config write tree with cfg_wr_busy; top waits for it`.

---

### Task 5: Serializer registered outputs, parser abort/last flag, writer counting

**Files:**
- Modify: `rtl/cfg_byte_serializer.sv`, `rtl/cfg_parser.sv`, `rtl/cfg_layer_writer.sv`
- Test: `verification/unit/tb_cfg_parser.sv` (changed), `verification/unit/tb_cfg_byte_serializer.sv` and `verification/unit/tb_cfg_layer_writer.sv` (unchanged characterization)

**Interfaces:**
- Consumes: nothing from Tasks 2–4.
- Produces:
  - Serializer: identical port timing.
  - Parser: `msg_abort` pulses one cycle after the handshake that carried the truncating TLAST; all other outputs unchanged.
  - Writer: identical port timing.
  - Note: spec §4 lists "delayed abort" for `tb_cfg_layer_writer`. The writer's own abort contract doesn't change (the delay is created in the parser), so that TB stays unchanged as a characterization test. The delayed abort is covered by `tb_cfg_parser` and `tb_cfg_sequences`.

- [ ] **Step 1: Characterize first.** `sim/run_unit.sh tb_cfg_byte_serializer tb_cfg_layer_writer`. Expected: PASS on the current RTL.
- [ ] **Step 2: Update `tb_cfg_parser`.**
  - The checker records `exp_abort` at the handshake and checks `msg_abort` in the next cycle. `msg_abort` must be 0 in every other cycle, including the handshake cycle itself.
  - Add to the directed flow:
    - an invalid header with `total_bytes = 1` (DISCARD one byte)
    - an invalid header with `total_bytes = 2`
    - a valid message whose last payload byte arrives as a beat with TLAST and is followed by a null item (TKEEP 0, TLAST 1)
  - Update the header comment.
  - Run `sim/run_unit.sh tb_cfg_parser`. Expected: FAIL (abort-timing check).
- [ ] **Step 3: Implement the parser (spec §3.7).**
  - `abort_q <= hs && state_q == S_PAYLOAD && tlast && !done_now`; `msg_abort = abort_q`.
  - `last_q` is loaded with `h_total == 32'd1` where `rem_q` is loaded, and with `rem_q == 32'd2` on each decrement. `last_cnt = last_q`.
  - Run `sim/run_unit.sh tb_cfg_parser`. Expected: PASS.
- [ ] **Step 4: Implement the serializer (spec §3.6).**
  - Registers `byte_q`, `null_q`, `eob_q`, `blast_q` load the next-state values on load and on advance; outputs are these registers.
  - Run `sim/run_unit.sh tb_cfg_byte_serializer`. Expected: PASS, with the TB unchanged.
- [ ] **Step 5: Implement the writer (spec §3.8).**
  - Remove `in_bn_q` and `in_last_q`. `bn_q` advances on `in_v_q && !mt_q`.
  - The stage write uses `bn_q`; `em_start = in_v_q && !mt_q && bn_q == BPN-1`.
  - `s_ready` per spec.
  - Run `sim/run_unit.sh tb_cfg_layer_writer`. Expected: PASS, with the TB unchanged.
- [ ] **Step 6: Gates + throughput.** Run the gate set (all five `tb_cfg_sequences` harnesses pass) and the throughput gate. Expected: all pass; latency unchanged.
- [ ] **Step 7: Commit.** Message `cfg path: registered serializer outputs, registered parser abort and last-byte flag, writer counts from its input register`.

---

### Task 6: Synthesis, popcount levels per stage comparison, results

**Files:**
- Modify: `sim/README.md` ("Latest results" section only); possibly `rtl/bnn_fcc.sv` (`POPCOUNT_LEVELS_PER_STAGE` default)

**Interfaces:**
- Consumes: Tasks 2–5.
- Produces: the final LPS default and measured results in the README.

- [ ] **Step 1: Synthesize with LPS 2.** Run openflex, `dump.tcl` and `group.py` as in Task 3 Step 12. Also run `report_utilization -hierarchical -hierarchical_depth 4` on `post_route.dcp`. Save `paths.csv`, the utilization report and the csv row under `<scratchpad>/sta/lps2/`.
- [ ] **Step 2: Synthesize with LPS 1.** Temporarily set the default to 1, run the same three steps, and save under `<scratchpad>/sta/lps1/`. Run the throughput gate with LPS 1. Expected: `SUCCESS`; cycles/output ≤ 101.3 (spec estimate: layers +6/+7, within budget).
- [ ] **Step 3: Choose.** Keep the LPS with the higher fMax among those that pass the throughput gate. If LPS 1 wins, commit the default change after the gate set passes (message `bnn_fcc: POPCOUNT_LEVELS_PER_STAGE default 1`); otherwise revert to 2. Record the ruling with both fMax values.
- [ ] **Step 4: Update `sim/README.md` "Latest results".** Add one bullet:
  - lint, unit, regress and `tb_mnist` results
  - `sfc_perf` cycles/output and latency; judged-configuration cycles/output and latency (scratch wrapper, noted as such)
  - fMax, LUT (logic/mem), REG, CARRY8 and BRAM, against the baseline (554.94 MHz, LUT 19,275, REG 11,104)
  - the per-family endpoint table at 650/700/750 MHz, before and after
  - the worst three path families now
  - both LPS results
  - the single-run noise note

  Leave the other bullets unchanged.
- [ ] **Step 5: Commit.** `git add sim/README.md` (plus `rtl/bnn_fcc.sv` if Step 3 changed it), message `sim/README: results for fanout localization and datapath pipelining`.
