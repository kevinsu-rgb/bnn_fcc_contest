# PARALLEL_INPUTS First-Layer-Only Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `PARALLEL_INPUTS` set the input width of the first hidden layer only; every later layer reads its full fan-in in one pass. Set the judged defaults to `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 3, 1}`.

**Architecture:** A one-line change to the `PW_L` derivation in `rtl/bnn_fcc.sv`. All lower modules are already generic in PW and handle `PASSES = 1`. A new `tb_cfg_sequences` harness whose bottleneck is a hidden layer proves the change. New unit cases cover 256-wide single-pass layers and full-width buffer reads.

**Tech Stack:** SystemVerilog, Vivado 2025.2 xsim (`sim/run_unit.sh`, `sim/run_regress.sh`), Verilator lint (`sim/lint.sh`), openflex + Vivado for synthesis.

**Spec:** `docs/superpowers/specs/2026-10-18-parallel-inputs-first-layer-design.md` (amends `docs/superpowers/specs/2026-10-10-bnn-fcc-design.md`).

## Global Constraints

- `PW[0] = min(PARALLEL_INPUTS, F[0])`; `PW[l] = F[l]` for l ≥ 1; `PN[l] = min(PARALLEL_NEURONS[l], N[l])`.
- Parameter types and the port list of `bnn_fcc` do not change.
- Judged defaults: `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 3, 1}`.
- Do not edit: provided TB logic, the existing `sim/cfg/*.sv` wrappers, `sim/*.sh`, `rtl/README.md`, `verification/README.md`, `doc/`, `openflex/README.md`, the 2026-10-10 spec. Do not add a `sim/cfg` wrapper.
- `verification/bnn_fcc_tb.sv`: only the `PARALLEL_*` default values may change. `openflex/rtl/bnn_fcc_timing.sv`: only the `PARALLEL_*` default values may change.
- `sim/README.md`: only the "## Latest results" section may change.
- A run passes iff xvlog/xelab/xsim all exit 0, the log contains `SUCCESS`, and it contains no `Error:`, `Fatal:`, `FAILURE` or `ERROR:` line (`sim/README.md`). Logs: `sim/build/<name>/run.log`.
- Report all test and synthesis output verbatim. If synthesis fMax < 100 MHz, stop and report; timing tuning is out of scope.
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Hidden fan-in not a multiple of 8, or of the previous PN** (e.g. `c_100`: F1 = 37 after PN0 = 5, F2 = 19 after PN1 = 3): config bytes pad each neuron's row; a full-width word must take exactly F bits, and the partial last write chunk must land in the right place. Pinned by `run_regress.sh c_100 c_100_bus` (Task 4) and the new `pp_case` WR_W 3 → RD_W 256 (Task 2).
2. **Hidden fan-in of 1** (`c_tiny`, `'{5,1,3,2}`: layer 1 has F = 1, so PW = 1): a 1-bit popcount and a 1-bit buffer read must still work. Pinned by `run_regress.sh c_tiny` (Task 4).
3. **Weights-only / thresholds-only reconfiguration of a single-pass layer while images stream**: the writer's address becomes `group` (no pass offset). Pinned by the new `topo_hid` harness, which runs the full reconfiguration suite (Task 1).
4. **Back-to-back images when a hidden layer is the bottleneck**: the ping-pong handoff must not add bubbles at one pass per group. Pinned by `topo_hid` `test_zero_gap_images` (Task 1).
5. **Deepest popcount pipeline on a 256-input NP** (LPS = 1, 8 levels → 8 stages): latency bookkeeping in `bnn_layer` must match. Pinned by the new `bl_case` F 256, N 10, PW 256, PN 1, LPS 1 (Task 2).

---

### Task 1: Hidden layers consume their full fan-in (failing system test + RTL)

The spec's first idea for a failing test (only updating the `tb_cfg_sequences:1050` bound) does not fail: in topo4, topo5 and topo4_lps1, layer 0 is the slowest layer under both the old and the new rule, so the bound does not move. Per spec §5.1, this task adds a harness whose bottleneck is a hidden layer.

**Files:**
- Modify: `verification/unit/tb_cfg_sequences.sv:1-5` (header comment), `:1049-1050` (bound formula), `:1176-1209` (`tb_cfg_sequences` module: fourth harness)
- Modify: `rtl/bnn_fcc.sv:6` (header comment), `rtl/bnn_fcc.sv:172` (`PW_L`)

**Interfaces:**
- Consumes: `cfgseq_harness` parameters `NAME, TOTAL_LAYERS, TOPOLOGY, PARALLEL_INPUTS, PARALLEL_NEURONS, SEED`; ports `go, done, nerr`.
- Produces: `g_layer[l].PW_L` = `imin(PARALLEL_INPUTS, F_L)` for l = 0, `F_L` otherwise. Tasks 2–4 rely only on this behavior.

- [ ] **Step 1: Update the zero-gap bound formula** at `tb_cfg_sequences.sv:1050`: the pass term becomes `cdiv_i(TOPOLOGY[l], (l == 0) ? imin_i(PARALLEL_INPUTS, TOPOLOGY[l]) : TOPOLOGY[l])`.

- [ ] **Step 2: Add the hidden-bottleneck harness** `u_topo_hid` to `tb_cfg_sequences`:
  `.NAME("topo_hid"), .TOTAL_LAYERS(4), .TOPOLOGY('{16, 24, 24, 4}), .PARALLEL_INPUTS(8), .PARALLEL_NEURONS('{24, 1, 1}), .SEED(53)`, with its own `go7/done7/nerr7`. Run it after `u_topo4_lps1`, include `nerr7 == 0` in the SUCCESS condition, and add `nerr7` to the error `$display`. Cycles/image for this harness:

  | Layer | F→N | PN | Old PW → cycles | New PW → cycles |
  |---|---|---|---|---|
  | 0 | 16→24 | 24 | 8 → 2 × 1 = 2 | 8 → 2 |
  | 1 | 24→24 | 1 | 8 → 3 × 24 = 72 | 24 → 24 |
  | 2 | 24→4 | 1 | 8 → 3 × 4 = 12 | 24 → 4 |

  The new bound is 24 + 2 = 26. F0 must be > 8 (`IMG_BEATS` = 2): `test_reset_mid_image` waits for `mon_in_mid` and line 801 calls `send_image(…, 1)`, and both need a multi-beat image. Update the header comment (lines 1–5) to list all four instances and their parameters.

- [ ] **Step 3: Run it to verify it fails on the current RTL**

  Run: `sim/run_unit.sh tb_cfg_sequences`
  Expected: exit code 1. `sim/build/tb_cfg_sequences/run.log` contains `test_zero_gap_images: outputs … are N cycles apart (bound 26)` for `topo_hid` with N ≈ 72. topo4, topo5 and topo4_lps1 still print PASS for every test.

- [ ] **Step 4: Change the RTL**
  - `rtl/bnn_fcc.sv:172`: `localparam int PW_L = (l == 0) ? imin(PARALLEL_INPUTS, F_L) : F_L;`
  - `rtl/bnn_fcc.sv:6`: the header comment states `PW = min(PARALLEL_INPUTS, F)` for layer 0 and `PW = F` (one pass) for layers ≥ 1.
  - `PW0` (line 62) is already layer 0 only; leave it.

- [ ] **Step 5: Run the test and lint to verify they pass**

  Run: `sim/run_unit.sh tb_cfg_sequences && sim/lint.sh`
  Expected: exit code 0. The log contains `[topo_hid] test_zero_gap_images: max steady-state output interval 24 cycles (bound 26)` (any interval ≤ 26 passes; report the actual value), `[topo_hid] done: 0 errors` and `SUCCESS: tb_cfg_sequences`. Lint prints no warnings.

- [ ] **Step 6: Commit**

  ```bash
  git add rtl/bnn_fcc.sv verification/unit/tb_cfg_sequences.sv
  git commit -m "PARALLEL_INPUTS sets layer-0 input width only; layers >= 1 read full fan-in"
  ```

---

### Task 2: Unit coverage for 256-wide single-pass layers and full-width buffer reads

These modules are already generic, so the new cases are expected to pass immediately (spec §5.2: coverage, not TDD). If one fails, stop and debug: it is a real RTL bug in an existing module.

**Files:**
- Modify: `verification/unit/tb_bnn_layer.sv:403-416` (`tb_bnn_layer` module)
- Modify: `verification/unit/tb_pingpong_buf.sv:143-158` (`tb_pingpong_buf` module)

**Interfaces:**
- Consumes: `bl_case #(F, N, PW, PN, IS_OUTPUT, LPS)` and `pp_case #(NBITS, WR_W, RD_W)` with ports `(clk, done, errors)`.
- Produces: nothing used by later tasks.

- [ ] **Step 1: Add the `bl_case` instances** `c14..c16` and set `NC = 17`:
  - `c14`: F 256, N 256, PW 256, PN 3, IS_OUTPUT 0
  - `c15`: F 256, N 10, PW 256, PN 1, IS_OUTPUT 1
  - `c16`: F 256, N 10, PW 256, PN 1, IS_OUTPUT 1, LPS 1

  Put them under a comment `// single-pass full-width hidden layers (PW = F; judged layers 1-2)`.

- [ ] **Step 2: Add the `pp_case` instances** `c8`, `c9` and set `NC = 10`:
  - `c8`: NBITS 256, WR_W 32, RD_W 256
  - `c9`: NBITS 256, WR_W 3, RD_W 256

  Check first: `pp_case` builds a write word with `{$urandom, $urandom, $urandom}` when `WR_W > 32`, which only covers 96 bits. Both new cases have WR_W ≤ 32, so no TB change is needed; confirm that the read-side expected value (`exp_new`, built bit by bit from `mdl`) has no width limit.

- [ ] **Step 3: Run the tests to verify they pass**

  Run: `sim/run_unit.sh tb_bnn_layer tb_pingpong_buf`
  Expected: exit code 0; logs contain `SUCCESS: tb_bnn_layer` and `SUCCESS: tb_pingpong_buf`. If `tb_bnn_layer` hits its `#20ms` timeout, report it with the log and do not raise the timeout without asking.

- [ ] **Step 4: Commit**

  ```bash
  git add verification/unit/tb_bnn_layer.sv verification/unit/tb_pingpong_buf.sv
  git commit -m "Unit cases: 256-wide single-pass layers and full-width ping-pong reads"
  ```

---

### Task 3: Judged defaults `PARALLEL_NEURONS = '{32, 3, 1}`

**Files:**
- Modify: `verification/bnn_fcc_tb.sv:118` (default value only)
- Modify: `openflex/rtl/bnn_fcc_timing.sv:25` (default value only)
- Modify: `verification/unit/tb_mnist.sv:2` (header comment), `:90` (DUT parameters)

**Interfaces:**
- Consumes: the Task 1 RTL.
- Produces: the judged configuration that Task 4 synthesizes and reports.

- [ ] **Step 1: Set the defaults**
  - `verification/bnn_fcc_tb.sv:118`: `parameter int PARALLEL_NEURONS[NON_INPUT_LAYERS] = '{32, 3, 1}`
  - `openflex/rtl/bnn_fcc_timing.sv:25`: `parameter int PARALLEL_NEURONS[TOTAL_LAYERS-1] = '{32, 3, 1}`
  - `PARALLEL_INPUTS = 66` stays in both. Check with `git diff` that no other line in either file changed.

- [ ] **Step 2: Point `tb_mnist` at the judged configuration**
  - Line 90: `.PARALLEL_NEURONS('{32, 3, 1})`.
  - Line 2: `(PARALLEL_INPUTS=66, PARALLEL_NEURONS={32,3,1})`.

- [ ] **Step 3: Run `tb_mnist`**

  Run: `sim/run_unit.sh tb_mnist`
  Expected: exit code 0; the log contains `MNIST: 100/100 match python` and `SUCCESS: tb_mnist`. Record the `Avg latency` and `Avg throughput` lines verbatim. They are measured under stress (0.8 valid, random ready), so they are not steady-state numbers.

- [ ] **Step 4: Commit**

  ```bash
  git add verification/bnn_fcc_tb.sv openflex/rtl/bnn_fcc_timing.sv verification/unit/tb_mnist.sv
  git commit -m "Judged defaults: PARALLEL_INPUTS 66, PARALLEL_NEURONS {32,3,1}"
  ```

---

### Task 4: Full verification, synthesis, results

**Files:**
- Modify: `sim/README.md`, section `## Latest results` only

**Interfaces:**
- Consumes: Tasks 1–3.
- Produces: the recorded results.

- [ ] **Step 1: Lint and all unit TBs**

  Run: `sim/lint.sh && sim/run_unit.sh`
  Expected: lint clean; `run_unit.sh` exit code 0 with all 13 unit TBs passing.

- [ ] **Step 2: Provided-TB regression**

  Run: `sim/run_regress.sh`
  Expected: exit code 0, all 11 configurations pass. For `sfc_perf` the script prints the `xsim_tb_fix: Avg latency (cycles)` and `Avg throughput (cycles/output)` lines. Expected steady state is 99 cycles/image; with 349-cycle latency before, the TB's figure was 101.5 = (99 × 99 + latency) / 100. Record both lines verbatim and label the latency as `sfc_perf` (PN `{32,11,1}`), not the judged configuration.

- [ ] **Step 3: Synthesis at the judged defaults**

  Run:
  ```bash
  source ~/envs/openflex/bin/activate
  source /tools/AMD/2025.2/Vivado/settings64.sh
  cd openflex && openflex bnn_fcc_timing.yml -c bnn_fcc.csv
  ```
  Expected: `openflex/bnn_fcc.csv` has fMax, LUT (LUTAsLogic / LUTAsMem), REG, CARRY8, BRAM, URAM, DSP. Compare against fMax 431.22 MHz, LUT 19,965 (12,655 / 7,310), REG 8,971, CARRY8 186, BRAM 0. If fMax < 100 MHz, stop here and report.

- [ ] **Step 4: Refresh `## Latest results` in `sim/README.md`**

  Replace the bullets with the measured values from Steps 1–3 and Task 3 Step 3:
  - lint and the unit/regression pass counts;
  - the `tb_mnist` line at PN `{32,3,1}`, noting it is under stress;
  - `sfc_perf` throughput and latency, labeled PN `{32,11,1}`, with the fill arithmetic redone using the new latency;
  - synthesis at PI 66 / PN `{32,3,1}`, next to the previous run's numbers.

  Nothing outside this section changes. The "Unit testbenches" line still names three `tb_cfg_sequences` instances; leave it and list it under follow-ups in the report.

- [ ] **Step 5: Commit**

  ```bash
  git add sim/README.md
  git commit -m "sim/README: results for first-layer-only PARALLEL_INPUTS"
  ```

- [ ] **Step 6: Report to the user**

  Include:
  - every pass/fail line, and the synthesis numbers before and after;
  - that the Questa MNIST check (`openflex/verify.sh`) is theirs to run;
  - follow-ups left alone: the stale "Unit testbenches" line in `sim/README.md`, and that no local provided-TB run uses PN `{32,3,1}` (spec §5.4).
