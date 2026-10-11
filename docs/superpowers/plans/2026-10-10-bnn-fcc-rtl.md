# BNN FCC RTL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `rtl/bnn_fcc.sv` as a layer-pipelined, arbitrary-parallelism AXI4-Stream BNN classifier that passes the provided testbench (xsim), a real-MNIST test, unit tests, and synthesizes at ≥ 100 MHz on XCU250-2L.

**Architecture:** One `bnn_layer` per non-input layer, each with its own neuron processors (NPs) and per-NP weight/threshold RAMs, connected by two-bank width-converting ping-pong buffers so all layers process different images concurrently. A byte-serial configuration manager validates message headers against `TOPOLOGY`, routes payloads (any order) to per-layer writers, and interlocks reconfiguration with pipeline idle. Images are binarized, TKEEP-compacted and packed into the layer-0 buffer; the output layer streams popcounts to a pipelined argmax and a credit-controlled output FIFO.

**Tech Stack:** SystemVerilog (IEEE 1800-2012), Vivado 2025.2 xsim (`xvlog/xelab/xsim`) for simulation, Verilator 5.020 for lint, openflex 0.1.4 + Vivado 2025.2 for synthesis.

**Spec:** `docs/superpowers/specs/2026-10-10-bnn-fcc-design.md` (read it; section numbers below refer to it).

## Global Constraints

- Language: synthesizable SystemVerilog-2012 in `rtl/`; synchronous active-high `rst`; no `initial` blocks in RTL except elaboration-time assertions.
- Never edit: `README.md`, `rtl/README.md`, `verification/README.md`, `doc/`, `openflex/README.md`, `verification/bnn_fcc_tb.sv` (except lines 117–118 default values), `verification/bnn_fcc_tb_pkg.sv`, `verification/axi4_stream_if.sv`, `openflex/verify.sh`. In `openflex/` edit only the file lists in the two `.yml` files and lines 24–25 default values of `openflex/rtl/bnn_fcc_timing.sv`.
- `PARALLEL_INPUTS` is a scalar `int`; `PARALLEL_NEURONS` is `int [TOTAL_LAYERS-1]`. DUT defaults stay `8` / `'{default: 8}`.
- Effective parallelism (spec §3.1): `PN[l] = min(PARALLEL_NEURONS[l], N[l])`, `PW[l] = min(PARALLEL_INPUTS, F[l])`, `GROUPS = ceil(N/PN)`, `PASSES = ceil(F/PW)`, `ACC_W = bits(F+1)`, `THR_W = bits(F+2)`, `bits(x) = max(1, $clog2(x))`.
- Judged defaults: `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 11, 1}`.
- Added DUT parameters: `POPCOUNT_LEVELS_PER_STAGE = 3`, `ARGMAX_LEVELS_PER_STAGE = 2`, `OUT_FIFO_DEPTH = 4`, `RAM_STYLE = "auto"`.
- `release` is a reserved SV keyword — use `rls_en` / `rls_bank` for ping-pong release.
- Every RTL module that uses package functions in its header imports it there: `module popcount import bnn_fcc_pkg::*; #(...) (...);`.
- Vivado attribute values must be literals: select `(* ram_style = "block" *)` / `"distributed"` / none with a generate `if` on `RAM_STYLE`.
- Simulation: `source /tools/AMD/2025.2/Vivado/settings64.sh`; always `xelab --timescale 1ns/100ps`; run in `sim/build/<name>/`.
- Pass rule for every simulation: log contains `SUCCESS` and no line matching `Error:|Fatal:|FAILURE` (case-sensitive, as `openflex/verify.sh` but also catching `$error`).
- xsim bug (spec §1.2): block-local uninitialized queues/dynamic arrays inside loops are not re-created per iteration. In new TB code, never declare an uninitialized queue/dynamic array inside a loop body — declare outside and `.delete()`/`new[]` explicitly.
- Commit after every task; messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Work on branch `feature/bnn-fcc-rtl`.

## Review Focus

1. **Back-to-back images with zero idle cycles** (TLAST beat immediately followed by the next image's first beat, valid probability 1.0): expect no lost/merged images and ~98 cycles/image. Test added in Task 13 (`test_zero_gap_images`).
2. **Long output backpressure** (`data_out_ready` low for thousands of cycles while images keep arriving): expect `data_in_ready` to drop, no result lost or reordered, all correct after release. Test added in Task 13 (`test_long_backpressure`).
3. **Non-64-bit buses** (`CONFIG_BUS_WIDTH` = 8/32, `INPUT_BUS_WIDTH` = 8/128 with a partial last beat): expect identical classification. Regression configs `c_100_bus` and `c_13753_bus8` added in Task 11.
4. **PW < 8 at top level** (one config byte yields several weight words; parser must stall without losing bytes): covered by `c_13753`/`c_tiny` (PI = 3/1) in Task 11 and by `tb_cfg_layer_writer` PW ∈ {1,3} in Task 7.
5. **Reset while a result is pending on `data_out`** (valid high, ready low): expect valid to drop on reset and no stale output after reconfiguration. Test added in Task 13 (`test_reset_with_output_pending`).

---

## File Structure

| File | Responsibility | Task |
|---|---|---|
| `rtl/bnn_fcc_pkg.sv` | `cdiv, imin, imax, bits, popcount_latency, np_latency` | 1 |
| `rtl/popcount.sv` | pipelined popcount | 1 |
| `rtl/bit_packer.sv` | bit stream → OUT_W-bit words, flush with PAD | 2 |
| `rtl/pingpong_buf.sv` | two-bank width-converting buffer | 3 |
| `rtl/neuron_processor.sv` | XNOR/popcount/accumulate/compare | 4 |
| `rtl/bnn_layer.sv` | layer sequencer, per-NP RAMs, NP array | 5 |
| `rtl/argmax.sv`, `rtl/axis_out.sv` | argmax stream reducer; output FIFO + AXIS | 6 |
| `rtl/cfg_layer_writer.sv` | payload → weight words / clamped thresholds | 7 |
| `rtl/cfg_byte_serializer.sv` | config beat → kept bytes | 8 |
| `rtl/cfg_parser.sv` | header/validate/route/load flags/interlock | 9 |
| `rtl/input_packer.sv` | binarize, compact, bound, pack image | 10 |
| `rtl/bnn_fcc.sv` | top-level integration | 11 |
| `verification/unit/tb_*.sv` | unit and system tests (new) | 1–13 |
| `sim/run_unit.sh`, `sim/run_regress.sh`, `sim/lint.sh`, `sim/common.sh` | scripts (new) | 1, 11, 14 |
| `sim/cfg/*.sv` | wrappers instantiating `bnn_fcc_tb` with overrides (new) | 11 |
| `.gitignore` | ignore `sim/build/`, `openflex/` run outputs | 1, 14 |

---

### Task 1: Simulation infrastructure, package, popcount

**Files:**
- Create: `rtl/bnn_fcc_pkg.sv`, `rtl/popcount.sv`, `sim/common.sh`, `sim/run_unit.sh`, `sim/lint.sh`, `verification/unit/tb_popcount.sv`, `.gitignore`

**Interfaces:**
- Produces (package `bnn_fcc_pkg`, all `function automatic int`):
  - `cdiv(int a, int b)` = ceil(a/b); `imin(int a, int b)`; `imax(int a, int b)`; `bits(int x)` = `imax(1, $clog2(x))`
  - `popcount_latency(int w, int lps)`: `T = 1 + $clog2(cdiv(w,6))` levels (level 0 = 6-bit chunk count, then binary tree); returns `cdiv(T, lps)` (≥ 1; output always registered)
  - `np_latency(int w, int lps)` = `popcount_latency(w, lps) + 1`
- Produces module `popcount #(int W = 8, int LEVELS_PER_STAGE = 3) (input logic clk, input logic [W-1:0] in_bits, output logic [bits(W+1)-1:0] count)`; `count` = `$countones(in_bits)` from exactly `popcount_latency(W, LEVELS_PER_STAGE)` cycles earlier; no reset, no valid.
- Produces scripts:
  - `sim/common.sh`: sources Vivado settings; defines `REPO`, `RTL_FILES` (`rtl/bnn_fcc_pkg.sv` first, then every other `rtl/*.sv`), `TBPKG_FILES` (`verification/axi4_stream_if.sv verification/bnn_fcc_tb_pkg.sv`), function `run_xsim <name> <top> <files...> [-- <xelab extra args>]` that builds in `sim/build/<name>/`, runs `xvlog -sv`, `xelab -debug off --timescale 1ns/100ps`, `xsim -R`, writes `sim/build/<name>/run.log`, and returns 0 iff the pass rule (Global Constraints) holds, printing `PASS <name>` / `FAIL <name> (log path)`.
  - `sim/run_unit.sh [tb ...]`: default list = every `verification/unit/tb_*.sv`; compiles `RTL_FILES TBPKG_FILES <tb file>` with top = file basename; if the TB file contains the string `BASE_DIR`, adds `-generic_top BASE_DIR=$REPO/python`; exit code = number of failures.
  - `sim/lint.sh`: `verilator --lint-only -Wall --top-module bnn_fcc $RTL_FILES` (no waivers yet; Task 14 makes it clean).

- [ ] **Step 1: Write `verification/unit/tb_popcount.sv`**

Generate block instantiating `popcount` for every (W, LPS) in W ∈ {1,5,6,7,8,16,43,49,64,66,131}, LPS ∈ {1,3}. Per instance: drive 2000 random vectors (plus all-zeros and all-ones) one per cycle, push expected `$countones` into a queue, compare each output with the value pushed `popcount_latency(W,LPS)` cycles earlier. Also `assert` that `popcount_latency(66,3) == 2` and `popcount_latency(1,3) == 1`. On completion print `SUCCESS: tb_popcount`; on mismatch `$error("W=%0d LPS=%0d exp=%0d got=%0d", ...)`.

- [ ] **Step 2: Write `sim/common.sh`, `sim/run_unit.sh`, `.gitignore` (`sim/build/`), then run and confirm failure**

Run: `sim/run_unit.sh tb_popcount`
Expected: `FAIL tb_popcount` (xvlog error: `popcount`/`bnn_fcc_pkg` not found).

- [ ] **Step 3: Implement `rtl/bnn_fcc_pkg.sv` and `rtl/popcount.sv`**

Popcount: level 0 counts each 6-bit chunk (last chunk zero-extended); a binary adder tree sums chunk counts; insert a register after every `LEVELS_PER_STAGE` levels counting level 0 as level 1, and always register the final output (so register count equals `popcount_latency`).

- [ ] **Step 4: Run test**

Run: `sim/run_unit.sh tb_popcount`
Expected: `PASS tb_popcount`

- [ ] **Step 5: Write `sim/lint.sh`; run `verilator --lint-only -Wall rtl/bnn_fcc_pkg.sv rtl/popcount.sv --top-module popcount`**

Expected: no errors (warnings allowed until Task 14).

- [ ] **Step 6: Commit**

```bash
git add rtl/bnn_fcc_pkg.sv rtl/popcount.sv sim/*.sh verification/unit/tb_popcount.sv .gitignore
git commit -m "Add package, pipelined popcount, xsim scripts"
```

---

### Task 2: bit_packer

**Files:**
- Create: `rtl/bit_packer.sv`, `verification/unit/tb_bit_packer.sv`

**Interfaces:**
- Consumes: `bnn_fcc_pkg::bits`.
- Produces:
```systemverilog
module bit_packer #(int IN_W = 8, int OUT_W = 8, bit PAD = 1'b0) (
  input  logic clk, rst, clear,                     // clear: synchronous, drops buffer and flush state
  input  logic in_valid, output logic in_ready,     // in_ready from registered state only
  input  logic [IN_W-1:0] in_bits,                  // valid bits are in_bits[in_count-1:0], LSB first
  input  logic [bits(IN_W+1)-1:0] in_count,         // 0..IN_W
  input  logic in_flush,                            // flush after this input
  output logic out_valid, output logic [OUT_W-1:0] out_word, // consumer always accepts
  output logic flush_done                           // pulses in the cycle the flush completes
);
```
- Behavior (spec §4.4): buffer width `OUT_W+IN_W`; each cycle emit low OUT_W bits if `count ≥ OUT_W`; `in_ready = (post_emit_count < OUT_W) && !flush_pending`. Accepted input with `in_flush` sets `flush_pending`. While pending: if `0 < post_emit_count < OUT_W` emit remaining bits padded with `PAD` and complete; if `post_emit_count == 0` complete. `flush_done` is asserted in the completion cycle, coinciding with the final word's `out_valid` when there is one. A flush whose bits end exactly on a word boundary emits no extra word.

- [ ] **Step 1: Write `tb_bit_packer.sv`**

Generate over (IN_W, OUT_W, PAD) ∈ {8}×{1,3,5,8,16,49}×{0,1} plus (IN_W=1,OUT_W=1,PAD=0) and (IN_W=8,OUT_W=262,PAD=0). Model: a bit queue fed with accepted bits; on flush, pad the queue to a multiple of OUT_W with PAD and record a "flush boundary". Stimulus: 3000 cycles of random `in_valid` (70%), random `in_count` 0..IN_W, random bits (bits above `in_count` randomized too, must be ignored), `in_flush` 10%; one directed flush where accumulated bits are an exact multiple of OUT_W. Checks: every `out_word` equals the next OUT_W model bits; `flush_done` occurs exactly once per flush, in the cycle of that flush's last word or (no word) after the boundary; when OUT_W ≥ IN_W, `in_ready` is never low except during a pending flush. Print `SUCCESS: tb_bit_packer`.

- [ ] **Step 2: Run, expect FAIL** — `sim/run_unit.sh tb_bit_packer` → `FAIL` (module missing).
- [ ] **Step 3: Implement `rtl/bit_packer.sv`.**
- [ ] **Step 4: Run, expect PASS** — `sim/run_unit.sh tb_bit_packer` → `PASS tb_bit_packer`.
- [ ] **Step 5: Commit** — `git commit -m "Add bit_packer"` (with both files added).

---

### Task 3: pingpong_buf

**Files:**
- Create: `rtl/pingpong_buf.sv`, `verification/unit/tb_pingpong_buf.sv`

**Interfaces:**
```systemverilog
module pingpong_buf #(int NBITS = 8, int WR_W = 8, int RD_W = 8) (
  input  logic clk, rst,
  input  logic wr_en, wr_bank, input logic [bits(cdiv(NBITS,WR_W))-1:0] wr_idx, input logic [WR_W-1:0] wr_data,
  input  logic commit_en, commit_bank,              // sets full[commit_bank]
  input  logic rd_bank, input logic [bits(cdiv(NBITS,RD_W))-1:0] rd_idx,
  output logic [RD_W-1:0] rd_data,                  // registered: bits [rd_idx*RD_W +: RD_W] of rd_bank, previous cycle's request
  input  logic rls_en, rls_bank,                    // clears full[rls_bank]
  output logic [1:0] full
);
```
- Storage: per bank `NBITS` flops; bit `j` written from `wr_data[j % WR_W]` when `wr_en && wr_bank == b && wr_idx == j / WR_W`; read bit `k` of word `i` = stored bit `i*RD_W + k` if `< NBITS` else `0` (constant per position; mux over `cdiv(NBITS,RD_W)` words). `rst` clears `full` only (data not reset). Same-cycle `commit_en` and `rls_en` on different banks both take effect.

- [ ] **Step 1: Write `tb_pingpong_buf.sv`**

Instances: (NBITS,WR_W,RD_W) ∈ {(784,66,66), (256,32,66), (256,11,66), (10,1,66), (13,2,3), (7,4,2), (5,8,8), (100,7,11)}. Model: two bit arrays. Per instance, 200 iterations: pick a bank, write all chunks in random order with random gaps, commit, check `full`, read every word index in random order and compare (including zero padding beyond NBITS), release, check `full` cleared. Also overlap: write bank 1 while reading bank 0. Print `SUCCESS: tb_pingpong_buf`.

- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement `rtl/pingpong_buf.sv`.**
- [ ] **Step 4: Run, expect PASS** — `PASS tb_pingpong_buf`.
- [ ] **Step 5: Commit** — `git commit -m "Add width-converting pingpong_buf"`.

---

### Task 4: neuron_processor

**Files:**
- Create: `rtl/neuron_processor.sv`, `verification/unit/tb_neuron_processor.sv`

**Interfaces:**
```systemverilog
module neuron_processor #(int PW = 8, int ACC_W = 4, int THR_W = 4, int POPCOUNT_LEVELS_PER_STAGE = 3) (
  input  logic clk, rst,
  input  logic in_valid, first_pass, last_pass,
  input  logic [PW-1:0] x, w,
  input  logic [THR_W-1:0] thr,          // only used with last_pass; presented in the same cycle as x/w
  output logic out_valid,                // pulses np_latency(PW,LPS) cycles after an in_valid&&last_pass input
  output logic out_y,                    // (final popcount sum) >= thr
  output logic [ACC_W-1:0] out_acc       // final popcount sum
);
```
- `rst` clears only `out_valid` and internal valid pipeline; accumulator/data not reset. The accumulator updates only for valid inputs; `first_pass` loads, otherwise adds.

- [ ] **Step 1: Write `tb_neuron_processor.sv`**

Instances PW ∈ {1,8,66} with ACC_W = bits(F+1), THR_W = bits(F+2) for F = 3*PW. For 500 neurons each: random PASSES ∈ 1..6, random weights/inputs per pass, random `in_valid` gaps between passes (in_valid low cycles must not disturb accumulation), threshold chosen from {0, F, F+1, random}. Expect `out_acc` = sum of `$countones(~(x^w))` over the passes and `out_y = out_acc >= thr`, arriving exactly `np_latency(PW,3)` cycles after the last pass. Print `SUCCESS: tb_neuron_processor`.

- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** — instantiates `popcount`; delays `in_valid/first_pass/last_pass/thr` by `popcount_latency`.
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** — `git commit -m "Add neuron_processor"`.

---

### Task 5: bnn_layer

**Files:**
- Create: `rtl/bnn_layer.sv`, `verification/unit/tb_bnn_layer.sv`

**Interfaces:**
- Consumes: `neuron_processor`, `pingpong_buf` (TB only), package functions.
- Produces (localparams inside: `GROUPS, PASSES, ACC_W, THR_W, LANE_W = bits(PN), WADDR_W = bits(GROUPS*PASSES), TADDR_W = bits(GROUPS), IN_IDX_W = bits(PASSES), OUT_IDX_W = bits(GROUPS)`):
```systemverilog
module bnn_layer #(int F = 8, int N = 8, int PW = 8, int PN = 8, bit IS_OUTPUT = 0,
                   int POPCOUNT_LEVELS_PER_STAGE = 3, string RAM_STYLE = "auto") (
  input  logic clk, rst,
  // config writes (from cfg_layer_writer)
  input  logic w_we, input logic [LANE_W-1:0] w_lane, input logic [WADDR_W-1:0] w_addr, input logic [PW-1:0] w_data,
  input  logic t_we, input logic [LANE_W-1:0] t_lane, input logic [TADDR_W-1:0] t_addr, input logic [THR_W-1:0] t_data,
  // input ping-pong (consumer side)
  input  logic [1:0] in_full, output logic in_rd_bank, output logic [IN_IDX_W-1:0] in_rd_idx,
  input  logic [PW-1:0] in_rd_data, output logic in_rls_en, in_rls_bank,
  // hidden-layer output ping-pong (producer side); tie out_full to 0 when IS_OUTPUT
  input  logic [1:0] out_full, output logic out_wr_en, out_wr_bank, output logic [OUT_IDX_W-1:0] out_wr_idx,
  output logic [PN-1:0] out_wr_data, output logic out_commit_en, out_commit_bank,
  // output-layer result stream; tie start_ok to 1 when !IS_OUTPUT
  input  logic start_ok, output logic res_valid, output logic [PN*ACC_W-1:0] res_counts,
  output logic [PN-1:0] res_mask, output logic res_last,
  output logic img_start,   // pulse when an image starts issuing
  output logic busy         // issuing or any data in flight
);
```
- Weight layout contract (shared with Task 7): neuron `n` lives in lane `n % PN`, group `n / PN`; its pass `p` word is at `w_addr = group*PASSES + p`; bit `k` of that word is weight index `p*PW + k` (pad bits are 1). Threshold of neuron `n` at lane `n % PN`, `t_addr = group`. Hidden output chunk `g` bit `k` = neuron `g*PN + k` (0 if `≥ N`). `res_counts[k*ACC_W +: ACC_W]` = popcount of neuron `g*PN + k`, `res_mask[k] = (g*PN + k < N)`, `res_last` on the last group.
- Sequencer, RAMs, lane validity and release timing per spec §5.1. Weight RAM and threshold RAM per lane in a generate loop with `(* ram_style = RAM_STYLE *)` on the weight array; 1-cycle registered read aligned with the ping-pong `rd_data` latency.

- [ ] **Step 1: Write `tb_bnn_layer.sv`**

The TB contains a `pingpong_buf` on the input side (and, for hidden layers, on the output side) plus a reference model. Configurations (generate): (F,N,PW,PN,IS_OUTPUT) ∈ {(784,256,66,32,0), (256,256,66,11,0), (256,10,66,1,1), (13,7,3,2,0), (7,5,2,4,0), (5,3,4,3,1), (8,8,8,8,0), (1,1,1,1,0), (20,9,64,9,1), (10,10,3,10,1)}. Per configuration: write random weights/thresholds via the config ports using the layout contract (thresholds include 0, F, F+1); then 20 random images: producer writes the input bank chunk-by-chunk with random delays and commits; the TB consumer of the output bank waits a random 0–200 cycles before reading and releasing (hidden), or `start_ok` toggles randomly (output). Check: every hidden output bit equals `popcount(xnor) >= thr` and padding lanes are 0; every output-layer count/mask/last matches; images complete in order; no-bubble check: whenever the next input bank is already full and the next output bank free (or `start_ok`) in the cycle the last `(g,p)` of an image issues, the next `img_start` occurs in the very next cycle (use a phase where the TB pre-fills both input banks and the output consumer releases immediately). Print `SUCCESS: tb_bnn_layer`.

- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement `rtl/bnn_layer.sv`.**
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** — `git commit -m "Add bnn_layer engine"`.

---

### Task 6: argmax and axis_out

**Files:**
- Create: `rtl/argmax.sv`, `rtl/axis_out.sv`, `verification/unit/tb_argmax.sv`, `verification/unit/tb_axis_out.sv`

**Interfaces:**
```systemverilog
module argmax #(int PN = 1, int ACC_W = 4, int N = 1, int IDX_W = 4, int LEVELS_PER_STAGE = 2) (
  input  logic clk, rst,
  input  logic in_valid, input logic [PN*ACC_W-1:0] in_counts, input logic [PN-1:0] in_mask, input logic in_last,
  output logic out_valid, output logic [IDX_W-1:0] out_idx,   // one pulse per image, after its in_last
  output logic busy
);
module axis_out #(int DATA_W = 4, int BUS_W = 8, int DEPTH = 4) (
  input  logic clk, rst,
  input  logic push, input logic [DATA_W-1:0] push_data,       // push never occurs when full (credit-protected)
  output logic [bits(DEPTH+1)-1:0] count,                     // entries held
  output logic m_valid, input logic m_ready,
  output logic [BUS_W-1:0] m_data, output logic [BUS_W/8-1:0] m_keep, output logic m_last
);
```
- Argmax: lowest index wins ties within a group and across groups (later group replaces only if strictly greater); masked lanes ignored. `axis_out`: `m_data` = zero-extended entry; `m_keep` low `cdiv(DATA_W,8)` bits set; `m_last = 1`; `m_valid`/`m_data` from registers; AXI-compliant hold.

- [ ] **Step 1: Write `tb_argmax.sv`** — instances (PN,N) ∈ {(1,10),(10,10),(3,10),(4,3),(2,5)}; 500 images each; counts drawn from a small range 0..3 to force many ties, plus all-equal images; random `in_valid` gaps between groups; expected = lowest index of the maximum over valid neurons. Print `SUCCESS: tb_argmax`.
- [ ] **Step 2: Write `tb_axis_out.sv`** — DEPTH ∈ {1,4}; random push (respecting `count < DEPTH`) and random `m_ready`; check order, no loss/duplication, `m_valid` never drops without a handshake, `m_keep == 8'h01` and `m_last == 1` for (DATA_W=4,BUS_W=8). Print `SUCCESS: tb_axis_out`.
- [ ] **Step 3: Run both, expect FAIL.**
- [ ] **Step 4: Implement both modules.**
- [ ] **Step 5: Run both, expect PASS.**
- [ ] **Step 6: Commit** — `git commit -m "Add argmax and axis_out"`.

---

### Task 7: cfg_layer_writer

**Files:**
- Create: `rtl/cfg_layer_writer.sv`, `verification/unit/tb_cfg_layer_writer.sv`

**Interfaces:**
```systemverilog
module cfg_layer_writer #(int F = 8, int N = 8, int PW = 8, int PN = 8, bit HAS_THR = 1) (
  input  logic clk, rst,
  input  logic start,               // pulse: new message for this layer; resets counters and packer
  input  logic abort,               // pulse: truncated message; resets counters and packer
  input  logic msg_type,            // 0 = weights, 1 = thresholds; stable from start to end of message
  input  logic s_valid, output logic s_ready, input logic [7:0] s_byte,
  // same widths as bnn_layer's config ports
  output logic w_we, output logic [LANE_W-1:0] w_lane, output logic [WADDR_W-1:0] w_addr, output logic [PW-1:0] w_data,
  output logic t_we, output logic [LANE_W-1:0] t_lane, output logic [TADDR_W-1:0] t_addr, output logic [THR_W-1:0] t_data
);
```
- `LANE_W, WADDR_W, TADDR_W, THR_W, GROUPS, PASSES` are computed exactly as in `bnn_layer` (Task 5).
- Weights per spec §4.3 using `bit_packer #(8, PW, 1'b1)`, writing the Task 5 layout contract; lane/group advance on `flush_done`. Thresholds: 4 bytes little-endian → signed 32-bit → clamp to `[0, F+1]`.

- [ ] **Step 1: Write `tb_cfg_layer_writer.sv`** — instances (F,N,PW,PN) ∈ {(784,256,66,32), (13,7,3,2), (7,5,1,4), (8,8,8,8), (256,10,66,1), (20,9,64,9)}. Feed the exact payload bytes `BNN_FCC_Model::get_layer_config` would produce (build them in the TB from random weights: per neuron `ceil(F/8)` bytes, LSB-first, pad bits 1) with random `s_valid` gaps; capture writes into shadow memories; check every word against the layout contract (pad bits 1) and that no write lands outside lane < PN, addr < GROUPS*PASSES. Thresholds: values {-5, 0, F, F+1, F+7, 2^31-1, random} → stored clamp. Also: `abort` mid-message then a full message → shadow memories equal the full message. Print `SUCCESS: tb_cfg_layer_writer`.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** — `git commit -m "Add cfg_layer_writer"`.

---

### Task 8: cfg_byte_serializer

**Files:**
- Create: `rtl/cfg_byte_serializer.sv`, `verification/unit/tb_cfg_byte_serializer.sv`

**Interfaces:**
```systemverilog
module cfg_byte_serializer #(int BUS_W = 64) (
  input  logic clk, rst,
  input  logic s_valid, output logic s_ready, input logic [BUS_W-1:0] s_data, input logic [BUS_W/8-1:0] s_keep, input logic s_last,
  output logic m_valid, input logic m_ready,
  output logic [7:0] m_byte, output logic m_null, output logic m_end_of_beat, output logic m_beat_last,
  input  logic drop_rest           // with an m_valid&&m_ready handshake: discard the rest of this beat
);
```
- Behavior per spec §4.1 (`s_ready` from registered state only; keep=0 & last=1 → one `m_null` item; keep=0 & last=0 → nothing).

- [ ] **Step 1: Write `tb_cfg_byte_serializer.sv`** — BUS_W ∈ {8, 32, 64, 128}; 2000 beats with random keep (25% full, 25% all-zero, 50% random sparse), random last, random `s_valid` gaps and random `m_ready`; `drop_rest` randomly on 10% of handshakes. Model: expected item queue (respecting drops). Check items and flags; check `s_ready` has no combinational dependence on `s_valid`: in a directed phase, toggle `s_valid` 0→1 after a `#1` delay within a cycle and assert `s_ready` sampled before and after the toggle is equal. Print `SUCCESS: tb_cfg_byte_serializer`.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** — `git commit -m "Add cfg_byte_serializer"`.

---

### Task 9: cfg_parser

**Files:**
- Create: `rtl/cfg_parser.sv`, `verification/unit/tb_cfg_parser.sv`

**Interfaces:**
```systemverilog
module cfg_parser #(int TOTAL_LAYERS = 4, int TOPOLOGY[TOTAL_LAYERS] = '{784,256,256,10}) (
  input  logic clk, rst,
  input  logic s_valid, output logic s_ready, input logic [7:0] s_byte, input logic s_null, s_beat_last,
  output logic drop_rest,
  input  logic pipeline_idle,
  output logic cfg_busy, configured,
  output logic msg_start, msg_abort,          // pulses
  output logic [7:0] msg_layer, output logic msg_type,  // valid from msg_start through end of PAYLOAD
  output logic pl_valid, input logic pl_ready, output logic [7:0] pl_byte
);
```
- FSM, validation table, WAIT_IDLE second-cycle rule, truncation, discard, load flags exactly per spec §4.2. A DISCARD whose `total_bytes == 0` returns to HEADER immediately without `drop_rest`.

- [ ] **Step 1: Write `tb_cfg_parser.sv`** (TOPOLOGY `'{13,7,5,3}`) — drive bytes directly (no serializer); `pl_ready` random; `pipeline_idle` driven by the TB. Tests: (a) valid messages for every layer/type in shuffled order → `configured` rises only after the last required message, `pl_byte` stream equals payloads, `drop_rest` on each last payload byte; (b) each invalid header kind (msg_type 2, layer_id 3 for weights, layer_id 2 for thresholds, wrong `layer_inputs`, wrong `num_neurons`, wrong `bytes_per_neuron`, wrong `total_bytes`) → payload consumed with `pl_valid` never asserted, flags unchanged; (c) truncated weights message (`s_beat_last` early) → `msg_abort`, that layer's flag cleared, `configured` falls; (d) WAIT_IDLE: with `pipeline_idle = 0`, `s_ready` stays 0 and `cfg_busy = 1`; raising idle for one cycle in the first WAIT_IDLE cycle is ignored, from the second cycle it proceeds; (e) TLAST during header discards the partial header. Print `SUCCESS: tb_cfg_parser`.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** — `git commit -m "Add cfg_parser"`.

---

### Task 10: input_packer

**Files:**
- Create: `rtl/input_packer.sv`, `verification/unit/tb_input_packer.sv`

**Interfaces:**
```systemverilog
module input_packer #(int DATA_W = 8, int BUS_W = 64, int F0 = 784, int PW0 = 66) (
  input  logic clk, rst,
  input  logic s_valid, output logic s_ready, input logic [BUS_W-1:0] s_data, input logic [BUS_W/8-1:0] s_keep, input logic s_last,
  input  logic accept_new,                 // configured && !cfg_busy (registered at top)
  input  logic [1:0] buf_full,             // layer-0 pingpong full flags
  output logic wr_en, wr_bank, output logic [bits(cdiv(F0,PW0))-1:0] wr_idx, output logic [PW0-1:0] wr_data,
  output logic commit_en, commit_bank,
  output logic mid_image
);
```
- Spec §6.1; uses `bit_packer #(BUS_W/DATA_W, PW0, 1'b0)`; commit on the packer's `flush_done` after TLAST.

- [ ] **Step 1: Write `tb_input_packer.sv`** — instances (BUS_W,F0,PW0) ∈ {(64,784,66), (8,13,3), (128,100,7), (64,5,1), (32,20,64)}; a TB `pingpong_buf` captures writes; the TB releases committed banks after a random delay. 30 images each with random valid gaps; keep patterns: full beats plus a TKEEP-trimmed last beat, and (separately) sparse keep inside beats. Checks: committed bank contents = binarized (MSB) valid pixels in order, zero-padded; one image with extra pixels before TLAST → only first F0 bits stored; `s_ready` low while `accept_new = 0` at an image boundary but stays governed by the packer mid-image; `s_ready` never depends combinationally on `s_valid`. Print `SUCCESS: tb_input_packer`.
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run, expect PASS.**
- [ ] **Step 5: Commit** — `git commit -m "Add input_packer"`.

---

### Task 11: bnn_fcc top-level integration and provided-TB regression

**Files:**
- Modify: `rtl/bnn_fcc.sv` (replace stub body; keep the existing port list and parameter names; add the four implementation parameters after `PARALLEL_NEURONS`)
- Modify: `verification/bnn_fcc_tb.sv:117-118` — defaults only: `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS[NON_INPUT_LAYERS] = '{32, 11, 1}`
- Create: `sim/run_regress.sh`, `sim/cfg/<name>.sv` for each config below

**Interfaces:**
- Consumes: every module above with the exact port lists in Tasks 1–10.
- Structure: generate block `g_layer[l]` (l = 0..L-1) holds the layer-l input `pingpong_buf` (`NBITS = F[l]`, `WR_W = (l==0) ? PW[0] : PN[l-1]`, `RD_W = PW[l]`), `cfg_layer_writer`, and `bnn_layer`; cross-layer wiring uses `g_layer[l-1].*` / `g_layer[l+1].*`. Config routing: `start = msg_start && msg_layer == l`, `pl_ready = |(sel_l & writer_ready_l)`. Credits: `in_flight` +1 on output-layer `img_start`, −1 on `argmax.out_valid`; `start_ok = (axis_out.count + in_flight) < OUT_FIFO_DEPTH` (registered). `pipeline_idle` per spec §6.3 (registered). `accept_new = configured && !cfg_busy`. Elaboration assertions from spec §3.1 in an `initial` block with `$fatal`.
- `sim/cfg/<name>.sv`: `module cfg_<name>; bnn_fcc_tb #( ...overrides... ) tb(); endmodule` — run with top `cfg_<name>`.

Regression configs (all `USE_CUSTOM_TOPOLOGY = 1`; "stress" = `TOGGLE_DATA_OUT_READY=1, CONFIG_VALID_PROBABILITY=0.8, DATA_IN_VALID_PROBABILITY=0.8`; `TIMEOUT = 100ms` for all):

| name | CUSTOM_LAYERS / TOPOLOGY | PI / PN | other |
|---|---|---|---|
| `sfc_stress` | 4 / `'{784,256,256,10}` | 66 / `'{32,11,1}` | stress, 50 images |
| `sfc_perf` | 4 / `'{784,256,256,10}` | 66 / `'{32,11,1}` | TOGGLE=0, 1.0/1.0, 100 images |
| `sfc_pn1` | 4 / `'{784,256,256,10}` | 66 / `'{1,1,1}` | stress, 50 images |
| `sfc_alt` | 4 / `'{784,256,256,10}` | 33 / `'{64,22,1}` | stress, 50 images |
| `sfc_orig` | 4 / `'{784,256,256,10}` | 8 / `'{8,8,10}` | stress, 20 images |
| `c_8888` | 4 / `'{8,8,8,8}` | 66 / `'{32,11,1}` | stress, 200 images |
| `c_13753` | 4 / `'{13,7,5,3}` | 3 / `'{2,4,3}` | stress, 200 images |
| `c_100` | 4 / `'{100,37,19,10}` | 7 / `'{5,3,4}` | stress, 200 images |
| `c_tiny` | 4 / `'{5,1,3,2}` | 1 / `'{1,1,1}` | stress, 200 images |
| `c_100_bus` | 4 / `'{100,37,19,10}` | 7 / `'{5,3,4}` | stress, 200 images, `CONFIG_BUS_WIDTH=32`, `INPUT_BUS_WIDTH=128` |
| `c_13753_bus8` | 4 / `'{13,7,5,3}` | 3 / `'{2,4,3}` | stress, 200 images, `CONFIG_BUS_WIDTH=8`, `INPUT_BUS_WIDTH=8` |
| `c_5layer` | 5 / `'{30,17,9,12,6}` | 5 / `'{4,9,5,6}` | stress, 200 images |

- [ ] **Step 1: Write the `sim/cfg/*.sv` wrappers and `sim/run_regress.sh [name ...]`** (default: all; compiles `RTL_FILES TBPKG_FILES verification/bnn_fcc_tb.sv sim/cfg/<name>.sv`, top `cfg_<name>`; exit code = failures; for `sfc_perf` also print the TB's `Avg latency (cycles)` and `Avg throughput (cycles/output)` lines).
- [ ] **Step 2: Run against the stub, expect FAIL** — `sim/run_regress.sh c_8888` → `FAIL` (timeout: stub produces no outputs).
- [ ] **Step 3: Edit `verification/bnn_fcc_tb.sv` lines 117–118 (defaults only) and implement `rtl/bnn_fcc.sv`.**
- [ ] **Step 4: Run small configs first** — `sim/run_regress.sh c_tiny c_8888 c_13753 c_13753_bus8 c_100 c_100_bus c_5layer` → 7× `PASS`.
- [ ] **Step 5: Run SFC configs** — `sim/run_regress.sh sfc_stress sfc_pn1 sfc_alt sfc_orig sfc_perf` → 5× `PASS`; `sfc_perf` prints `Avg throughput (cycles/output)` ≤ 105.0 and `Avg latency (cycles)` between 330 and 400. If outside, stop and report numbers before changing anything.
- [ ] **Step 6: Re-run all unit tests** — `sim/run_unit.sh` → all `PASS`.
- [ ] **Step 7: Commit** — `git add rtl/bnn_fcc.sv verification/bnn_fcc_tb.sv sim/run_regress.sh sim/cfg && git commit -m "Integrate bnn_fcc top level; add provided-TB regression"`.

---

### Task 12: tb_mnist (real trained model under xsim)

**Files:**
- Create: `verification/unit/tb_mnist.sv`

**Interfaces:**
- Consumes: `bnn_fcc_tb_pkg::{BNN_FCC_Model, BNN_FCC_Stimulus, LatencyTracker, ThroughputTracker}` (unmodified); `bnn_fcc` with defaults `PARALLEL_INPUTS = 66, PARALLEL_NEURONS = '{32,11,1}`.
- Produces: `class BNN_FCC_Model_XsimSafe #(int BUS_WIDTH = 64) extends bnn_fcc_tb_pkg::BNN_FCC_Model #(BUS_WIDTH)` overriding `load_from_file(string data_dir, int expected_topology[])` and `load_single_layer(...)` with the same file format, declaring the per-line bit queue outside the read loop and calling `delete()` each line. Parameter `string BASE_DIR = "../python"` (overridden to the absolute path by `run_unit.sh`, Task 1).

- [ ] **Step 1: Write `tb_mnist.sv`**

Steps inside: load model with the subclass; for all 100 images assert `compute_reference(img) == expected_outputs[i]` (fatal if not — proves the loader); `encode_configuration`; drive config with 0.8 valid probability; drive 100 images (64-bit bus, TKEEP on the last beat) with 0.8 valid probability; random `data_out_ready`; check every output against `expected_outputs.txt` in order; also check `data_out_keep == 8'h01`, `data_out_last == 1`; print accuracy-equivalence line `MNIST: 100/100 match python` and `SUCCESS: tb_mnist`.

- [ ] **Step 2: Run** — `sim/run_unit.sh tb_mnist` → `PASS tb_mnist` (DUT already exists; if it fails, debug the DUT, not the reference — the reference self-check runs first).
- [ ] **Step 3: Commit** — `git commit -m "Add xsim-safe MNIST system test"`.

---

### Task 13: tb_cfg_sequences (configuration ordering, reconfiguration, misuse, reset)

**Files:**
- Create: `verification/unit/tb_cfg_sequences.sv`

**Interfaces:**
- Consumes: `bnn_fcc_tb_pkg::BNN_FCC_Model` (`create_random`, `get_layer_config`, `compute_reference`, public `weight`/`threshold` fields); `bnn_fcc` with TOPOLOGY `'{20,12,9,5}`, `PARALLEL_INPUTS = 4`, `PARALLEL_NEURONS = '{3,4,2}`.
- TB-local tasks (names fixed so failures are traceable): `send_msg(stream, keep, bit tlast_on_last_beat)`, `send_image(img, bit zero_gap)`, `expect_outputs(model_q)`, plus a monitor checking outputs against a queue of expected classes.

- [ ] **Step 1: Write the tests, each printing `PASS <test>` and run in sequence with a reset between them:**
  - `test_reordered` — all threshold messages in reverse layer order, then weights in reverse order; 50 images match `compute_reference`.
  - `test_not_configured` — after sending all but one required message, `data_in_ready` stays 0 for 1000 cycles; after the last message it rises.
  - `test_weights_only_reconfig` — after 20 images, replace layer 1 weights only (model B = model A with layer 1 weights from a fresh random model); next 20 images match model B.
  - `test_thresholds_only_reconfig` — same for layer 0 thresholds.
  - `test_reconfig_while_streaming` — images stream continuously while a layer-2 weights message is sent; every output matches model A or model B, and once one matches only B, none later matches only A.
  - `test_invalid_headers` — send each invalid header kind from Task 9 (b) plus an output-layer threshold message, each with a random payload of its `total_bytes`; then 20 images still match the unchanged model.
  - `test_truncated` — send layer-0 weights with TLAST on an early beat → `data_in_ready` falls to 0 within 10 cycles after the configuration drains; resend the full message → images match.
  - `test_per_message_tlast` — full configuration with TLAST on every message's last beat (README semantics); 20 images match.
  - `test_reset_mid_config` / `test_reset_mid_image` — assert `rst` mid-stream, reconfigure from scratch, 20 images match.
  - `test_zero_gap_images` (Review Focus 1) — valid probability 1.0, `zero_gap = 1`, `data_out_ready = 1`; 100 images match and consecutive outputs are ≤ `max(beats_per_image, max layer cycles) + 2` cycles apart in steady state.
  - `test_long_backpressure` (Review Focus 2) — `data_out_ready = 0` for 5000 cycles while 30 images are offered; `data_in_ready` must fall; then ready = 1; all 30 outputs correct and in order.
  - `test_reset_with_output_pending` (Review Focus 5) — leave an output pending (ready = 0), reset: `data_out_valid` = 0 the cycle after reset; reconfigure; next 10 outputs match and no extra output appears.
  - Final line `SUCCESS: tb_cfg_sequences`.
- [ ] **Step 2: Run** — `sim/run_unit.sh tb_cfg_sequences` → `PASS`. Failures are DUT bugs: fix the RTL, rerun this TB, then rerun `sim/run_unit.sh` and `sim/run_regress.sh`.
- [ ] **Step 3: Commit** — `git commit -m "Add configuration sequencing/reconfig/reset system tests"`.

---

### Task 14: Lint, openflex synthesis, results

**Files:**
- Modify: `openflex/bnn_fcc_timing.yml` and `openflex/bnn_fcc_verify.yml` — file lists only: add `- ../rtl/bnn_fcc_pkg.sv` first, then every other new `../rtl/*.sv` (keep existing entries)
- Modify: `openflex/rtl/bnn_fcc_timing.sv:24-25` — defaults only: `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS[TOTAL_LAYERS-1] = '{32, 11, 1}`
- Modify: `rtl/*.sv` only as lint requires; `.gitignore` (add openflex run outputs, e.g. `openflex/*.csv`, `openflex/run.log`, generated project dirs, after seeing what openflex creates)

- [ ] **Step 1: Lint** — `sim/lint.sh` → no warnings, or each remaining warning suppressed with an inline `/* verilator lint_off <ID> */ ... lint_on` plus a one-line justification comment.
- [ ] **Step 2: Re-run everything after lint edits** — `sim/run_unit.sh && sim/run_regress.sh` → all `PASS`.
- [ ] **Step 3: Edit the openflex file lists and timing-wrapper defaults.**
- [ ] **Step 4: Synthesize** — `source ~/envs/openflex/bin/activate && source /tools/AMD/2025.2/Vivado/settings64.sh && cd openflex && openflex bnn_fcc_timing.yml -c bnn_fcc.csv`. Expected: CSV row with `fMax` ≥ 100. If below: lower the DUT defaults `POPCOUNT_LEVELS_PER_STAGE`/`ARGMAX_LEVELS_PER_STAGE` to 1 in `rtl/bnn_fcc.sv` (the timing wrapper may not be changed for these), rerun Step 2 and Step 4, and report the critical path if still below.
- [ ] **Step 5: Record results** — extract fMax, LUT, LUTAsMem, REG, BRAM, URAM, DSP from `openflex/bnn_fcc.csv`; latency and cycles/output from `sim/build/sfc_perf/run.log`; MNIST match line from `sim/build/tb_mnist/run.log`. Report them to the user (no new documentation files).
- [ ] **Step 6: Commit** — `git add openflex/bnn_fcc_timing.yml openflex/bnn_fcc_verify.yml openflex/rtl/bnn_fcc_timing.sv rtl .gitignore && git commit -m "Lint clean; openflex file lists and timing defaults"` (do not commit generated synthesis outputs).
