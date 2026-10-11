# BNN FCC Design Spec — Layer-Pipelined, Arbitrary-Parallelism Implementation

Date: 2026-10-10
Status: Approved (revised 2026-10-10: xsim TB-loader finding, `tb_mnist`)
Scope: complete `rtl/bnn_fcc.sv` (AXI4-Stream binary neural network, fully connected classifier, MNIST SFC topology 784→256→256→10) in SystemVerilog, verify with Vivado xsim, synthesize with Vivado via openflex.

---

## 1. Intent and agreed requirements

### 1.1 Stated by the user
| # | Requirement |
|---|---|
| U1 | Complete the design in SystemVerilog using Vivado tools (Vivado 2025.2 at `/tools/AMD/2025.2`). |
| U2 | Each layer supports any number of neuron processors (NPs) from 1 to N, e.g. layer 0 with 1 NP for 256 neurons. Non-divisor values must work. Same for parallel inputs vs fan-in. |
| U3 | Optimize for **maximum throughput** (all layers run concurrently). |
| U4 | Default (judged) parallelism: every layer ≤ 98 cycles/image (64-bit input bus is the bottleneck) with minimum XNOR lanes. |
| U5 | `PARALLEL_INPUTS` stays a **single scalar** and is the parallel-input width P_W of **every** non-input layer (clamped to that layer's fan-in), independent of `PARALLEL_NEURONS`. Inter-layer buffers convert widths (producer writes PN[l]-bit chunks, consumer reads P_W-bit words). Few-NP layers therefore do not force narrow downstream inputs. |
| U6 | Configuration messages accepted in **any order**; **reconfiguration without reset** (weights-only / thresholds-only / per layer) between images. |
| U7 | Protocol misuse handled by **safe-ignore**: no images accepted until fully configured; invalid config messages consumed and discarded without corrupting memory; image writes bounded. No new ports. |
| U8 | Verification: new self-checking unit testbenches + xsim scripts in new folders. Provided TB files are not modified except the DUT parameter mapping defaults in `verification/bnn_fcc_tb.sv`. |
| U9 | Synthesis via the installed openflex (`~/envs/openflex`). Allowed openflex edits: source-file lists in `openflex/bnn_fcc_timing.yml` and `openflex/bnn_fcc_verify.yml`, and `PARALLEL_*` defaults in `openflex/rtl/bnn_fcc_timing.sv`. Nothing else. |
| U10 | Clock target: **≥ 100 MHz** for now (pipelining remains parameterized so it can be raised later). |
| U11 | Do not modify existing documentation (`README.md`, `rtl/README.md`, `verification/README.md`, `doc/`, `openflex/README.md`) or testbench logic. |

### 1.2 Facts established from the repository
- Contest device: `XCU250-FIGD2104-2L-E` (openflex timing YAML).
- Binarization: pixel `>= 128` → 1. Hidden neuron: `popcount(XNOR(x,w)) >= threshold`. Output layer: popcounts → argmax; ties resolve to the **lowest index** (reference model uses strict `>`).
- The provided TB asserts `config_last` **only on the final beat of the entire configuration stream**, not per message (README says per message). Every message starts on a fresh bus beat; unused bytes of a message's final beat have TKEEP = 0.
- TB message order: L0 W, L0 T, L1 W, L1 T, …, L(last) W. No threshold message for the output layer. Threshold headers carry `layer_inputs = 32` (to be ignored).
- TB waits for `data_in_ready` after configuration before streaming images, and inserts one idle cycle between images (≈ 99 cycles/image at best on a 64-bit bus for 784 pixels).
- No Questa locally; xsim (Vivado 2025.2) and Verilator 5.020 are available. openflex `sim` mode supports Questa only, so local simulation uses xsim scripts.
- xsim needs `xelab --timescale 1ns/100ps` (the TB has a timescale; the package/interface/DUT do not) and an absolute `BASE_DIR` via `-generic_top`.
- **xsim bug affecting the provided TB:** xsim does not re-create a block-local, uninitialized queue/dynamic array on each loop iteration (minimal repro: `for (...) begin bit q[$]; q.push_back(1); end` grows 1,2,3). In `BNN_FCC_Model::load_single_layer`, `bit bit_q[$]` therefore accumulates, so every neuron in a layer gets neuron 0's first F weights; the TB's MNIST mode fails `verify_model` under xsim ("SV Model says 0, Python says 7"). Initialized locals (`int popcount = 0;`) are fine, and `create_random` / `compute_reference` / `encode_configuration` are unaffected, so **custom-topology mode works correctly in xsim**. The TB file is not edited; MNIST is verified locally by a new test (§7.1 `tb_mnist`), and the user runs the contest MNIST check with Questa (`openflex/verify.sh`).

### 1.3 Design decisions made during brainstorming (confirmed in review)
- Message boundary = header `total_bytes`; after the last payload byte the **remainder of that beat is dropped**. A TLAST arriving before the payload completes **truncates** the message (parser returns to header state). This is compatible with both the TB and the README interpretation.
- Effective parallelism is clamped at elaboration (§3.1), so any positive parameter value is legal for any topology.
- `PARALLEL_INPUTS` keeps its scalar type but its meaning widens from "first hidden layer" to "all non-input layers". `rtl/README.md` permits changing the parallelization parameters; only default values change in the TB mapping and timing wrapper.

---

## 2. Top-level architecture

```
 config AXIS ─► cfg_byte_serializer ─► cfg_parser ─► cfg_layer_writer[l] ─► layer[l] weight/threshold RAMs
                                          │  ▲
                                cfg_busy  │  │ pipeline_idle
                               configured ▼  │
 data_in AXIS ─► input_packer ─► pingpong[0] ─► bnn_layer[0] ─► pingpong[1] ─► bnn_layer[1] ─► … 
                                                         … ─► bnn_layer[L-1] (output) ─► argmax ─► out_fifo ─► data_out AXIS
```

- One `bnn_layer` instance per non-input layer, each with its own NPs and per-NP RAMs. All layers process different images concurrently.
- Stages hand off images via two-bank, width-converting ping-pong buffers (producer writes WR_W-bit chunks, consumer reads RD_W-bit words). A layer starts the next image the cycle after issuing the last pass of the current one, provided its next input bank is full and its next output bank (or output credit) is free → no bubbles in steady state.
- Throughput = max(input-bus time, max layer time). Latency ≈ sum of stage times + pipeline depths.

### 2.1 File list (`rtl/`)
| File | Purpose |
|---|---|
| `bnn_fcc_pkg.sv` | Helper functions (`cdiv`, `imin`, `imax`, `bits(x)=max(1,$clog2(x))`), shared types. |
| `bnn_fcc.sv` | Top level: parameter derivation, instantiation, idle/credit/glue logic. |
| `cfg_byte_serializer.sv` | Config AXIS beat → TKEEP-filtered byte stream. |
| `cfg_parser.sv` | Header collection, validation, routing, load tracking, reconfig interlock. |
| `cfg_layer_writer.sv` | Per-layer payload → weight words (via `bit_packer`) / clamped thresholds, with NP lane/address generation. |
| `bit_packer.sv` | Generic: accepts 0..IN_W bits/cycle, emits OUT_W-bit words; flush pads with PAD. |
| `input_packer.sv` | Image AXIS → binarized, TKEEP-compacted, bounded bits → `bit_packer` → layer-0 ping-pong. |
| `pingpong_buf.sv` | Two-bank, width-converting buffer (WR_W-bit chunk writes, RD_W-bit word reads) with per-bank full flags. |
| `bnn_layer.sv` | Layer engine: sequencer, per-NP RAMs, NP array, output write/emit. |
| `neuron_processor.sv` | XNOR → popcount → accumulate → threshold compare. |
| `popcount.sv` | Parameterized pipelined popcount. |
| `argmax.sv` | Pipelined argmax over the output-layer popcount stream. |
| `axis_out.sv` | Output FIFO + AXIS producer (registered outputs). |

---

## 3. Parameters

### 3.1 Top-level parameters (`bnn_fcc`)
Interface parameters from the provided stub are kept: `INPUT_DATA_WIDTH, INPUT_BUS_WIDTH, CONFIG_BUS_WIDTH, OUTPUT_DATA_WIDTH, OUTPUT_BUS_WIDTH, TOTAL_LAYERS, TOPOLOGY[TOTAL_LAYERS], PARALLEL_NEURONS[TOTAL_LAYERS-1]`. `PARALLEL_INPUTS` (scalar `int`) is the P_W of every non-input layer. The DUT keeps its existing generic defaults (`PARALLEL_INPUTS = 8`, `PARALLEL_NEURONS = '{default: 8}`).

Added implementation parameters (with defaults, all optional):
| Parameter | Default | Meaning |
|---|---|---|
| `POPCOUNT_LEVELS_PER_STAGE` | 3 | Adder-tree levels between pipeline registers in `popcount` (lower = higher fmax, more latency). |
| `ARGMAX_LEVELS_PER_STAGE` | 2 | Compare levels between registers in argmax group reduction. |
| `OUT_FIFO_DEPTH` | 4 | Output FIFO depth (credit pool). |
| `RAM_STYLE` | `"auto"` | Vivado `ram_style` for weight RAMs (`auto`, `distributed`, `block`). |

Derived localparams (L = `TOTAL_LAYERS-1` non-input layers; layer `l` has fan-in `F[l] = TOPOLOGY[l]`, neurons `N[l] = TOPOLOGY[l+1]`):
- `PN[l]  = min(PARALLEL_NEURONS[l], N[l])` (must be ≥ 1)
- `PW[l]  = min(PARALLEL_INPUTS, F[l])` (must be ≥ 1)
- `GROUPS[l] = ceil(N[l]/PN[l])`, `PASSES[l] = ceil(F[l]/PW[l])`
- `ACC_W[l] = bits(F[l]+1)`, `THR_W[l] = bits(F[l]+2)`
- Cycles per image in layer l = `GROUPS[l] * PASSES[l]`.

Elaboration-time assertions: all `PARALLEL_*` ≥ 1; `TOTAL_LAYERS ≥ 2`; all `TOPOLOGY[i] ≥ 1`; `INPUT_DATA_WIDTH % 8 == 0`; `INPUT_BUS_WIDTH % INPUT_DATA_WIDTH == 0`; `CONFIG_BUS_WIDTH % 8 == 0`; `bits(N[L-1]) ≤ OUTPUT_DATA_WIDTH`; `OUTPUT_DATA_WIDTH ≤ OUTPUT_BUS_WIDTH`; `F[l] < 2^16`, `N[l] < 2^16` (header field widths).

### 3.2 Default judged configuration (SFC)
`PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 11, 1}`:

| Layer | F | N | PW | PN | PASSES | GROUPS | Cycles/image | XNOR lanes | Weight RAM per NP |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 784 | 256 | 66 | 32 | 12 | 8 | 96 | 2112 | 96 × 66 |
| 1 | 256 | 256 | 66 | 11 | 4 | 24 | 96 | 726 | 96 × 66 |
| 2 | 256 | 10 | 66 | 1 | 4 | 10 | 40 | 66 | 40 × 66 |

Total 2904 XNOR lanes, 44 NPs. Selection (exhaustive sweep over the single P_W, each layer ≤ 97 cycles/image, minimum PN per layer): the absolute minimum is PI=33, PN={64,22,1} at 2871 lanes but 87 NPs; PI=66 costs 1.1% more lanes for half the NPs (each NP carries an accumulator, threshold RAM and compare). Applied in: `verification/bnn_fcc_tb.sv` defaults (parameter mapping only) and `openflex/rtl/bnn_fcc_timing.sv` defaults.

---

## 4. Configuration manager

### 4.1 `cfg_byte_serializer #(BUS_W)`
- Ports: `s_valid/s_ready/s_data/s_keep/s_last` (AXIS slave); `m_valid/m_ready`, `m_byte[7:0]`, `m_null`, `m_end_of_beat`, `m_beat_last`; input `drop_rest`.
- Holds one beat. `s_ready` = holder empty, or holder will be empty after this cycle's handshake (registered state only; no combinational path from `s_valid`).
- Emits the beat's bytes with TKEEP = 1, lowest byte lane first, one per cycle. Null bytes may be anywhere.
- `m_end_of_beat` marks the beat's last kept byte; `m_beat_last` = `s_last` of that beat, presented on its last kept byte.
- A beat with TKEEP = 0 and TLAST = 1 emits a single item with `m_null = 1, m_beat_last = 1` (carries TLAST, no data). A beat with TKEEP = 0 and TLAST = 0 is consumed silently.
- `drop_rest` asserted with a byte handshake discards the remaining bytes of the current beat (no further items from it, including any TLAST it carried — the parser has already completed that message).
- Throughput: 1 byte/cycle (config speed is not judged).

### 4.2 `cfg_parser`
States: `HEADER → (VALIDATE) → WAIT_IDLE → PAYLOAD → HEADER`, or `HEADER → DISCARD → HEADER`.

- **HEADER:** shift in 16 bytes little-endian: `msg_type[7:0]`, `layer_id[15:8]`, `layer_inputs[31:16]`, `num_neurons[47:32]`, `bytes_per_neuron[63:48]`, `total_bytes[95:64]`, `reserved[127:96]`. A TLAST (incl. `m_null`) received before 16 bytes resets the header collection (partial header discarded).
- **VALIDATE** (combinational on the completed header, against elaboration-time tables):
  - Weights (`msg_type == 0`): `layer_id < L`, `layer_inputs == F[l]`, `num_neurons == N[l]`, `bytes_per_neuron == ceil(F[l]/8)`, `total_bytes == N[l]*ceil(F[l]/8)`.
  - Thresholds (`msg_type == 1`): `layer_id < L-1` (hidden layers only), `num_neurons == N[l]`, `bytes_per_neuron == 4`, `total_bytes == 4*N[l]`. `layer_inputs` and `reserved` ignored.
  - Anything else (unknown type, output-layer thresholds, mismatched fields) → **DISCARD**.
- **WAIT_IDLE:** `cfg_busy = 1`; parser accepts no bytes until `pipeline_idle` (§6.3). `pipeline_idle` is only honored from the second cycle of WAIT_IDLE onward, so an image that started in the same cycle `cfg_busy` was registered is seen as `mid_image` and drains first. Then pulse `msg_start(layer, type)` (clears that layer's `w_loaded`/`t_loaded` flag and resets the layer writer's counters) and enter PAYLOAD.
- **PAYLOAD:** forward bytes to `cfg_layer_writer[layer_id]` (valid/ready). Count `total_bytes`. On the last byte: assert `drop_rest`, set the `w_loaded`/`t_loaded` flag, clear `cfg_busy`, go to HEADER. If TLAST arrives before the count completes: message truncated — flag stays cleared, writer reset, `cfg_busy` cleared, go to HEADER.
- **DISCARD:** consume `total_bytes` bytes without writing (TLAST also ends discard early); on the last byte assert `drop_rest`. Does not touch load flags or `cfg_busy`.
- `configured = &w_loaded[0..L-1] & &t_loaded[0..L-2]`.
- Reset clears all flags and returns to HEADER.

### 4.3 `cfg_layer_writer #(F, N, PW, PN, THR_W, HAS_THR)`
One per layer. Inputs: `start`, `type`, byte stream (valid/ready). Outputs (to `bnn_layer`): `w_we, w_lane, w_addr, w_data[PW]`, `t_we, t_lane, t_addr, t_data[THR_W]`.

- **Weights:** counters `byte_in_neuron` (0..BPN-1, BPN = ceil(F/8)), `lane` (0..PN-1), `group`, `pass`. Each byte supplies `nbits = min(8, F - 8*byte_in_neuron)` valid bits (LSB-first) to `bit_packer #(IN_W=8, OUT_W=PW, PAD=1)`; the neuron's last byte also asserts `flush`. Each emitted word is written to NP `lane` at address `group*PASSES + pass` (address maintained incrementally, no multipliers in the datapath). After the neuron's last word: `pass ← 0`, advance `lane`; on `lane == PN-1` wrap and advance `group`. Stream padding bits (beyond F) are dropped; words are padded with 1s to a multiple of PW.
- **Thresholds:** gather 4 bytes little-endian into signed 32-bit `t`; store `clamp(t, 0, F+1)` as THR_W bits to NP `lane`, address `group`. (Negative → always fires; > F → never fires; preserves `popcount >= threshold` exactly.)
- `ready` deasserts while the packer cannot accept (only when PW < 8).

### 4.4 `bit_packer #(IN_W, OUT_W, PAD)`
- Ports: `in_valid, in_ready, in_bits[IN_W], in_count (0..IN_W), in_flush`; `out_valid, out_word[OUT_W]` (consumer always accepts); `clear`.
- State: bit buffer (width OUT_W+IN_W) and count. Each cycle: if count ≥ OUT_W, emit the low OUT_W bits and shift. Input is accepted when the post-emit count < OUT_W and no flush is pending; accepted bits are appended at the post-emit count. `in_flush` with an accepted input sets `flush_pending`; when pending and 0 < count < OUT_W, emit the remaining bits padded with PAD to OUT_W and clear; when count == 0, just clear `flush_pending`.
- For OUT_W ≥ IN_W, `in_ready` stays high except during a flush cycle → 1 input/cycle.
- `in_ready` depends on registered state only.

---

## 5. Compute

### 5.1 `bnn_layer #(F, N, PW, PN, IS_OUTPUT, ...)`
Ports: config write ports (§4.3); input ping-pong read side (`in_full[1:0]`, `in_rd_bank`, `in_rd_addr`, `in_rd_data[PW]`, `in_release`, `in_release_bank`); hidden output: `out_full[1:0]` of next ping-pong, `out_we, out_bank, out_addr, out_data[PN], out_commit, out_commit_bank`; output-layer result: `res_valid, res_counts[PN][ACC_W], res_mask[PN], res_last`, plus `start_ok` (credit) input; `busy` output.

- **Sequencer:** owns `rd_bank` and `wr_bank` pointers. Starts an image when `in_full[rd_bank]` and (hidden: `!out_full[wr_bank]`; output layer: `start_ok`). Issues `(g, p)` every cycle: `p` 0..PASSES-1 inner, `g` 0..GROUPS-1 outer. After issuing the last `(g,p)`, it releases `rd_bank` (after the read is captured), toggles both pointers, and may start the next image on the very next cycle. Side-band (`bank`, `first_pass`, `last_pass`, `g`, `last_group`) is delayed alongside the data path.
- **Per-NP memories:** weight RAM (simple dual-port, `GROUPS*PASSES` × PW, `(* ram_style = RAM_STYLE *)`, 1-cycle registered read, write port from config) and threshold RAM (`GROUPS` × THR_W). Write enables decoded per lane.
- **Read addressing:** weight address = `g*PASSES + p` maintained incrementally; input address = `p`; threshold address = `g`.
- **Lane validity:** `valid_lane[k] = (g*PN + k < N)` (only the last group can be partial; computed from a registered "last group" flag and a constant mask).
- **Hidden output:** on `last_pass`, `out_data[k] = valid_lane[k] & (acc_k >= thr_k)`; write chunk `g` (PN bits) of `out_bank`; after the last group's write, `out_commit(out_bank)`. Invalid lanes are forced to 0. The next ping-pong buffer converts PN-bit chunks to the next layer's PW-bit words (§5.5).
- **Output layer:** on `last_pass`, emit `res_counts = acc`, `res_mask = valid_lane`, `res_last = last_group`.
- `busy` = an image is being issued or is in the data path.

### 5.2 `neuron_processor #(PW, ACC_W, THR_W, POPCOUNT_LEVELS_PER_STAGE)`
`xnor = ~(x ^ w)` → `popcount` (latency `PC_LAT` derived from PW) → accumulator (`first_pass ? pc : acc + pc`) → on `last_pass`: `y = (acc_final >= thr)`; also outputs `acc_final`. The threshold is read from its RAM in time to meet `last_pass`. Datapath registers are not reset.

### 5.3 `popcount #(W, LEVELS_PER_STAGE)`
Level 0: count each 6-bit chunk (LUT6-friendly) → binary adder tree over chunk counts; a register every `LEVELS_PER_STAGE` levels and at the output. Exposes `localparam LATENCY` (function of W and LEVELS_PER_STAGE) used by callers to align side-band signals.

### 5.4 Padding invariant
Inputs beyond F are always 0 (ping-pong bits at index ≥ NBITS read as constant 0; image packer pads 0 and drops extra pixels; hidden layers force invalid lanes to 0). Weights beyond F are always 1 (writer pads 1). Either alone makes padded bits contribute 0 to the popcount; both are guaranteed.

### 5.5 `pingpong_buf #(NBITS, WR_W, RD_W)`
- Two banks, each a register vector of NBITS bits (bits at index ≥ NBITS are not stored and read as 0). Registers, not RAM: SFC needs 2×784 + 2×256 + 2×256 FFs.
- **Write side:** `wr_en, wr_bank, wr_idx, wr_data[WR_W]` writes chunk `wr_idx`, i.e. bits `[wr_idx*WR_W +: WR_W]`. Because chunk boundaries are multiples of WR_W, every stored bit `j` has a constant source lane `j % WR_W` and a constant chunk `j / WR_W`; the write is a per-bit enable decoded from `wr_idx` — no shifter. `commit(bank)` sets `full[bank]`.
- **Read side:** `rd_bank, rd_idx` → `rd_data[RD_W]` (registered, 1-cycle latency) = bits `[rd_idx*RD_W +: RD_W]`; each output bit is a mux over `ceil(NBITS/RD_W)` constant positions. `release(bank)` clears `full[bank]`.
- `full[1:0]` exported; producer and consumer each own their bank pointer.
- Instances: layer-0 input (`NBITS=F[0], WR_W=RD_W=PW[0]`), between layer l and l+1 (`NBITS=N[l], WR_W=PN[l], RD_W=PW[l+1]`; WR_W = RD_W only when PN[l] equals the clamped P_W).

---

## 6. Input, output, glue

### 6.1 `input_packer`
- K = `INPUT_BUS_WIDTH / INPUT_DATA_WIDTH` pixels per beat; pixel j valid iff all its TKEEP bits are 1.
- Binarize: `bit = pixel[INPUT_DATA_WIDTH-1]` (MSB; exactly `>= 128` for 8-bit).
- Compact valid bits to the LSBs in pixel order (prefix count over K lanes) → `(bits, count)`.
- Bound: a pixel counter clips `count` so at most F[0] bits per image are packed; extras dropped.
- `bit_packer #(IN_W=K, OUT_W=PW[0], PAD=0)`; each emitted word written to the layer-0 ping-pong bank as chunk `wr_idx++`. TLAST → flush, then commit the bank and toggle the bank pointer. An image ending early commits what it has (classification is undefined; no hang).
- `data_in_ready` (registered-state only): at an image boundary requires `configured & !cfg_busy & !full[wr_bank] & packer_ready`; within an image only `packer_ready`. Mid-image the image always completes even if a config header arrives.
- `mid_image` output for idle detection.

### 6.2 `argmax` and `axis_out`
- **Group reduction:** pipelined compare tree over PN lanes (ignoring `res_mask == 0` lanes); ties → lower lane.
- **Running best across groups:** a later group replaces the best only if strictly greater; first group initializes. Global index = `group_base + lane`, `group_base += PN` per group (no multiplier).
- On `res_last`: push the class index into the output FIFO.
- **Credits:** the output layer may start an image only if `fifo_count + images_in_flight < OUT_FIFO_DEPTH`; results are never dropped and `data_out_ready` backpressure propagates upstream via ping-pong full flags to `data_in_ready`.
- **AXIS out:** `data_out_valid` = FIFO non-empty (held until handshake); `data_out_data` = class zero-extended to OUTPUT_BUS_WIDTH; `data_out_keep` = low `ceil(OUTPUT_DATA_WIDTH/8)` bytes set (default `8'h01`); `data_out_last = 1`. Outputs driven from registers.

### 6.3 Idle detection and reconfiguration
`pipeline_idle = !input_packer.mid_image & (all ping-pong full flags == 0) & (all bnn_layer.busy == 0) & !argmax.busy`. The output FIFO is excluded (results already computed do not depend on memories). While `cfg_busy`, no new image starts; in-flight images drain; the parser then writes.

### 6.4 Reset
Synchronous, active-high `rst` clears all control state: FSMs, counters, pointers, full flags, FIFO pointers, load flags, `cfg_busy`. RAM contents and pure datapath registers are not reset. After reset `configured = 0` ⇒ `data_in_ready = 0`.

### 6.5 Expected SFC performance (default config)
- Throughput: input-bus bound ≈ 99 cycles/image with the provided TB (98 beats + 1 idle cycle).
- Latency (first accepted beat → output handshake): ≈ 98 + (96+~6) + (96+~6) + (40+~6) + ~5 ≈ 355 cycles at the defaults; exact value reported from simulation.
- Config load: ~35.7k bytes → ~36k cycles.

---

## 7. Verification

### 7.1 Unit testbenches (`verification/unit/`, new)
All self-checking, randomized (seeded), xsim; each prints `SUCCESS` or `$error`/`$fatal`.

| TB | Checks |
|---|---|
| `tb_popcount` | Random vectors vs `$countones`; W ∈ {1,5,6,7,8,16,43,49,64,131}; several LEVELS_PER_STAGE; latency matches `LATENCY`. |
| `tb_bit_packer` | Random counts 0..IN_W, both PAD values, flush at/not-at word boundary, OUT_W ∈ {1,3,5,8,16,49}, IN_W ∈ {8}; queue model. |
| `tb_cfg_byte_serializer` | Random TKEEP (incl. sparse, all-zero, all-zero+TLAST), valid gaps, `m_ready` backpressure, `drop_rest`. |
| `tb_pingpong_buf` | Random NBITS/WR_W/RD_W incl. WR_W ≠ RD_W, non-divisors, WR_W or RD_W > NBITS; bits ≥ NBITS read 0; bank full/release sequencing. |
| `tb_neuron_processor` | Random weights/inputs/thresholds over 1..N passes; thresholds 0, F, F+1. |
| `tb_bnn_layer` | TB imitates the config writer's layout; random F, N, PW, PN incl. non-divisors, PN=1, PN=N, PW>F, GROUPS=PASSES=1; input-bank gaps, slow output release; checks every output bit / popcount per image. |
| `tb_argmax` | Ties within and across groups, partial groups, PN=1 and PN=N. |
| `tb_mnist` | New top-level test: `class BNN_FCC_Model_XsimSafe extends bnn_fcc_tb_pkg::BNN_FCC_Model` overriding `load_from_file`/`load_single_layer` with an xsim-safe loader (explicit `bit_q.delete()` per line). Loads `python/model_data`, asserts the SV reference matches `python/test_vectors/expected_outputs.txt` for all 100 images, configures `bnn_fcc` (default SFC parallelism) with `encode_configuration`, streams all 100 MNIST images with 0.8 valid probability and random `data_out_ready`, and checks every DUT output against `expected_outputs.txt`. Reports latency/throughput like the provided TB. |
| `tb_cfg_sequences` | Top-level `bnn_fcc` directed tests reusing `bnn_fcc_tb_pkg::BNN_FCC_Model` (imported, unmodified) to build messages: reordered messages; weights-only and thresholds-only reconfiguration between images; invalid headers (discarded, model unchanged); truncated message (TLAST early) blocks images until reloaded; reset mid-config and mid-image; per-message TLAST variant. |

### 7.2 System regression with the provided `bnn_fcc_tb`
Array parameters cannot be overridden from the command line, so each configuration is a small **new** wrapper module in `sim/cfg/` that instantiates `bnn_fcc_tb` with parameter overrides (TB file unchanged apart from the default mapping in §3.2). Because of the xsim loader bug (§1.2), every "SFC" row below runs the provided TB in **custom mode** with `CUSTOM_LAYERS = 4`, `CUSTOM_TOPOLOGY = '{784,256,256,10}` (random model and images, full SFC shape); the real trained MNIST model is covered by `tb_mnist`.

| Config | Topology | PI / PN | Settings |
|---|---|---|---|
| `sfc_stress` | SFC shape (custom mode) | 66 / {32,11,1} | TOGGLE=1, 0.8/0.8, 50 images (contest stress settings) |
| `sfc_perf` | SFC shape (custom mode) | 66 / {32,11,1} | TOGGLE=0, 1.0/1.0, 100 images (records latency/throughput) |
| `sfc_pn1` | SFC | 66 / {1,1,1} | stress settings, 50 images (3072 cycles/image), TIMEOUT raised |
| `sfc_alt` | SFC | 33 / {64,22,1} | stress settings |
| `sfc_orig` | SFC | 8 / {8,8,10} (original TB values; WR_W = RD_W) | stress settings, 20 images, TIMEOUT raised |
| `c_8888` | 8→8→8→8 | 66 / {32,11,1} (clamped; 1-cycle/image layers) | stress settings |
| `c_13753` | 13→7→5→3 | 3 / {2,4,3} | stress settings |
| `c_100` | 100→37→19→10 | 7 / {5,3,4} | stress settings |
| `c_tiny` | 5→1→3→2 | 1 / {1,1,1} | stress settings |

### 7.3 Scripts (`sim/`, new)
- `run_unit.sh` — compile and run all unit TBs with `xvlog/xelab/xsim`.
- `run_regress.sh [cfg...]` — run §7.2 wrappers (`xelab --timescale 1ns/100ps`, absolute `BASE_DIR`); pass iff the log contains `SUCCESS:` and no `Error:`/`Fatal:`/`Failure:` (same rule as `openflex/verify.sh`).
- `lint.sh` — `verilator --lint-only -Wall` over `rtl/`.
- Vivado environment sourced from `/tools/AMD/2025.2/Vivado/settings64.sh`.
- If xsim cannot handle a construct in the provided TB, it is reported to the user, not edited.

---

## 8. Synthesis
- Add all `rtl/*.sv` files (package first) to the file lists of `openflex/bnn_fcc_timing.yml` and `openflex/bnn_fcc_verify.yml`.
- Set `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32,11,1}` in `openflex/rtl/bnn_fcc_timing.sv` (default values only).
- Run `openflex bnn_fcc_timing.yml -c bnn_fcc.csv` (openflex venv + Vivado 2025.2) on `XCU250-FIGD2104-2L-E`; report fmax and LUT/LUTRAM/FF/BRAM/DSP utilization.
- `bnn_fcc_verify.yml` uses Questa (not available locally); the user runs `verify.sh` where Questa exists.

---

## 9. Definition of done
1. All unit testbenches pass in xsim.
2. All §7.2 regression configurations pass, including `sfc_stress`, and `tb_mnist` classifies all 100 MNIST images exactly as `expected_outputs.txt` under stress settings.
3. Verilator lint clean (or each waiver justified in a comment).
4. Vivado/openflex reports fmax ≥ 100 MHz on XCU250-2L.
5. Results reported to the user: avg latency (cycles), cycles/image, fmax, resource utilization.

## 10. Out of scope
- Widening `INPUT_BUS_WIDTH` for performance beyond the bus limit.
- Overlapping layer-0 compute with image arrival (latency optimization).
- Writing the contest `report.pdf`.
- Any change to existing docs, TB logic, or openflex beyond U9.
