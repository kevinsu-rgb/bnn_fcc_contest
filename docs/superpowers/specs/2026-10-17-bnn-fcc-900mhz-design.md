# BNN_FCC 900 MHz Design Specification

Date: 2026-10-17
Branch: `bnn-fcc-900mhz`
Status: approved design, pending written-spec review

## 1. Goal

Implement the `bnn_fcc` AXI4-Stream binary neural network classifier (rtl/bnn_fcc.sv)
for the MNIST SFC topology (784 → 256 → 256 → 10), optimized for **maximum throughput**
at **900 MHz** on the contest part `xcu250-figd2104-2L-e`, while supporting every
non-parallelism parameter of the interface (any topology, layer count, and bus width).

### 1.1 Success criteria

| # | Criterion | How it is checked |
|---|---|---|
| S1 | Provided `bnn_fcc_tb` passes in contest stress mode (`USE_CUSTOM_TOPOLOGY=0`, `TOGGLE_DATA_OUT_READY=1`, `CONFIG_VALID_PROBABILITY=0.8`, `DATA_IN_VALID_PROBABILITY=0.8`) and with custom topologies | Run by the user (Questa is not installed locally) |
| S2 | All UVM 1.2 unit/integration regressions pass on Vivado xsim, with functional coverage reported | `sim/regress.sh` |
| S3 | Vivado sign-off at 1.111 ns with default (LUTRAM) memories: setup WNS ≥ 0, hold WHS ≥ 0, pulse-width WPWS ≥ 0 | `synth/` sign-off script |
| S4 | openflex timing flow (`openflex bnn_fcc_timing.yml`, default Vivado directives, no custom XDC) reports fMax ≥ 900 MHz | openflex CSV |
| S5 | Steady-state throughput of one image per 98 cycles on the 64-bit input bus (TB reports ≈99 because it idles one cycle between images) | TB stats with `TOGGLE_DATA_OUT_READY=0`, valid probabilities 1.0 |
| S6 | Switching `RAM_STYLE` to `"block"` or `"ultra"` compiles and passes simulation unchanged (900 MHz closure not required for those styles) | Unit + integration regressions with alternate `RAM_STYLE` |
| S7 | `verilator --lint-only -Wall` clean on `rtl/` | lint script |

## 2. Constraints and decisions

### 2.1 Stated by the user

- Target 900 MHz. Memories default to LUTRAM; BRAM and URAM must be easy to swap in.
- Use case: maximum throughput. Close 900 MHz at `INPUT_BUS_WIDTH = 64`; the RTL must scale
  structurally with wider buses, which will be explored later.
- Architecture A (uniform buffered layers) with defaults `PARALLEL_INPUTS = 16`,
  `PARALLEL_NEURONS = '{128, 8, 5}`, `TOPOLOGY = '{784, 256, 256, 10}`.
- Per-layer input parallelism follows the slide convention: PW₀ = `PARALLEL_INPUTS`,
  PW_l = `PARALLEL_NEURONS[l-1]`. No additional parallelism parameter.
- `PARALLEL_INPUTS` may be any value ≥ 1 (bit-level gearbox and config packing).
- Configuration messages may arrive in any order and any subset; live reconfiguration
  without reset is supported (images drain first).
- After reset, `data_in_ready` stays low until every layer is fully configured.
- Unit testbenches are UVM 1.2 on Vivado xsim. The user runs the provided full testbench.
- Never modify provided docs or testbenches, except the DUT parameter mapping in
  `verification/bnn_fcc_tb.sv`, the file lists in `openflex/*.yml`, and the `PARALLEL_*`
  defaults in `openflex/rtl/bnn_fcc_timing.sv`.
- Work is committed on branch `bnn-fcc-900mhz`.

### 2.2 Facts established from the repository and tools

- Configuration framing: the TB asserts `config_last` only on the final beat of the whole
  configuration and pads each message to a beat boundary with `keep = 0`. Messages are
  therefore delimited by header byte counts, not TLAST.
- The TB sends no thresholds for the output layer.
- Reference math: `popcount = #(input == weight)`; hidden output = `popcount >= threshold`
  (signed int compare); output layer = argmax over popcounts with strict `>` (lowest index
  wins ties). Binarization: `pixel >= 128`.
- The TB compares the full `data_out_data` byte against the class index; upper bits must be 0.
- Trained thresholds: L0 297–491, L1 113–147. Random TB thresholds lie in [0, fan_in].
- openflex fMax = `1000 / (period − setup WNS)`; it ignores pulse-width checks. Its Vivado
  flow uses default directives and only an auto-generated XDC (clock + `HD.CLK_SRC`).
- Measured minimum periods on `xcu250-figd2104-2L-e` (throwaway probe, routed at 1.111 ns):

  | Primitive | Min period | Usable at 1.111 ns |
  |---|---|---|
  | RAMB36E2 | 1.569 ns | no |
  | URAM288 | 1.666 ns | no |
  | LUTRAM (RAMS64E) | 1.064 ns | yes (47 ps margin) |
  | DSP48E2 output | 0.650 ns | yes |
  | FDRE | 0.550 ns | yes |

- Tools available: Vivado 2025.2 (`/tools/AMD/2025.2/Vivado`, includes xsim + UVM 1.2),
  openflex 0.1.4 (`~/envs/openflex`), Verilator 5.020. Questa is not installed.

## 3. Terminology and parameters

**Layer numbering.** L0 is the first hidden layer, matching the config header `layer_id`,
the TB model (`weight[0]`), and `PARALLEL_NEURONS[0]`. `TOPOLOGY[0]` is the input count.
The design has `NL = TOTAL_LAYERS − 1` computing layers; L(NL−1) is the output layer.

For layer l:

| Symbol | Definition |
|---|---|
| N_l | fan-in = `TOPOLOGY[l]` |
| M_l | neurons = `TOPOLOGY[l+1]` |
| PN_l | effective neuron processors = `min(PARALLEL_NEURONS[l], M_l)` |
| PW_0 | `min(PARALLEL_INPUTS, N_0)` |
| PW_l (l ≥ 1) | PN_{l−1} |
| C_l | chunks per neuron = ⌈N_l / PW_l⌉ |
| G_l | neuron groups = ⌈M_l / PN_l⌉ |
| T_l | cycles per image = C_l · G_l |
| AW_l | popcount/accumulator width = `$clog2(N_l + 1)` |
| TW_l | threshold width = `$clog2(N_l + 2)` |

Neuron processor (NP) p in group g computes neuron n = g · PN_l + p.

All derived values come from constant functions in `bnn_fcc_pkg.sv`.

### 3.1 Default sizing (64-bit bus)

| Layer | N | M | PW | PN | C × G | T | XNOR |
|---|---|---|---|---|---|---|---|
| L0 | 784 | 256 | 16 | 128 | 49 × 2 | 98 | 2,048 |
| L1 | 256 | 256 | 128 | 8 | 2 × 32 | 64 | 1,024 |
| L2 | 256 | 10 | 8 | 5 | 32 × 2 | 64 | 40 |

Throughput is set by L0 (98 cycles, equal to the bus limit). Estimated latency
≈ 98 (image load) + 98 + 64 + 64 + ~35 (pipelines) ≈ 360 cycles (≈ 0.4 µs at 900 MHz).

### 3.2 New top-level parameters

Added to `bnn_fcc` with defaults, so no TB change is needed for them:

| Parameter | Default | Purpose |
|---|---|---|
| `RAM_STYLE` | `"distributed"` | Weight/threshold/buffer memory style: `"distributed"`, `"block"`, `"ultra"` |
| `RAM_READ_LATENCY` | 1 | Registered read latency; ≥ 2 recommended for block/ultra |
| `OUT_FIFO_DEPTH` | 4 | Output FIFO depth / credit count |

## 4. Architecture overview

```
config ─▶ axis_skid ─▶ bnn_cfg_mgr ── write bus {layer, kind, np, addr, data} ──┬─────┬─────┐
                          │ cfg_req/cfg_grant, loaded flags                      ▼     ▼     ▼
data_in ─▶ axis_skid ─▶ bnn_input_gate ─▶ bnn_gearbox ─▶ Buf0 ─▶ L0 ─▶ Buf1 ─▶ L1 ─▶ Buf2 ─▶ L2 ─▶ bnn_argmax ─▶ axis_fifo ─▶ data_out
                      (binarize, keep,     (pixels →     A/B          A/B          A/B   (popcounts)
                       compaction)          PW₀ words)
```

Layers and buffers are generated for `NL` layers, so any depth works.

### 4.1 Modules (`rtl/`)

| File | Responsibility |
|---|---|
| `bnn_fcc_pkg.sv` | Constant functions for per-layer derived values (§3) |
| `bnn_fcc.sv` | Top level: instantiates and connects everything, readiness gating |
| `axis_skid.sv` | AXI-Stream register slice (registered `ready`) |
| `axis_fifo.sv` | Output FIFO with registered outputs and credit reservation |
| `bnn_input_gate.sv` | Binarize, apply TKEEP, compact kept pixels, image-boundary gating |
| `bnn_gearbox.sv` | Pack compacted bits into PW₀-bit words; zero-pad at TLAST |
| `bnn_act_buf.sv` | Double-buffered activation buffer with slot handshake |
| `bnn_layer.sv` | Sequencer, broadcast tree, NP array, memories, output collection |
| `bnn_np.sv` | XNOR, popcount, accumulator, threshold compare |
| `bnn_popcount.sv` | Pipelined popcount tree, any width |
| `bnn_ram.sv` | Memory wrapper (`RAM_STYLE`, `READ_LATENCY`, SP/SDP) |
| `bnn_cfg_mgr.sv` | Byte extraction, header FSM, weight packer, threshold assembler, write bus, loaded flags, drain handshake |
| `bnn_argmax.sv` | Pipelined argmax over groups, strict-greater tie rule |

## 5. Flow control, reconfiguration, reset

### 5.1 Slot handshake

Every buffer has two slots (A/B) with `full[2]`, a write pointer, and a read pointer.
The producer writes into the free slot and pulses `commit`; the consumer reads the full
slot and pulses `release`. A layer starts an image only when its input slot is full and its
output slot is free, then issues C·G tokens without stopping. The output layer starts only
after reserving an output-FIFO credit (occupancy + reserved ≤ `OUT_FIFO_DEPTH`).

Consequences: there is no global stall or clock-enable net; `data_out_ready` backpressure
propagates upstream one layer at a time, at image boundaries only.

### 5.2 `data_in_ready`

Asserted when all hold:
1. every layer's weights are loaded and every hidden layer's thresholds are loaded;
2. no reconfiguration is pending, or the input is mid-image (an image already started is completed);
3. the gearbox can accept a beat;
4. a Buf0 slot is free (or the current one is still filling).

### 5.3 Live reconfiguration

1. `bnn_cfg_mgr` parses a header and raises `cfg_req`.
2. The input gate stops accepting new images after the current image's TLAST.
3. When every layer is idle and every buffer is empty, `cfg_grant` asserts.
   Results already in the output FIFO remain and drain normally.
4. The payload is written while `config_ready` paces the stream.
5. The message's loaded flag is set, `cfg_req` drops, and images resume.

### 5.4 Reset

Synchronous, active-high `rst`. Internally registered and distributed through a replicated
pipeline tree. Clears control state only (FSMs, counters, slot flags, loaded flags, FIFO
pointers). Memory contents survive reset but are unusable until reloaded (loaded flags).

## 6. Configuration manager (`bnn_cfg_mgr`)

### 6.1 Byte extraction

The config skid feeds a beat register. The manager walks one byte position per cycle
(`CONFIG_BUS_WIDTH/8` cycles per beat), dropping `keep = 0` bytes. `config_ready` asserts
when the beat register can accept the next beat. TLAST is ignored for framing; the byte
after a payload is the next header byte, so messages may start mid-beat. Supports any
`CONFIG_BUS_WIDTH` that is a multiple of 8.

### 6.2 Header FSM

Collects 16 bytes (little-endian) and latches `msg_type`, `layer_id`, `num_neurons`,
`bytes_per_neuron`, `total_bytes` (`layer_inputs` and `reserved` are not used). It then
raises `cfg_req` and waits for `cfg_grant` before consuming any payload byte.

The payload is dropped (counted out, not written) when `layer_id ≥ NL`, `msg_type ∉ {0, 1}`,
or the message carries thresholds for the output layer. Neuron and byte counters follow the
header fields; writes beyond the memory dimensions implied by `TOPOLOGY` are suppressed.

### 6.3 Weight packer (any PW)

Per neuron, payload bytes form a bit stream: weight i is bit `i mod 8` of byte `⌊i/8⌋`.
The packer appends 8 bits per cycle into a PW-bit word at a variable fill offset (alignment
mux split over two pipeline stages). A word is emitted when full. At the end of a neuron
(after `bytes_per_neuron` bytes) the last word is padded with 1s up to C·PW bits, and bits
beyond C·PW (excess byte padding, e.g. N = 13, PW = 13) are dropped. A `generate` fast path
for PW a multiple of 8 removes the variable shifter.

### 6.4 Threshold assembler

Four bytes per neuron form a signed 32-bit value, clamped to `[0, N_l + 1]` and stored in
TW_l bits. This preserves `popcount >= threshold` exactly (negative ⇒ always 1,
> N ⇒ always 0).

### 6.5 Address mapping

| Item | Value |
|---|---|
| neuron n → NP | p = n mod PN_l |
| group | g = n / PN_l |
| weight word address | g · C_l + c, chunk c |
| threshold address | g |

### 6.6 Write bus and loaded flags

A registered broadcast `{layer, kind, np, addr, data}` travels through a pipelined fan-out
tree to all layers. Each layer one-hot decodes `np` in a register stage to drive its RAM
write enables. Writes only happen while the pipeline is drained, so weight and threshold RAMs
are single-port with the address mux placed before the address register.

`w_loaded[l]` / `t_loaded[l]` set when a message's final payload byte has been written
(`t_loaded` is constant 1 for the output layer). All clear on reset.

## 7. Layer datapath (`bnn_layer`, `bnn_np`, `bnn_popcount`)

Parameters: N, M, PW, PN, `IS_OUTPUT`, `RAM_STYLE`, `READ_LATENCY`.

### 7.1 Sequencer

When the input slot is full and the output slot is free, the sequencer emits one token per
cycle: `(c, g, first = (c == 0), last = (c == C−1))` for C·G cycles. The next image's tokens
follow without a bubble. Each token reads the input buffer at chunk c and every weight RAM at
`g·C + c`; the faster read path is delayed to align with the slower one (delays derived from
`READ_LATENCY`). After the last group's results are written, the output slot is committed and
the input slot released.

### 7.2 Broadcast tree

Chunk data, address, and control travel through a balanced registered fan-out tree
(fanout ≈ 16 per stage, ⌈log₁₆ PN⌉ stages). Every NP receives a given token in the same cycle,
so NP outputs are aligned. Replicated registers are protected from merging (§9).

### 7.3 Neuron processor (`bnn_np`)

1. XNOR of PW input bits with PW weight bits.
2. `bnn_popcount`: leaves are 3-pair XNOR-popcounts (6 inputs, one LUT6 per output bit),
   followed by a registered adder tree with about one LUT level plus a short carry per stage.
   Stage count is a function of PW (≈3 for PW = 16, ≈6 for PW = 128).
3. Accumulator (AW bits): `acc = first ? pc : acc + pc`.
4. On `last`: hidden layer → registered compare `acc >= T[g]` (threshold read timed to this
   stage); output layer → emit `acc`.

### 7.4 Padding

- Inputs beyond N within the last chunk are 0 (gearbox zero-pad, or zero outputs of padded
  NPs in the previous layer); their weights are 1 (config packer). XNOR(0, 1) = 0, so the
  popcount is unaffected.
- NPs with `g·PN + p ≥ M` in the last group are masked (generate-time constant compare):
  hidden layers output 0; the output layer flags them invalid so they never win argmax.

### 7.5 Output collection

Hidden layers: the PN aligned bits form one word written to the next buffer at address g.
Output layer: PN popcounts plus a valid mask per group go to `bnn_argmax`.

## 8. Memories, buffers, input path

### 8.1 `bnn_ram`

Parameters `WIDTH`, `DEPTH`, `RAM_STYLE` (`"distributed"` default, `"block"`, `"ultra"`),
`READ_LATENCY` (≥ 1, always registered), `PORTS` (`"SP"` for weights/thresholds,
`"SDP"` for buffers). Implemented with inference plus `ram_style` attributes. All consumers
derive alignment delays from `READ_LATENCY`, so swapping styles is a parameter change.

### 8.2 Threshold storage

Per NP, depth G_l, width TW_l. Registers when G_l ≤ 2, otherwise `bnn_ram`.

### 8.3 `bnn_act_buf`

Two slots in an SDP memory of 2·DEPTH words × WIDTH bits plus slot state. Registers when
2·DEPTH ≤ 4, otherwise `bnn_ram`. Buf0: WIDTH = PW₀, DEPTH = C₀. Buf_l (l ≥ 1):
WIDTH = PN_{l−1}, DEPTH = G_{l−1} (= C_l).

### 8.4 `bnn_input_gate` and `bnn_gearbox`

- Binarize each pixel (`pixel >= 128`), zero pixels with `keep = 0`, and compact kept pixels
  in order (two-stage prefix compactor, supports sparse TKEEP).
- The gearbox packs bits into PW₀-bit words (any PW₀, variable fill), zero-pads the final
  word at TLAST, writes words 0…C₀−1 into the current Buf0 slot, and commits the slot at TLAST.
- Pixels beyond N₀ before TLAST are dropped. If TLAST arrives before N₀ pixels (a malformed
  image; the README allows assuming correct sizes), the remaining words are zero-filled, one
  per cycle, before the slot is committed. At most one word per cycle is emitted; the gearbox
  backpressures when PW₀ < pixels per beat.

### 8.5 Default memory footprint (LUTRAM)

| Memory | Shape | Estimate |
|---|---|---|
| L0 weights | 128 NPs × (98 × 16) SP | ~4,096 LUTs |
| L1 weights | 8 × (64 × 128) SP | ~1,024 LUTs |
| L2 weights | 5 × (64 × 8) SP | ~40 LUTs |
| L0 thresholds | 128 × (2 × 10) registers | 2,560 FFs |
| L1 thresholds | 8 × (32 × 9) | ~72 LUTs |
| Buf0 | 2 · 49 × 16 SDP | ~64 LUTs |
| Buf1 | 2 · 2 × 128 registers | 512 FFs |
| Buf2 | 2 · 32 × 8 SDP | ~10 LUTs |

Whole design estimate: ~15k LUTs, ~20k FFs, 0 BRAM/URAM/DSP (≈1% of the device).

## 9. Argmax and output

### 9.1 `bnn_argmax`

Stage A: registered tournament tree over a group's PN popcounts; a higher-index candidate wins
only when strictly greater; invalid candidates never win. Stage B: running best across groups,
replaced only by a strictly greater value. After the last group, the index goes to the FIFO.

### 9.2 Output stream

`axis_fifo` (depth `OUT_FIFO_DEPTH`) with registered `valid`/`data`.
`data_out_data` = class index zero-extended to `OUTPUT_BUS_WIDTH`;
`data_out_keep` = low ⌈`OUTPUT_DATA_WIDTH`/8⌉ bytes set; `data_out_last` = 1.
Elaboration fails if `$clog2(M_{NL−1}) > OUTPUT_DATA_WIDTH` or
`OUTPUT_DATA_WIDTH > OUTPUT_BUS_WIDTH`.

### 9.3 Elaboration checks

`$error` at elaboration for: `TOPOLOGY` entries < 1, `PARALLEL_INPUTS` < 1, any
`PARALLEL_NEURONS[l]` < 1, `INPUT_BUS_WIDTH` or `CONFIG_BUS_WIDTH` not a multiple of 8,
`INPUT_BUS_WIDTH` not a multiple of `INPUT_DATA_WIDTH`, unsupported `RAM_STYLE`,
`RAM_READ_LATENCY` < 1, and the output-width conditions in §9.2.

## 10. 900 MHz implementation rules

Budget at 1.111 ns: ~0.9 ns after clock-to-out and setup — one LUT plus a routed net, or one
LUT plus a short CARRY8 chain.

1. Every submodule output is registered; no combinational paths between modules.
2. One LUT level per stage for wide logic; at most two for narrow local logic.
3. Adders ≤ 12 bits; compares get their own stage.
4. Counter wrap and terminal-count flags are precomputed and registered one cycle early; FSMs one-hot.
5. Nets with fanout > ~32 use explicit replicated-register trees; replicas carry
   `(* keep = "true" *)` / `equivalent_register_removal` protection. Pipeline and broadcast
   registers carry `(* shreg_extract = "no" *)`.
6. Reset is registered and tree-distributed; datapath registers have no reset.
7. LUTRAM outputs are always registered.

## 11. Vivado flow

- `synth/signoff.tcl`: top `bnn_fcc_timing`, part `xcu250-figd2104-2L-e`, out-of-context,
  clock 1.111 ns; synth `PerformanceOptimized`, opt `Explore`, place `ExtraTimingOpt`,
  phys_opt `AggressiveExplore`, route `AggressiveExplore`, post-route phys_opt. Reports:
  timing summary (WNS/WHS/WPWS), utilization, worst paths, SLR placement.
- `synth/probe.tcl`: per-module out-of-context timing probe at 1.111 ns with registered I/O.
- openflex: `openflex bnn_fcc_timing.yml -c bnn_fcc.csv` (default directives) is the scored
  result; the RTL must close 900 MHz there without custom XDC or Pblocks.

## 12. Verification

### 12.1 Shared infrastructure (`verification/unit/common/`)

- `axis_agent`: parameterized AXI-Stream driver and monitor. Source mode: continuous,
  probability, and burst `valid` patterns, sparse TKEEP. Sink mode: always, random, burst
  `ready`, asserted before/with/after `valid`.
- Reference model: imports the provided `bnn_fcc_tb_pkg` read-only and reuses
  `BNN_FCC_Model` (`create_random`, `encode_configuration`, `compute_reference`,
  `layer_outputs`).
- Layout model: maps (neuron, chunk) → (NP, address, packed word) per §6.5 for checking the
  write bus and memory contents.

### 12.2 Environments

Each is a UVM 1.2 test/env/agent/scoreboard/covergroup set, swept across parameter sets with
`xelab -generic_top`.

| Env | Stimulus focus | Key coverage |
|---|---|---|
| `bnn_popcount` | PW ∈ {1, 3, 7, 8, 16, 64, 128, 784}; random, all-0, all-1 | PW, result extremes |
| `bnn_ram` | each RAM_STYLE × READ_LATENCY × SP/SDP | latency, port mode |
| `bnn_np` | random chunks, first/last spacing | threshold 0, N, N+1; popcount = threshold ± 1 |
| `bnn_act_buf` | random commit/release timing | both slots full/empty, simultaneous read/write |
| `bnn_input_gate` + `bnn_gearbox` | any PI, bus widths, sparse keep, gaps | PI vs pixels/beat, TLAST position, padding |
| `bnn_cfg_mgr` | random topologies, shuffled/partial messages, invalid headers, gaps | message order, layer × kind, drop rules, bus widths 8–512 |
| `bnn_layer` | write bus and input slot driven directly; random N/M/PW/PN | groups/chunks, padding cases, output backpressure |
| `bnn_argmax` | ties, invalid masks, multiple groups | tie positions, winning class |
| `bnn_core` (layers + buffers) | random topologies and parameters | layer count, all classes output |
| `bnn_fcc` (top) | `verification/coverage_plan.txt` scenarios: config order, partial/live reconfig, resets in every phase, backpressure, TKEEP | coverage-plan categories 1–5 |

### 12.3 Scripts

- `sim/run_unit.sh <env> [param-set]` and `sim/regress.sh` (`xvlog -sv -L uvm`, `xelab`, `xsim`).
- Coverage merged and reported with `xcrg`.
- `sim/lint.sh`: `verilator --lint-only -Wall` over `rtl/`.

## 13. File changes

Existing files (only these edits):
- `rtl/bnn_fcc.sv`: implemented.
- `verification/bnn_fcc_tb.sv`: DUT parameter mapping only — `PARALLEL_INPUTS = 16`,
  `PARALLEL_NEURONS = '{128, 8, 5}`. Custom topologies with a different layer count need a
  matching `PARALLEL_NEURONS` override.
- `openflex/bnn_fcc_timing.yml`, `openflex/bnn_fcc_verify.yml`: source file lists only.
- `openflex/rtl/bnn_fcc_timing.sv`: `PARALLEL_*` defaults only.

New: `rtl/*.sv` (§4.1), `verification/unit/**`, `sim/`, `synth/`, `docs/superpowers/**`.

## 14. Build order

1. `bnn_fcc_pkg`, `bnn_ram`, `bnn_popcount`
2. `bnn_np`
3. `bnn_layer`
4. `bnn_act_buf`
5. `bnn_core` integration (layers + buffers)
6. `bnn_input_gate`, `bnn_gearbox`
7. `bnn_cfg_mgr`
8. `bnn_argmax`, `axis_fifo`, `axis_skid`
9. `bnn_fcc` top level, TB mapping, openflex lists
10. 900 MHz closure iterations (sign-off script and openflex)

Each module gets its UVM unit test and a timing probe before integration.

## 15. Risks

| Risk | Mitigation |
|---|---|
| L0 broadcast/address fanout (~2k LUTRAM address pins, 128 NPs) | Balanced replicated trees, protected replicas, early timing probe |
| 128-bit popcount trees in L1 | Pipelined compressor tree, probe at 1.111 ns |
| Weight packer variable-offset mux (config only) | Two-stage alignment, PW%8 fast path |
| openflex default directives may close worse than the sign-off script | Close with margin in sign-off; treat openflex as the gate |
| LUTRAM pulse-width margin only 47 ps | No mitigation needed in RTL; monitor WPWS in every run |
| xsim UVM/constraint support gaps | Keep sequences to standard UVM 1.2 and simple constraints |
