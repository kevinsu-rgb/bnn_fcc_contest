# BNN FCC Accelerator — Design Spec

Date: 2026-10-10
Status: Approved in brainstorming; pending written-spec review

## 1. Goal and Scope

Complete the contest's `bnn_fcc` module (AXI4-Stream binary neural network,
fully connected classifier) in SystemVerilog, verified with the provided
testbench in Vivado xsim and implemented with Vivado on an UltraScale+ part.

The number of neuron processors is configurable **per layer**, independent of
the neuron count: e.g. layer 1 can use 1 processor for its 256 neurons, which
then time-multiplexes all 256.

### Decisions (from brainstorming)

| Topic | Decision |
| :--- | :--- |
| Optimization target | Maximum throughput |
| Neuron parallelism | `PARALLEL_NEURONS[TOTAL_LAYERS-1]`, per layer |
| Input parallelism | `PARALLEL_INPUTS`, one global value for all layers |
| Layer execution | `PARALLELIZE_LAYERS=1`: pipelined across images; `=0`: one image in flight |
| Architecture | Neuron-outer, broadcast input chunk (approach A) |
| FPGA | `xczu7ev-ffvc1156-2-e`, out-of-context implementation |
| Clock | Push to maximum Fmax (sweep), report best achieved |
| Existing RTL | May be rewritten or deleted freely |
| `PN[l] > TOPOLOGY[l+1]` | `$fatal` at elaboration (no clamping) |
| New verification | No unit testbenches. Run scripts + a `bind`-attached perf monitor only |

### Constraints

- Existing documentation (`README.md`, `rtl/README.md`, `verification/README.md`,
  `doc/`, `architecture/`) is not modified.
- Existing testbench files are not modified, **except** the DUT parameter block of
  `verification/bnn_fcc_tb.sv` (section 9). `rtl/README.md` names this as the one
  allowed TB change, and the TB comments invite adapting that block.
- Tools: Vivado 2025.2 (`/tools/AMD/2025.2/Vivado`); xsim is the simulator.
- Reset is synchronous, active high (per `rtl/README.md`).
- The top-level port list of `bnn_fcc` is unchanged.

## 2. Interface Facts the Design Relies On

Taken from `rtl/README.md` and `verification/bnn_fcc_tb_pkg.sv`:

- Config messages: 128-bit header (`msg_type`[7:0], `layer_id`[15:8],
  `layer_inputs`[31:16], `num_neurons`[47:32], `bytes_per_neuron`[63:48],
  `total_bytes`[95:64], reserved[127:96]), little-endian bytes, then
  `total_bytes` payload bytes.
- `layer_id` 0 is the first neuron layer (the input layer has no id).
- TB order: L0 weights, L0 thresholds, L1 weights, L1 thresholds, …, last-layer
  weights. The output layer receives no thresholds.
- TB starts every message on a new beat and flushes partial beats with TKEEP.
  `config_last` is asserted **only on the final beat of the whole
  configuration**, so it cannot frame messages.
- Weight payload: weight *i* of a neuron is bit *i%8* of byte *i/8*; each neuron
  is padded with 1s to a byte boundary.
- Threshold payload: 4 bytes per neuron, little-endian, value is an `int`
  (signed 32-bit). Neuron fires when `popcount >= threshold`.
- Image: `TOPOLOGY[0]` pixels, `INPUT_BUS_WIDTH/INPUT_DATA_WIDTH` per beat, TKEEP on
  the final partial beat, TLAST on the last beat. Binarize as `pixel >= 128`.
- Output: argmax of the last layer's popcounts, ties to the lowest index;
  `data_out_last` always 1; `data_out_keep` per AXI semantics.
- The TB inserts one idle cycle between images, so the input-bound rate for
  SFC on a 64-bit bus is 1 image per 99 cycles.

## 3. Architecture

```
            config AXI                                      data_out AXI
                │                                                ▲
          ┌─────▼──────┐   weight/threshold write ports    ┌─────┴─────┐
          │config_loader├──────────┬──────────┬──────────► │ out_stage │
          └────────────┘           │          │            │ (FIFO +   │
 data_in AXI                       ▼          ▼            │  AXI)     │
     │     ┌──────────┐  ┌───┐ ┌───────┐ ┌───┐ ┌───────┐   └─────▲─────┘
     └────►│input_stage├─►│AB0├►│layer 0├►│AB1├►│layer 1├► … ►argmax
           └──────────┘  └───┘ └───────┘ └───┘ └───────┘
```

`bnn_fcc` instantiates one `config_loader`, one `input_stage`, `TOTAL_LAYERS-1`
activation buffers (AB*l* holds the input vector of layer *l*), `TOTAL_LAYERS-1`
`bnn_layer`s (generate loop, any depth), `argmax`, and `out_stage`.

### Flow-control rule

A stage starts an image only when it has a full input bank **and** a reserved
place for its result. Once started, an image runs to completion with no
mid-image stall. Back-pressure therefore exists only at image boundaries, and
the compute pipelines have no stall logic.

Space is tracked with **credits**: a producer holds a counter initialised to the
downstream capacity (bank count of the next activation buffer, or the output
FIFO depth for the last layer). It decrements on image start and increments when
the consumer releases that space. Control signals for the downstream buffer
(start, write, commit) travel down the pipeline with the data, so they arrive in
order. This lets image *k+1* start the cycle after image *k*'s final read.

### Modes

- `PARALLELIZE_LAYERS=1`: activation buffers have 2 banks; every stage works on a
  different image concurrently.
- `PARALLELIZE_LAYERS=0`: activation buffers have 1 bank, and `input_stage`
  admits a new image only after the previous result completes its `data_out`
  handshake (exactly one image in flight).

Both modes give every layer its own processors and memories.

## 4. `config_loader`

- **Byte walker.** Latches one config beat, then consumes its byte lanes one per
  cycle, skipping lanes with TKEEP=0. `config_ready` is high only when the beat
  register is empty (or about to be emptied). This handles any
  `CONFIG_BUS_WIDTH`, sparse TKEEP, and messages that begin partway through a beat.
  Config throughput is 1 byte/cycle (~36K cycles for SFC). This is accepted
  because configuration is outside the image latency and throughput metrics.
- **FSM.** `HEADER` collects 16 bytes and decodes the fields. `PAYLOAD` consumes
  exactly `total_bytes`, then returns to `HEADER`. TLAST is ignored for framing.
- **Routing.** `layer_id` selects the target layer; `msg_type` selects weights
  or thresholds. Neuron *n* of layer *l* belongs to processor
  `p = n mod PN[l]`, group `g = n div PN[l]`, tracked by incrementing counters.
  Writes outside the compiled `TOPOLOGY` (unknown `layer_id`, surplus neurons or
  bytes) are dropped while bytes are still consumed, so parsing stays aligned.
- **Weight packing.** A bit packer collects a neuron's bytes into `PI`-bit words.
  A partial final word is padded with 1s. At most
  `CHUNKS = ceil(FANIN/PI)` words are written per neuron, so surplus byte
  padding is discarded. Works for any `PI >= 1`; for `PI < 8` the packer emits
  several words per byte and back-pressures the byte walker. Word *c* of neuron
  *n* is written to processor *p*'s weight memory at address `g*CHUNKS + c`.
- **Thresholds.** 4 bytes form a signed 32-bit value, saturated to
  `[0, FANIN+1]` and stored in `$clog2(FANIN+2)` bits. Saturation preserves
  `popcount >= threshold` exactly: `<= 0` always fires, `> FANIN` never fires.
- **`config_done`.** Set once every layer's weight message, and every hidden
  layer's threshold message, has been fully received. `data_in_ready` is held low
  until then.

## 5. `input_stage`

- A beat has `BPB = INPUT_BUS_WIDTH/INPUT_DATA_WIDTH` pixel lanes. A lane is valid
  when all of its TKEEP bytes are set.
- Binarize: `bit = pixel >= 2**(INPUT_DATA_WIDTH-1)` (i.e. `>= 128` for 8-bit).
- Compaction: valid bits are compacted in lane order (prefix sum over `BPB`
  lanes) and fed into a `2*BPB`-bit gearbox. The gearbox emits aligned `BPB`-bit
  words to AB0 at word index `w`.
- On TLAST: flush the partial word (zero fill) and commit the bank. Pixels beyond
  `TOPOLOGY[0]` are dropped. Missing pixels (short image) read as 0 because a
  bank is cleared when acquired.
- `data_in_ready = config_done && AB0 write bank available`, and in sequential
  mode also `&& !image_in_flight`. A bank is held from an image's first beat to
  its TLAST. Rate: 1 beat/cycle.

## 6. `act_buffer`

Parameters: `WIDTH` (bits held), `WW` (write word width), `PI` (read chunk
width), `NUM_BANKS` (1 or 2).

- Storage: `NUM_BANKS` x `WIDTH` flip-flops. Bit *i* = input/neuron *i*.
- Producer port: `wr_start` (acquire the next bank and clear it in one cycle,
  using the FFs' synchronous reset), `wr_en` / `wr_idx` / `wr_data[WW]` (write bits
  `[wr_idx*WW +: WW]`; bits `>= WIDTH` are dropped), `wr_commit` (mark full).
- Consumer port: `rd_avail` (a full bank exists), `rd_chunk` (registered output
  of bits `[c*PI +: PI]`; bits `>= WIDTH` read 0), `rd_release` (mark free and
  return a credit to the producer).
- Bank pointers rotate. With 2 banks the producer fills one while the consumer
  reads the other.
- The read mux is `CHUNKS:1` per output bit. An optional extra register stage is
  added if it limits Fmax; controller latency is a parameter, so this is absorbed.

AB0: `WIDTH=TOPOLOGY[0]`, `WW=BPB`. AB*l* (l>=1): `WIDTH=TOPOLOGY[l]`, `WW=PN[l-1]`.

## 7. `bnn_layer` and `neuron_proc`

Per layer *l*: `FANIN=TOPOLOGY[l]`, `N=TOPOLOGY[l+1]`, `PN=PARALLEL_NEURONS[l]`,
`PI=PARALLEL_INPUTS`, `CHUNKS=ceil(FANIN/PI)`, `GROUPS=ceil(N/PN)`,
`IS_OUTPUT = (l == TOTAL_LAYERS-2)`.

### Controller

- Start when the input buffer's `rd_avail` is high and a downstream credit is
  available; reserve the credit.
- Issue one read per cycle for `g in 0..GROUPS-1`, `c in 0..CHUNKS-1`: activation
  chunk *c*, weight address `g*CHUNKS + c` (a single counter
  `0..GROUPS*CHUNKS-1`), threshold address *g*.
- Tags `first` (c==0), `last` (c==CHUNKS-1), `g`, plus downstream
  `start` / `commit` markers, travel in a delay line matched to datapath latency.
- After the final read, `rd_release` the input bank.
- Throughput: exactly `GROUPS*CHUNKS` cycles per image, back to back.
  Latency: that plus the pipeline depth.

### Memories (per processor *p*)

- Weight RAM: depth `GROUPS*CHUNKS`, width `PI`, simple dual-port (config write,
  compute read), registered output. `ram_style` is left to Vivado.
- Threshold RAM: depth `GROUPS`, width `$clog2(FANIN+2)`. Absent when `IS_OUTPUT`.

### `neuron_proc` pipeline

1. Register activation chunk and weight word.
2. `x = ~(act ^ w) & mask`. The mask is all ones except on `last`, where only
   the low `FANIN - (CHUNKS-1)*PI` bits are kept. Correctness is therefore
   independent of padding values.
3. Popcount adder tree, with a register every `POP_LEVELS_PER_STAGE` levels
   (Fmax tuning parameter).
4. `acc <= first ? pc : acc + pc`, width `$clog2(FANIN+1)`.
5. On `last`: hidden layer outputs `acc >= thr`; output layer outputs `acc`.

### Group output

- Hidden layers write the `PN` result bits to the next buffer at word *g*.
- In a partly filled last group, lanes with `g*PN + p >= N` are discarded: the
  buffer drops bits `>= WIDTH`, and `argmax` gets a per-lane valid mask.

## 8. `argmax` and `out_stage`

- `argmax` stage 1: a pipelined comparison tree reduces a group's `PN`
  (popcount, index) pairs to one, skipping invalid lanes; ties go to the lower
  index.
- `argmax` stage 2: running max across groups. The first group loads it; later
  groups replace it only when strictly greater. Groups arrive in increasing index
  order, so overall ties go to the lowest index. After the last group, the index
  is pushed to `out_stage`.
- `out_stage`: FIFO of `OUT_FIFO_DEPTH` (default 2) entries = the last layer's
  credit count. `data_out_data` = index zero-extended to `OUTPUT_BUS_WIDTH`;
  `data_out_keep` sets the low `ceil(OUTPUT_DATA_WIDTH/8)` bytes;
  `data_out_last = 1`. Each handshake returns a credit and, in sequential mode,
  clears `image_in_flight`.

## 9. Parameters

### `bnn_fcc`

Existing parameters are unchanged. The parallelism parameters become:

```systemverilog
parameter bit PARALLELIZE_LAYERS = 1'b1,
parameter int PARALLEL_INPUTS    = 64,
parameter int PARALLEL_NEURONS[TOTAL_LAYERS-1] = '{default: 1}
```

Elaboration `$fatal` checks:

- `CONFIG_BUS_WIDTH`, `INPUT_BUS_WIDTH`, `OUTPUT_BUS_WIDTH` are multiples of 8.
- `INPUT_DATA_WIDTH % 8 == 0` and `INPUT_BUS_WIDTH >= INPUT_DATA_WIDTH`.
- `TOTAL_LAYERS >= 2` and `TOTAL_LAYERS - 1 <= 256` (8-bit `layer_id`).
- Every `TOPOLOGY[i]` is in `1..65535` (16-bit header fields).
- `PARALLEL_INPUTS >= 1`.
- `1 <= PARALLEL_NEURONS[l] <= TOPOLOGY[l+1]` for every layer.
- `OUTPUT_DATA_WIDTH >= $clog2(TOPOLOGY[TOTAL_LAYERS-1])` and
  `OUTPUT_BUS_WIDTH >= OUTPUT_DATA_WIDTH`.

Internal tuning parameters (localparams or defaulted parameters):
`POP_LEVELS_PER_STAGE`, `OUT_FIFO_DEPTH`, act-buffer read register stages.

### Testbench edit (DUT parameter block of `verification/bnn_fcc_tb.sv` only)

```systemverilog
parameter bit PARALLELIZE_LAYERS = 1'b1,
parameter int PARALLEL_NEURONS[(USE_CUSTOM_TOPOLOGY ? CUSTOM_LAYERS : 4)-1] = '{37, 12, 1},
parameter int PARALLEL_INPUTS = 64
```

### Default sizing for SFC (64-bit bus, PI=64)

| Layer | FANIN | N | PN | CHUNKS | GROUPS | Cycles per image |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0 | 784 | 256 | 37 | 13 | 7 | 91 |
| 1 | 256 | 256 | 12 | 4 | 22 | 88 |
| 2 | 256 | 10 | 1 | 4 | 10 | 40 |

Every layer is under the 98–99 cycle input time, so the input bus is the
bottleneck. These values may be retuned after the first implementation run.

## 10. RTL Files

| File | Action |
| :--- | :--- |
| `rtl/bnn_fcc.sv` | Rewrite (top level) |
| `rtl/config_loader.sv` | New |
| `rtl/input_stage.sv` | New (replaces `binarize.sv`) |
| `rtl/act_buffer.sv` | New |
| `rtl/bnn_layer.sv` | New |
| `rtl/neuron_proc.sv` | Rewrite |
| `rtl/argmax.sv` | Rewrite (streaming) |
| `rtl/out_stage.sv` | New |
| `rtl/delay.sv` | Convert to synchronous reset (used for tag delay lines) |
| `rtl/binarize.sv`, `rtl/bnn_topology.sv`, `rtl/neuron_proc_popcount.sv` | Delete |

Small helpers (e.g. a popcount tree, simple dual-port RAM) may be split into
their own files if a module grows large.

## 11. Simulation and Measurement (`sim/`)

xsim cannot override unpacked-array parameters from the command line:
`-generic_top N={1,2,3}` elaborates with garbage values. This was verified on
2025.2. It affects `CUSTOM_TOPOLOGY` and `PARALLEL_NEURONS`.

- `sim/run_sim.sh`: compiles and runs `bnn_fcc_tb` with xsim. Scalar options are
  passed via `-generic_top`. For array options (`--topology`, `--pn`) it writes a
  patched **copy** of `bnn_fcc_tb.sv` into `sim/build/<run>/` with the defaults
  replaced, so the repository TB is never touched. It returns nonzero unless the
  log contains `SUCCESS` and no `Error`/`FATAL`.
- `sim/regress.sh`: runs a configuration list and prints a pass/fail table:
  - MNIST, 100 images, stress settings (`TOGGLE_DATA_OUT_READY=1`,
    `CONFIG_VALID_PROBABILITY=0.8`, `DATA_IN_VALID_PROBABILITY=0.8`), both
    `PARALLELIZE_LAYERS` values
  - `PARALLEL_NEURONS` of `{1,1,1}`, `{37,12,1}`, `{5,3,3}`, and fully parallel
    `{256,256,10}`
  - `PARALLEL_INPUTS` in `{1, 4, 13, 64, 784}`
  - bus widths `{32, 64, 256}` for config and input
  - custom topologies such as `13-7-5-3`, `9-1-2`, and a 5-layer case
  - `TIMEOUT` is scaled per configuration (`PN={1,1,1}` is ~53K cycles per image).
    It is a `realtime` parameter, so `run_sim.sh` sets it in the patched TB copy
    (same mechanism as the array options) rather than relying on `-generic_top`.
- `sim/perf_monitor.sv`: attached to `bnn_fcc` with `bind` (no TB edit). It
  timestamps each image's first accepted input beat and its output handshake, and
  prints per-image latency, average and minimum latency in cycles, and steady-state
  cycles per image. Perf runs use `TOGGLE_DATA_OUT_READY=0` and probabilities of
  1.0, per the TB's guidance.

## 12. Implementation (`synth/`)

- `synth/build.tcl`: non-project Vivado flow on `xczu7ev-ffvc1156-2-e`, top
  `bnn_fcc`, `-mode out_of_context`, generics for the chosen configuration. The
  XDC `create_clock` period is a Tcl argument. Runs synth, opt, place, phys_opt,
  route, then writes `report_utilization`, `report_timing_summary`, and
  `report_power` to `synth/reports/<run>/`.
- `synth/fmax_sweep.sh`: binary search on clock period until routed WNS >= 0,
  then reports the best Fmax and its utilization.

## 13. Success Criteria

1. `bnn_fcc_tb` (with only the parameter-block edit) passes MNIST, 100 images,
   under contest stress settings, with `PARALLELIZE_LAYERS` = 1 and = 0.
2. Every configuration in `sim/regress.sh` passes.
3. With SFC defaults, a 64-bit bus, `PARALLELIZE_LAYERS=1`, and no back-pressure,
   steady-state throughput is input-bound: 1 image per 99 cycles in the TB.
4. Vivado out-of-context implementation completes on `xczu7ev-ffvc1156-2-e` with
   no inferred latches, no combinational loops, and routed WNS >= 0 at the
   reported Fmax.
5. Latency, throughput, Fmax, and utilization numbers are produced by the scripts
   for the contest report.

## 14. Out of Scope

- Unit testbenches for individual modules.
- Editing existing docs or TB files beyond the parameter block.
- Run-time reconfiguration without reset (the README says reset precedes a new
  configuration).
- A panic/error output port (the port list is fixed); malformed config is
  tolerated by dropping out-of-range writes.
