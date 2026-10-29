# sim/ - local simulation, lint and synthesis flow

All simulation uses Vivado 2025.2 xsim; lint uses Verilator. Run everything from the repository root.

## Prerequisites
- Vivado 2025.2. `sim/common.sh` sources `/tools/AMD/2025.2/Vivado/settings64.sh`; set `VIVADO_SETTINGS=/path/to/settings64.sh` to override.
- Verilator (for `sim/lint.sh`, `verilator --lint-only -Wall`).
- For synthesis: the openflex virtual environment (`~/envs/openflex`).

## Scripts
| Script | Purpose |
|---|---|
| `sim/run_unit.sh [tb ...]` | Compile and run the unit testbenches in `verification/unit/` (default: all). Build and log in `sim/build/<tb>/run.log`. Exit code = number of failures. |
| `sim/run_regress.sh [cfg ...]` | Run the provided `verification/bnn_fcc_tb.sv` through the wrappers in `sim/cfg/` (default: all). For `sfc_perf` it also prints throughput and latency. |
| `sim/lint.sh` | `verilator --lint-only -Wall --top-module bnn_fcc` over `rtl/` (must be clean). |

A run passes iff xvlog/xelab/xsim all exit 0, the log contains `SUCCESS`, and it contains no `Error:`, `Fatal:`, `FAILURE` or `ERROR:` line.

### Unit testbenches
`tb_popcount`, `tb_pingpong_buf`, `tb_neuron_processor`, `tb_bnn_layer` (includes LPS=1/2 cases), `tb_argmax`, `tb_axis_out`, `tb_input_packer`, `tb_cfg_byte_serializer`, `tb_cfg_parser`, `tb_cfg_layer_writer`, `tb_cfg_sequences` (4-layer, 5-layer and an LPS=1 / argmax LPS=1 / FIFO depth 1 instance), `tb_mnist` (real model, 100 MNIST images, 0.8 valid probability, random `data_out_ready`).

### Provided-TB regression configurations (`sim/cfg/`)
`sfc_stress`, `sfc_perf`, `sfc_pn1`, `sfc_alt`, `sfc_orig`, `c_8888`, `c_13753`, `c_13753_bus8`, `c_100`, `c_100_bus`, `c_tiny`. Each is a small module that instantiates `bnn_fcc_tb` with parameter overrides (array parameters cannot be set from the command line) and includes `xsim_tb_fix.svh`. A `c_5layer` configuration was dropped: the provided TB cannot elaborate `CUSTOM_LAYERS != 4` under xsim (`bnn_fcc_tb.sv:123`); a 5-layer topology is covered by `tb_cfg_sequences`.

## Why `sim/cfg/xsim_tb_fix.svh` exists
The provided TB is not edited, but xsim 2025.2 has three bugs that would make it fail or pass vacuously:
1. A block-local, uninitialized queue/dynamic array inside a loop is not re-created per iteration. This breaks `BNN_FCC_Model::load_single_layer` (MNIST mode: every neuron gets neuron 0's weights). `tb_mnist` therefore uses an xsim-safe subclass loader and the regressions use custom-topology mode.
2. `std::randomize()` on a dynamic array resizes it to 0. In custom-topology mode the TB then builds an all-zero model and empty images.
3. `.sum()` on a real queue returns 0, so the TB's own `Avg latency` line.

`xsim_tb_fix.svh` (included at the end of every wrapper) runs at t=1: it refills the TB's model weights, re-encodes the configuration stream and regenerates the images through the TB's public members (hierarchical access), and adds a monitor that prints `xsim_tb_fix: Avg latency (cycles) per image`. The TB's own driver, backpressure, checker and reference model still run. The TB's `Avg latency` line prints 0.0 under xsim; the `xsim_tb_fix` latency line is authoritative.

## Contest MNIST check
The contest's MNIST verification (`openflex/verify.sh`) uses Questa, which is not available locally. Run it where Questa is installed. Locally, `tb_mnist` checks all 100 MNIST images against `python/test_vectors/expected_outputs.txt`.

## Synthesis (openflex)
```
source ~/envs/openflex/bin/activate
source /tools/AMD/2025.2/Vivado/settings64.sh
cd openflex && openflex bnn_fcc_timing.yml -c bnn_fcc.csv
```
Target `XCU250-FIGD2104-2L-E`; generated outputs are gitignored.

## Latest results
`PARALLEL_INPUTS` sets the input width of the first hidden layer only; layers >= 1 read their whole fan-in in one pass (`docs/superpowers/specs/2026-10-18-parallel-inputs-first-layer-design.md`). Judged defaults: PI 66, PN {32,3,1}.
- `sim/lint.sh`: clean. Unit TBs: 13/13 PASS (`tb_cfg_sequences` now also runs a `topo_hid` instance). Provided-TB regression: 11/11 PASS.
- `tb_mnist` (PI 66, PN {32,3,1}): `MNIST: 100/100 match python` under stress (0.8 valid, random ready); avg latency 335.7 cycles, 125.5 cycles/output (stalls included, not steady state).
- `sfc_perf` (PI 66, PN {32,11,1} - this wrapper's own values, not the judged PN; 100 images, no stalls): the TB reports 100.5 cycles/output. That is (first beat to last output)/N and includes pipeline fill: (99*99 + 249)/100 = 100.5. Steady state is 99 cycles/image, bound by the TB's input driver (98 beats + 1 idle cycle). Latency 249 cycles (`xsim_tb_fix` monitor; 349 before this change). 995,024.9 outputs/s at a 10 ns clock.
- input_packer timing rework (overlong images clipped at word level via `ovf_q` instead of a per-beat pixel count; `bit_packer` `MASK_IN=0` skips the redundant input mask): lint clean, unit 13/13 PASS, regression 11/11 PASS, `sfc_perf` unchanged (100.5 cycles/output, latency 249). Synthesis with `RAM_STYLE` "auto" (committed default): fMax 455.79 MHz; LUT 21486 (LUTAsLogic 14082, LUTAsMem 7404), REG 9875, CARRY8 214, BRAM 0. Unmodified HEAD re-run in the same flow reproduces the 414.59 MHz / BRAM 12 line below. Two effects are mixed: the worst input_packer path went from -1.412 ns (9 levels, 2 CARRY8) to -1.133 ns, and Vivado re-mapped layer-1 weights from BRAM (still BRAM in the preliminary synthesis map) to LUTRAM, removing the BRAM-read path that was #2 in the old build. With `RAM_STYLE` "distributed" (same memory mapping) the same RTL gave 424.27 MHz, so treat single-run fMax as noisy by several percent. New worst paths (-1.19 ns): `u_in/pk/cnt_q` -> `pk_valid && !ovf_q` -> layer-0 ping-pong bit enables, and config-side packer reset/CE paths.
- Packer removal and config-path pipelining (`docs/superpowers/specs/2026-10-28-packer-removal-config-pipeline-design.md`): `bit_packer` deleted; image input is position-based (pixel i = lane i % K of beat i / K, TKEEP = 0 lanes stored as 0) and writes 8-bit chunks straight into the layer-0 ping-pong; config writers stage one neuron at fixed byte positions and emit PASSES words through registered write ports; the parser drives a registered one-hot `msg_sel`; `pingpong_buf` reads through a plain mux; a new message waits for every writer to be idle. Lint clean, unit 12/12 PASS (`tb_bit_packer` removed; `tb_cfg_sequences` adds a `topo_pi1` instance), regression 11/11 PASS, `tb_mnist` 100/100 (avg latency 334.8, 124.7 cycles/output under stress). `sfc_perf`: 100.5 cycles/output (unchanged), latency 248 cycles (was 249). Synthesis (`RAM_STYLE` "distributed", PI 66, PN {32,3,1}): fMax 554.94 MHz (baseline 424.27 MHz, same flow); LUT 19275 (LUTAsLogic 11871, LUTAsMem 7404) vs 21834; REG 11104 vs 9860; CARRY8 210; F7MUX 198; BRAM 0. Per module (LUT / FF, before -> after): config writers 1874 / 841 -> 781 / 2151; layer-0 ping-pong + input_packer 3213 / 1747 -> 2132 / 1659 (the layer-0 bit enables are reported under `u_in` after cross-boundary optimization); serializer 108 -> 58; parser 219 -> 171. The old worst classes (serializer/parser -> writer packer, -1.36 ns; writer packer -> weight-RAM WE; input packer, -1.23 ns) are gone; worst slack is now -0.802 ns inside the layer-0 popcount (2 CARRY8), then ping-pong `rd_data` -> popcount stage 1 (-0.783 ns) and ping-pong read mux (-0.76 ns); config paths are at about -0.74 ns. The datapath (neuron_processor/popcount) is unchanged, so part of the gain is lower routing pressure rather than shorter logic; single-run fMax is noisy by several percent.
- Synthesis (Vivado 2025.2, XCU250-FIGD2104-2L-E, PI 66, PN {32,3,1}): fMax 414.59 MHz; LUT 19035 (LUTAsLogic 13407, LUTAsMem 5628), REG 9028, CARRY8 218, BRAM 12, URAM 0, DSP 0. Previous design (PI 66 for every layer, PN {32,11,1}): fMax 431.22 MHz; LUT 19965 (LUTAsLogic 12655, LUTAsMem 7310), REG 8971, CARRY8 186, BRAM 0.
