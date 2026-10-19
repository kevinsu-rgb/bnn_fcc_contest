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
`tb_popcount`, `tb_bit_packer`, `tb_pingpong_buf`, `tb_neuron_processor`, `tb_bnn_layer` (includes LPS=1/2 cases), `tb_argmax`, `tb_axis_out`, `tb_input_packer`, `tb_cfg_byte_serializer`, `tb_cfg_parser`, `tb_cfg_layer_writer`, `tb_cfg_sequences` (4-layer, 5-layer and an LPS=1 / argmax LPS=1 / FIFO depth 1 instance), `tb_mnist` (real model, 100 MNIST images, 0.8 valid probability, random `data_out_ready`).

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
- Synthesis (Vivado 2025.2, XCU250-FIGD2104-2L-E, PI 66, PN {32,3,1}): fMax 414.59 MHz; LUT 19035 (LUTAsLogic 13407, LUTAsMem 5628), REG 9028, CARRY8 218, BRAM 12, URAM 0, DSP 0. Previous design (PI 66 for every layer, PN {32,11,1}): fMax 431.22 MHz; LUT 19965 (LUTAsLogic 12655, LUTAsMem 7310), REG 8971, CARRY8 186, BRAM 0.
