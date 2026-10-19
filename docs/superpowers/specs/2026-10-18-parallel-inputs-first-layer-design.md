# PARALLEL_INPUTS Applies to the First Hidden Layer Only — Design Spec

Date: 2026-10-18
Status: Draft (awaiting user review)
Amends: `docs/superpowers/specs/2026-10-10-bnn-fcc-design.md` (that file is left unchanged; this spec supersedes the sections listed in §2).

---

## 1. Intent and agreed requirements

### 1.1 Stated by the user
| # | Requirement |
|---|---|
| P1 | `PARALLEL_INPUTS` is the number of inputs consumed simultaneously in the **first hidden layer only**, as `rtl/README.md` describes ("Number of inputs consumed simultaneously in the first hidden layer"). |
| P2 | Every later non-input layer (l ≥ 1) consumes its **entire fan-in** each cycle (option B in brainstorming): `PW[l] = F[l]`. |
| P3 | Judged defaults: `PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 3, 1}`. |
| P4 | No new `sim/cfg` wrapper (explicitly declined). The "Latest results" section of `sim/README.md` may be refreshed. |
| P5 | Standing rules unchanged: provided docs, provided TBs and existing `sim/cfg` wrappers are not edited; only the TB DUT parameter mapping defaults and the `PARALLEL_*` defaults in `openflex/rtl/bnn_fcc_timing.sv` may change; our own unit TBs in `verification/unit/` may change. |

### 1.2 Facts established from the repository
- Today `PW[l] = min(PARALLEL_INPUTS, F[l])` for **every** non-input layer (`rtl/bnn_fcc.sv:172`, decision U5 of the 2026-10-10 spec).
- `pingpong_buf`, `bnn_layer`, `neuron_processor`, `popcount` and `cfg_layer_writer` are generic in PW and already handle `PW = F` (`PASSES = 1`): the `c_8888` regression (fan-in 8, PI 66) and `tb_bnn_layer` cases c6/c7 run single-pass layers today. The largest single-pass case exercised so far is F = 8.
- No `sim/cfg` wrapper uses PN `{32,3,1}`; `sfc_perf`/`sfc_stress`/`c_8888` use `{32,11,1}`.
- Current results (`sim/README.md`): 99 cycles/image steady state (input-driver bound), 349-cycle latency (`sfc_perf`), fMax 431.22 MHz, LUT 19,965, REG 8,971, BRAM 0.

### 1.3 Trade-offs accepted (raised during brainstorming)
1. Hidden layers lose their input-width knob: the minimum size of a hidden layer is one NP with F[l] XNOR lanes and an F[l]-input popcount. Large custom hidden fan-ins produce correspondingly wide NPs.
2. Deeper popcount in layers ≥ 1: 256 inputs = 8 adder levels = 3 pipeline stages at `POPCOUNT_LEVELS_PER_STAGE = 3` (layer 0 at PW 66: 7 levels).
3. XNOR lanes rise from 2904 to 3136 (+8%); NPs fall from 44 to 36. Hidden layers can only grow in steps of F[l] lanes, so layer 2 gets 256 lanes for a job that needs ≈ 27 and is idle ≈ 90% of the time.
4. Weight RAMs keep the same total bits (F×N per layer) but become wide and shallow (layer 1: 86 × 256 per NP instead of 96 × 66).
5. Existing `sim/cfg` regressions compile unchanged but now build option-B hardware with their own PN values (e.g. `sfc_pn1` layer 1: 256 cycles/image instead of 1024).
6. The provided TB comment (`verification/bnn_fcc_tb.sv:80`) and `rtl/README.md:16` already say "first hidden layer"; no doc edits are needed for consistency.

---

## 2. Superseded sections of the 2026-10-10 spec
| 2026-10-10 section | Replaced by |
|---|---|
| U5 (single scalar P_W for every non-input layer) | P1, P2, §3.1 |
| §1.3 bullet "`PARALLEL_INPUTS` … meaning widens … to all non-input layers" | §3.1 |
| §3.1 `PW[l] = min(PARALLEL_INPUTS, F[l])` | §3.1 |
| §3.2 default judged configuration table and selection rationale | §3.2 |
| §8 "Set `PARALLEL_NEURONS = '{32,11,1}`" and the synthesis result | §4 (defaults), §5 (new result recorded in `sim/README.md`) |

All other sections (architecture, configuration manager, protocol handling, ping-pong buffer, layer engine, argmax, output) stand unchanged.

---

## 3. Design

### 3.1 Parameter semantics
Parameter types and the port list do not change.

- `PARALLEL_INPUTS` (scalar `int`, ≥ 1): inputs consumed per cycle by layer 0.
  `PW[0] = min(PARALLEL_INPUTS, F[0])`.
- Layers l ≥ 1: `PW[l] = F[l]`, hence `PASSES[l] = 1` and cycles/image `= GROUPS[l] = ceil(N[l] / PN[l])`. `PARALLEL_INPUTS` has no effect on them.
- `PARALLEL_NEURONS[l]` (≥ 1): unchanged, `PN[l] = min(PARALLEL_NEURONS[l], N[l])`.
- All other derived values keep their 2026-10-10 definitions (`GROUPS`, `PASSES`, `ACC_W`, `THR_W`).
- Error handling: the existing elaboration checks (`PARALLEL_INPUTS ≥ 1`, every `PARALLEL_NEURONS[l] ≥ 1`) are sufficient; there is no new illegal value.

### 3.2 Judged configuration (`PARALLEL_INPUTS = 66`, `PARALLEL_NEURONS = '{32, 3, 1}`)
| Layer | F → N | PW | PN | Passes × groups | Cycles/image | XNOR lanes | Weight RAM per NP |
|---|---|---|---|---|---|---|---|
| 0 | 784 → 256 | 66 | 32 | 12 × 8 | 96 | 2112 | 96 × 66 |
| 1 | 256 → 256 | 256 | 3 | 1 × 86 | 86 | 768 | 86 × 256 |
| 2 | 256 → 10 | 256 | 1 | 1 × 10 | 10 | 256 | 10 × 256 |

Total 3136 XNOR lanes, 36 NPs. Throughput is unchanged: 96 cycles/image for the slowest layer, below the input bus bound (98 beats + 1 idle cycle = 99).
Selection: under P2, PN1 = 3 and PN2 = 1 are the minimum values that keep layers 1 and 2 at ≤ 97 cycles/image. Layer 0 keeps PI 66 / PN 32 (7-level popcount, already timing-closed). PI 131 / PN0 16 (3120 lanes, 20 NPs, 8 levels) and PI 262 / PN0 8 (3120 lanes, 12 NPs, 9 levels) were considered and not chosen.

### 3.3 Hardware effect
- The ping-pong buffer in front of layer l ≥ 1 becomes `WR_W = PN[l-1]`, `RD_W = F[l]`: chunked writes as today, one full-width read (`NRD = 1`, no read mux).
- `bnn_layer` for l ≥ 1 runs one pass per group; each NP's popcount is F[l] wide and its accumulator sees one pass.
- `cfg_layer_writer` for l ≥ 1 packs each neuron's full weight row into one F[l]-bit word at address `group`.
- No module other than `bnn_fcc` changes.

---

## 4. Files changed
| File | Change | Allowed by |
|---|---|---|
| `rtl/bnn_fcc.sv:172` | `localparam int PW_L = (l == 0) ? imin(PARALLEL_INPUTS, F_L) : F_L;` | RTL |
| `rtl/bnn_fcc.sv:6` | Header comment: `PW = min(PARALLEL_INPUTS, F)` for layer 0, `PW = F` for layers ≥ 1 | RTL |
| `verification/bnn_fcc_tb.sv:118` | Default `PARALLEL_NEURONS = '{32, 3, 1}` (`PARALLEL_INPUTS` stays 66) | TB parameter-mapping exception |
| `openflex/rtl/bnn_fcc_timing.sv:25` | Default `PARALLEL_NEURONS = '{32, 3, 1}` | openflex `PARALLEL_*` exception |
| `verification/unit/tb_cfg_sequences.sv:1050` | Cycle-bound formula uses `PW = (l == 0) ? min(PARALLEL_INPUTS, F) : F` | own unit TB |
| `verification/unit/tb_bnn_layer.sv` | New single-pass full-width cases (§5.2) | own unit TB |
| `verification/unit/tb_pingpong_buf.sv` | New full-width-read cases (§5.2) | own unit TB |
| `verification/unit/tb_mnist.sv:2,90` | Judged `PARALLEL_NEURONS = '{32, 3, 1}` and header comment | own unit TB |
| `sim/README.md` | Refresh "Latest results" only | P4 |

Not changed: the existing `sim/cfg/*.sv` wrappers, `sim/*.sh`, provided TB logic, `rtl/README.md`, `verification/README.md`, `doc/`, `openflex/README.md`, the 2026-10-10 spec.

---

## 5. Verification

### 5.1 Failing test first
Update the `tb_cfg_sequences` zero-gap cycle bound (§4). For topo4 (`'{20,12,9,5}`, PI 4) layer 1 goes from 3 passes to 1, so the bound tightens and the unchanged RTL is expected to exceed it. If it does not fail, add a test that does (e.g. a top-level elaboration/latency check of layer-1 cycles/image) before changing the RTL.

### 5.2 New unit cases (coverage; these modules are already generic)
- `tb_bnn_layer`:
  - F 256, N 256, PW 256, PN 3, hidden
  - F 256, N 10, PW 256, PN 1, output
  - F 256, N 10, PW 256, PN 1, output, LPS 1 (deepest popcount pipeline)
- `tb_pingpong_buf`: NBITS 256 with (WR_W 32, RD_W 256) and (WR_W 3, RD_W 256).
- `tb_mnist`: run at the judged parallelism `{32,3,1}`.

### 5.3 Pass criteria (outputs reported verbatim)
1. `sim/lint.sh`: clean.
2. `sim/run_unit.sh`: every unit TB prints SUCCESS/PASS.
3. `sim/run_regress.sh`: all 11 provided-TB configurations PASS.
4. `tb_mnist`: `MNIST: 100/100 match python`.
5. `sfc_perf` (PN `{32,11,1}`, now option-B hardware with the judged layer 0): steady state 99 cycles/image. Its latency is reported as `sfc_perf` latency, not as the judged configuration's latency.
6. openflex synthesis at the judged defaults: report fMax, LUT (logic/mem), REG, CARRY8, BRAM, URAM, DSP against 431.22 MHz / 19,965 LUT. If fMax < 100 MHz (U10), stop and report; no timing tuning is in scope.
7. Questa MNIST check (`openflex/verify.sh`): run by the user; not available locally.

### 5.4 Coverage gap (accepted)
No provided-TB (`bnn_fcc_tb`) run of PN `{32,3,1}` happens locally: the existing wrappers fix their PN values and a new wrapper was declined (P4). The judged configuration is covered by `tb_mnist` (full DUT, 100 MNIST images), the new `tb_bnn_layer`/`tb_pingpong_buf` cases, and the user's Questa run of `verify.sh`.

---

## 6. Out of scope
- Per-layer `PARALLEL_INPUTS` arrays, chained widths (`PW[l] = PN[l-1]`), auto-derived widths, or new parameters for hidden-layer widths (options A, C, D).
- Timing optimization of the 256-input popcount.
- Removing width-conversion logic from `pingpong_buf` (it stays generic; layer 0 and custom topologies still use it).
