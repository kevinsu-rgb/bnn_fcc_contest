# Packer Removal and Config-Path Pipelining — Design Spec

Date: 2026-10-28
Status: Draft (awaiting user review)
Amends: `docs/superpowers/specs/2026-10-10-bnn-fcc-design.md` and `docs/superpowers/specs/2026-10-18-parallel-inputs-first-layer-design.md` (both left unchanged; this spec supersedes the sections listed in §2).
Baseline: commit `209d7b7` (424.27 MHz, LUT 21,834, REG 9,860, BRAM 0; PI 66, PN {32,3,1}, `RAM_STYLE` "distributed").

---

## 1. Intent and agreed requirements

### 1.1 Stated by the user
| # | Requirement |
|---|---|
| R1 | Optimize the design: check the critical paths, simplify logic, focus on `input_packer` / `bit_packer`. |
| R2 | Image TKEEP semantics become **position-based**: pixel i is lane i % K of beat i / K; a TKEEP = 0 lane is stored as a 0 pixel in place (no compaction). |
| R3 | Scope: config path, `input_packer` / `bit_packer`, ping-pong read mux. The neuron datapath (`neuron_processor`, `popcount`, `bnn_layer` sequencer) is **not** changed. |
| R4 | Weight writers use a **neuron staging register**; `bit_packer` is deleted from the design. |
| R5 | The provided TB (`verification/bnn_fcc_tb.sv`, `bnn_fcc_tb_pkg.sv`) and `sim/cfg` wrappers are not edited. |
| R6 | Allowed edits beyond RTL: our unit TBs in `verification/unit/`; source lists in `openflex/bnn_fcc_timing.yml` / `bnn_fcc_verify.yml`; `sim/README.md` "Latest results" and (one-off exception) removing `tb_bit_packer` from its "Unit testbenches" line. |

### 1.2 Assumptions (stated during brainstorming, not corrected)
- Goal is fMax: the use case is maximum throughput, and cycles/image are fixed at 98–99 by the 64-bit input bus. Area is secondary but must not grow.
- Judged defaults unchanged: PI 66, PN {32,3,1}.
- Config-load time is not a metric; extra config-path cycles are acceptable.
- Success: openflex fMax up, LUT not up; lint, all unit TBs, 11/11 regressions and `tb_mnist` 100/100 pass; `sfc_perf` throughput unchanged.

### 1.3 Facts from the baseline build (routed, 1.0 ns constraint)
| Path class | Worst slack | LUT levels |
|---|---|---|
| cfg: serializer `keep_q` → layer-writer `bit_packer.buf_q` | −1.357 | 6 |
| cfg: parser `layer_q` → writer packer / weight-RAM WE / parser `hdr_q`, `rem_q` | −1.340 | 5–6 |
| data: layer-0 ping-pong `rd_data` (fanout 64) → popcount stage 1 | −1.298 | 4 |
| cfg: writer packer `cnt_q` → weight-RAM WE | −1.268 | 3 |
| data: `rd_bank` → layer-0 ping-pong read mux | −1.253 | 4 |
| cfg: serializer → threshold clamp | −1.250 | 6 (1 CARRY8) |
| image: `accept_new_q` → `input_packer` `bit_packer.buf_q` | −1.231 | 5 |

Per-module LUTs: `bit_packer` instances 306 + 717 + 712 (writers) + 294 (image) = 2,029; `input_packer` total 365; layer-0 `pingpong_buf` 2,848 LUTs / 1,659 FFs, with 1,641 of its flops D-driven by a LUT (write enables folded into feedback muxes).

### 1.4 Trade-offs accepted
1. Mid-image TKEEP holes are no longer compacted (R2). The provided TB never produces them (`bnn_fcc_tb.sv:380-388` clears TKEEP only on trailing lanes of the final beat).
2. fMax is expected to be capped near the −1.298 ns popcount path (≈ 435 MHz) because the datapath is out of scope (R3). The main gains are area and simpler code.
3. Writers gain ≈ 1,296 staging FFs (784 + 256 + 256) and lose ≈ 632 packer FFs.
4. Config load is a few cycles per neuron slower (input and write-port registers); for PW < 8 the writer stalls PASSES cycles per neuron.

---

## 2. Superseded sections
| Earlier spec section | Replaced by |
|---|---|
| 2026-10-10 module table rows `bit_packer.sv`, `input_packer.sv` (TKEEP-compacted, bounded bits → bit_packer) | §3.3, §3.4 |
| 2026-10-10 input-packer rule "pixel j valid iff all its TKEEP bits are 1" (compaction) | §3.4 |
| 2026-10-10 configuration-writer description (bytes through `bit_packer`, 8 → PW, pad 1) | §3.2 |
| 2026-10-10 `tb_input_packer` row ("TKEEP holes" compaction) and `tb_bit_packer` | §4 |

All other sections stand.

---

## 3. Design

### 3.1 `cfg_parser`
- New output `msg_sel[L-1:0]`: registered one-hot layer select, loaded on the WAIT → PAYLOAD transition from `h_layer` (bit l set iff `h_layer == l`; a valid header guarantees exactly one bit), held until the next transition.
- The loaded-flag updates (`w_loaded`, `t_loaded`) index `msg_sel` instead of comparing `layer_q`.
- `msg_layer` is removed (the top uses `msg_sel` as `cw_sel`). Its only other user is `tb_cfg_parser`, which is updated.
- All other behaviour (header collection, validation, WAIT ignoring its first cycle, PAYLOAD / DISCARD, abort, `drop_rest`) is unchanged.

### 3.2 `cfg_layer_writer` (rewritten; port list unchanged)
Derived constants as today: `GROUPS`, `PASSES`, `BPN = cdiv(F, 8)`, `THR_W`, `LANE_W`, `WADDR_W`, `TADDR_W`. New: `localparam bit OVERLAP = (PW >= 8)`.

**Message type.** `mt_q` latched on `start`; used everywhere (no `start ? msg_type : mt_q` bypass). The parser never presents a byte in the `start` cycle.

**Input register.** On `s_valid && s_ready`: `in_v_q <= 1`, `in_byte_q <= s_byte`, plus for weights the byte index `in_bn_q` (byte counter value at accept) and `in_last_q` (`bn == BPN-1`). All weight/threshold processing starts from these flops one cycle after the accept. `clr = start || abort` clears it.

**Weights: staging.** `stage[F-1:0]`. When `in_v_q && !mt_q`: `stage[8*in_bn_q + i] <= in_byte_q[i]` for every `i` with `8*in_bn_q + i < F` (byte-index enable decode; bits ≥ F of the last byte are not stored).

**Weights: emission.** When the registered last byte is written (`in_v_q && in_last_q && !mt_q`), emission starts next cycle: `em_q = 1`, pass `p_q = 0 .. PASSES-1`, one word per cycle:
- word = `{PASSES*PW - F {1'b1}, stage}[p_q*PW +: PW]` (pad bits are the constant 1),
- address `waddr_q` (= group base + p), lane `lane_q`; after the last pass, `lane_q` / group base advance exactly as today (lane wraps at PN, group base += PASSES).
- Write port registered: `w_we`, `w_lane`, `w_addr`, `w_data` are flops loaded from the emission state.

**Overlap rule.**
- `OVERLAP` (PW ≥ 8): emission runs concurrently with the next neuron's bytes. Word p is read at emission cycle E + p; the next neuron's byte b is written no earlier than the end of cycle E + b. Byte b covers bits [8b, 8b+8), word p covers [p·PW, (p+1)·PW); for b < p, 8b + 8 ≤ 8p ≤ p·PW, so they never overlap. PASSES ≤ BPN when PW ≥ 8, so an emission always ends before the next neuron's last byte is written. `s_ready = 1`.
- `!OVERLAP` (PW < 8): `s_ready = !(in_v_q && in_last_q) && !em_q` for weights (registered state only); no byte is accepted from the cycle after a neuron's last byte until its emission ends.
- Thresholds: `s_ready = 1`.

**Thresholds.** Same as today but fed from `in_byte_q`: 4 bytes little-endian shifted into `tsh_q`; on the 4th registered byte the clamped value (`< 0` → 0, `> F+1` → F+1) is written via registered `t_we`, `t_lane`, `t_addr`, `t_data` (one cycle later than today). `HAS_THR = 0` keeps the tie-offs.

**Idle.** `idle` is low from the cycle after `start` until the message's last write, high from the cycle after it: weights — the registered `w_we` of the last pass of neuron N-1 (lane `(N-1) % PN`, group `GROUPS-1`); thresholds — the registered `t_we` of neuron N-1. `abort` and `rst` force it high and clear all state.

### 3.3 `bit_packer`
Deleted (`rtl/bit_packer.sv`, `verification/unit/tb_bit_packer.sv`, the two openflex source-list lines).

### 3.4 `input_packer` (rewritten)
Parameters `DATA_W`, `BUS_W`, `F0` (`PW0` removed). `K = BUS_W / DATA_W`, `KB = DATA_W / 8`, `NCH = cdiv(F0, K)`.
- Beat bits: `c_bits[j] = s_data[j*DATA_W + DATA_W-1] & (&s_keep[j*KB +: KB])`, j = 0..K-1 (position-based; a TKEEP = 0 lane is a 0 pixel).
- `gate = mid_q || (accept_new && !buf_full[bank_q])`; `s_ready = gate`; `accept = s_valid && gate`. One beat per cycle, no flush bubble; `s_ready` depends on registered state only.
- Write: `wr_en = accept && !ovf_q`, `wr_bank = bank_q`, `wr_idx = idx_q` (`bits(NCH)` wide), `wr_data = c_bits` (K bits). On `wr_en`: `idx_q++`; `ovf_q <= 1` when `idx_q == NCH-1`.
- Commit on the TLAST beat: `commit_en = accept && s_last`, `commit_bank = bank_q`. On commit: `bank_q` toggles, `idx_q`, `ovf_q`, `mid_q` clear. Otherwise `accept` sets `mid_q`. `mid_image = mid_q`.
- Overlong images: beats after chunk NCH-1 are dropped (`ovf_q`); bits ≥ F0 of the last chunk are not stored by the ping-pong (NBITS = F0). Short images (early TLAST) commit with stale tail bits, as today (images are assumed correctly sized).

### 3.5 `bnn_fcc` (top)
- `u_in`: drop `.PW0`; `ip_wr_idx` is `bits(cdiv(F0, K))` wide, `ip_wr_data` is K bits (`K = INPUT_BUS_WIDTH / INPUT_DATA_WIDTH`).
- `g_layer[0]`: ping-pong `WR_W = K` (was PW0). Layers ≥ 1 unchanged.
- `cw_sel = msg_sel` (from the parser); `msg_layer` wiring removed.
- `pipeline_idle_q` additionally requires `&cw_idle`, so the parser's WAIT holds a new message until every writer has finished its last write (needed for long emission tails at small PW, e.g. PI = 1: 784 cycles; ≤ 14 cycles in the judged config). Header comment updated.

### 3.6 `pingpong_buf` (behaviour unchanged)
- Read: each bank viewed as `words[b][r]`, r = 0 .. 2**bits(NRD) - 1, RD_W bits; bits ≥ NBITS and rows ≥ NRD are constant 0; `rd_nxt = words[rd_bank][rd_idx]` (plain mux replacing the priority chain). `rd_data` stays registered; read data stays independent of the full flags.
- Write: one shared enable vector `we[b][c] = wr_en && wr_bank == b && wr_idx == c` (2 × NWR decodes); bit j of bank b loads `wr_data[j % WR_W]` when `we[b][j / WR_W]`. If Vivado still folds the enables into D-pin feedback muxes, apply `(* direct_enable = "yes" *)` to `we`.
- Each of the two changes is kept only if the measured LUTs and slack improve over the baseline structure.
- Full flags and the commit-wins rule unchanged.

---

## 4. Verification

Gates after every implementation step: `sim/lint.sh` clean; `sim/run_unit.sh` all PASS; `sim/run_regress.sh` 11/11 PASS; `tb_mnist` 100/100 (0.8 valid, random ready); `sfc_perf` 100.5 cycles/output, latency reported (expected ≤ 249).

| Unit TB | Change |
|---|---|
| `tb_bit_packer` | Deleted with the module. |
| `tb_input_packer` | Rewritten: position rule incl. a mid-beat TKEEP = 0 lane stored as 0; trailing partial beat with F0 % K ≠ 0; overlong clip; commit on the TLAST beat; single-beat image (F0 ≤ K); back-to-back images; `accept_new` / `buf_full` gating; random valid gaps. |
| `tb_cfg_layer_writer` | Updated: registered write-port timing; PASSES = 1 and > 1; F % 8 ≠ 0; pad bits = 1; OVERLAP (PW ≥ 8, `s_ready` constant 1, back-to-back neurons) and stall (PW < 8) cases; `idle` timing; abort mid-neuron. |
| `tb_cfg_parser` | Updated: `msg_sel` one-hot; WAIT holds while `pipeline_idle` (now including writer idle) is low. |
| `tb_pingpong_buf` | Extended with the new layer-0 shape (NBITS 784, WR_W 8, RD_W 66). |
| `tb_cfg_sequences`, `tb_mnist`, others | Expected to pass unchanged. |

Provided TB, `bnn_fcc_tb_pkg.sv` and `sim/cfg` wrappers: not edited. Sim scripts glob `rtl/*.sv` and `verification/unit/tb_*.sv`; no change.

## 5. Implementation order
1. `pingpong_buf` read/write rewrite (behaviour-preserving, isolated).
2. `input_packer` + top wiring for the layer-0 ping-pong.
3. `cfg_parser` `msg_sel`, `cfg_layer_writer` rewrite, top `cw_sel` / `pipeline_idle_q`.
4. Delete `bit_packer` (RTL, unit TB, openflex lists).
5. openflex synthesis.

Each step passes the §4 gates before the next starts.

## 6. Measurement and reporting
- openflex (`bnn_fcc_timing.yml`, XCU250-FIGD2104-2L-E) against the baseline: fMax, LUT (logic / mem), REG, CARRY8, BRAM.
- Hierarchical utilization (per module) before / after.
- Critical-path class table (§1.3) before / after.
- Single-run fMax is noisy by several percent; reported as such.
- `sim/README.md`: refresh "Latest results"; remove `tb_bit_packer` from the "Unit testbenches" line (one-off exception, R6).
