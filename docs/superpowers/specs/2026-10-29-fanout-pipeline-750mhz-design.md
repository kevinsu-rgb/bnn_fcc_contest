# Fanout Localization and Datapath Pipelining (750 MHz) — Design Spec

Date: 2026-10-29
Status: Draft (awaiting user review)
Branch: `feature/bnn-fcc-750mhz` (from `765192a`)
Amends: `2026-10-10-bnn-fcc-design.md`, `2026-10-18-parallel-inputs-first-layer-design.md`, `2026-10-28-packer-removal-config-pipeline-design.md` (all left unchanged; this spec supersedes the parts named in §2).
Baseline: commit `765192a` — 554.94 MHz (WNS −0.802 ns at the 1.0 ns constraint), LUT 19,275 (11,871 logic / 7,404 mem), REG 11,104, CARRY8 210, BRAM 0; PI 66, PN {32,3,1}, `RAM_STYLE` "distributed", `POPCOUNT_LEVELS_PER_STAGE` 3.

---

## 1. Intent and agreed requirements

### 1.1 Stated by the user
| # | Requirement |
|---|---|
| R1 | Optimize the current design from its critical paths. |
| R2 | Target about **750 MHz** (option B): fix all three path families systematically (broadcast fanout, popcount depth, config control). |
| R3 | Approach 1: localize every hop with RTL pipelining and register replication (not attribute-only, not a PI/PN change). |
| R4 | Work on a new branch from the current HEAD (`feature/bnn-fcc-750mhz`). |
| R5 | Standing rules: RTL edits only. The provided TB (`verification/bnn_fcc_tb.sv`, `bnn_fcc_tb_pkg.sv`, `axi4_stream_if.sv`), `sim/cfg` wrappers, provided READMEs and `doc/` are not edited. Allowed besides RTL: our unit TBs in `verification/unit/`, openflex source lists / PARALLEL_* defaults, the `sim/README.md` "Latest results" section. |

### 1.2 Assumptions (stated during brainstorming, confirmed)
- `neuron_processor`, `popcount` and `bnn_layer` are in scope.
- Register count may grow (thousands of FFs).
- Extra latency cycles are acceptable as long as steady-state II stays at 99 (§1.4).
- PI/PN changes are allowed as a lever but are not part of this spec (a later sweep).
- Use case is maximum throughput: throughput = fMax / 99 images/s, so fMax is the only lever. No XDC is ours to change, so every fix is structural.

### 1.3 Facts from the baseline routed build
Worst path per failing endpoint (60,314 of 74,745 endpoints fail at 1.0 ns). Endpoints that would still fail at a given target:

| Target | 600 | 650 | 700 | 750 | 800 MHz |
|---|---|---|---|---|---|
| Failing endpoints | 775 | 8,956 | 24,003 | 35,543 | 42,906 |

By family at 700 MHz (mostly layer 0, 32 lanes × 66 bits):

| Family | Endpoints @700 | Typical levels | Worst slack |
|---|---|---|---|
| Config write bus (`w_addr`/`w_data`/`w_we`/`w_lane`, `t_*`) → `wmem`/`tmem` | 16,960 | 0–2 | −0.732 |
| Weight read address `waddr_r` → `w_rd` | 2,797 | 1–2 | −0.719 |
| Ping-pong write enables (`full`, `accept_new_q`, `idx_q`, `data_in_*`, `o_y`) → bank bits | 1,435 | 0–2 | −0.732 |
| Operand `rd_data` / `w_rd` → popcount stage 1; `f_d`/`vpipe` → `acc`/`out_y` | ~1,000 | 3–5 | −0.783 |
| Ping-pong read mux (`rd_bank`, `p_r`, `mem` → `rd_data`) | ~400 | 1–3 | −0.760 |
| Config control (serializer `keep_q`, parser `rem_q` → writer / parser state) | ~1,250 | 4–5 | −0.743 |
| Popcount tree inside (layer 1: three 9-bit adds, 2 CARRY8) | ~125 | 4–6 | **−0.802** |

0-level paths fail by about −0.7 ns (≈1.5 ns of route): the cause is distance and fanout, not logic depth. Pulse-width check of the distributed weight RAM fails by 0.064 ns at 1.0 ns, so LUTRAM caps fMax at ≈940 MHz.

### 1.4 Latency budget (measured)
Artificial delay on the judged configuration (scratch RTL copy, provided TB, 100 images, no stalls):

| Extra cycles | TB cycles/output | Steady-state II |
|---|---|---|
| none | 101.1 (latency 310) | 99 |
| each hidden layer +8 (issue → output commit) | 101.3 | 99 |
| each hidden layer +12 | 101.8 | ≈99.4 (lost) |
| input → layer-0 buffer +4 | 101.2 | 99 |
| input → layer-0 buffer +6 | 102.2 | ≈100 (lost) |

**Budget: ≤ 8 extra cycles per hidden layer (issue to next-buffer commit) and ≤ 4 on the image input path.** The output layer has no budget limit.

### 1.5 Trade-offs accepted
- REG grows by about 5k (operand XNOR registers, address and operand copies, write tree, one-hot pointers).
- Latency grows by about 12 cycles (310 → ≈322 in the judged configuration); in ns it drops (559 → ≈429 ns at 750 MHz).
- Config writes land 2 cycles later and an abort reaches the writer 1 cycle later (config-load time is not a metric).

---

## 2. Superseded sections
- `pingpong_buf` interface and read contract (2026-10-10 spec, ping-pong section; 2026-10-28 §3.6): write index → one-hot, registered write port (port-level `full` timing unchanged), rotating read with 2-cycle latency, operand copies.
- `bnn_layer` read alignment, NP latency (`np_latency`), weight/threshold write path (2026-10-10 / 2026-10-18 layer sections).
- `neuron_processor` accumulate/compare stage (2026-10-10 NP section).
- `cfg_parser` abort timing and `cfg_layer_writer` input-register contents (2026-10-28 §3.1, §3.2).
- Default `POPCOUNT_LEVELS_PER_STAGE` 3 → 2 (subject to the §6 comparison).

Unchanged: message formats, AXI behaviour at the DUT ports, weight/threshold memory layout, sequencer issue order, argmax, `axis_out`, popcount RTL, TKEEP semantics.

---

## 3. Design

Notation: an image's (g, p) word issues in cycle t (sequencer `issue`). `LG = 8` (lanes per lane group, `bnn_layer` localparam); `NG = cdiv(PN, LG)` lane groups (layer 0: 4; layers 1, 2: 1).

### 3.1 `pingpong_buf`

Parameters: `NBITS`, `WR_W`, `RD_W`, new `RD_COPIES` (default 1). `NWR = cdiv(NBITS, WR_W)`, `NRD = cdiv(NBITS, RD_W)`, `LEN = NRD * RD_W`.

Ports (changes in bold): `wr_en`, `wr_bank`, **`wr_oh [NWR]`** (one-hot chunk select, replaces `wr_idx`), `wr_data [WR_W]`, `commit_en`, `commit_bank`, **`rd_adv`** (replaces `rd_idx`), `rd_bank`, **`rd_data [RD_COPIES][RD_W]`**, `rls_en`, `rls_bank`, `full [2]`.

**Write port (one register stage).** Every cycle the buffer registers `en_q[b] = wr_en && wr_bank == b`, `oh_q <= wr_oh`, `dat_q[b] <= wr_data` (one data copy per bank, `(* keep = "true" *)`), `cm_q <= commit_en`, `cm_bank_q <= commit_bank`. Chunk c of bank b is written when `en_q[b] && oh_q[c]`; bit j of chunk c takes `dat_q[b][j % WR_W]`. The flag register `full_q[cm_bank_q]` sets on `cm_q`; release clears `full_q[rls_bank]` on `rls_en`; commit wins over release on the same bank (unchanged rule, applied to `cm_q`). The `full` output is `full_q | (in-flight commit: cm_q for bank cm_bank_q)`, so `full` at the ports rises in the cycle after `commit_en`, exactly as today. This keeps the producers' `pending`/`out_full` logic and `input_packer`'s gate correct with no timing change (otherwise a bank whose `pending` was cleared by `out_commit_en` would look free for one cycle). Reads stay safe: a consumer that starts in the cycle `full` first shows captures data at the end of the following cycle, after the last write (stored at the end of the `cm_q` cycle).

**Bank storage.** Each bank is `LEN` bits. Bits `[NBITS, LEN)` are pad: reset to 0 by `rst`, never written. Data bits are not reset (unchanged).

**Rotating read (NRD > 1).**
- The buffer registers the advance: `adv_q[b][r] <= rd_adv && rd_bank == b` for every row r of every bank (NRD copies per bank, `(* keep = "true" *)`, each driving the RD_W bits of its row), and `bsel_q <= rd_bank` (RD_COPIES copies, `keep`).
- In a cycle with `adv_q[b]`: every `rd_data` copy loads `bank[b][RD_W-1:0]`, and bank b rotates right by RD_W (`bit i <= bit (i + RD_W) % LEN`).
- Per-bit next state: `we ? dat : bit[(i+RD_W)%LEN]`, enable `we | adv`.
- An image issues exactly GROUPS × PASSES advances on a full bank, i.e. GROUPS full turns, so the bank is aligned again when it is released. Producers only write banks that are not full; banks only rotate while full; writes therefore always land at aligned positions.
- Release safety: `rls_en` comes with the last issue (cycle t). The last rotation lands at the end of t+1. A producer that sees `!full` in t+1 has its write registered at the end of t+1 and stored at the end of t+2. No overlap.

**Read when NRD = 1** (layers ≥ 1 with PW = F): no rotation; one registered advance per bank (`adv_q[b]`), and the `rd_data` copies load `bank[bsel_q]` when it is set.

**Read latency: 2 cycles** — `rd_adv` in cycle t, `rd_data` valid in t+2 (the word read is the one aligned at the start of t+1).

### 3.2 `input_packer`
- `idx_q` becomes a one-hot pointer `oh_q [NCH]` (reset and commit: bit 0 set). Advance on an accepted, non-overflow beat: shift left. Overflow: `ovf_q` sets when an accepted beat writes chunk NCH−1 (`oh_q[NCH-1]`).
- Output `wr_oh [NCH] = oh_q` replaces `wr_idx`. All other ports and the `s_ready`, commit-on-TLAST, TKEEP and overflow rules are unchanged.
- Input → layer-0 buffer cost: 0 cycles (the write register is hidden by the `full` rule in §3.1). Budget 4.

### 3.3 `neuron_processor`
- Input stage: `xn_q <= ~(x ^ w)` (registered, one LUT level; differs per lane so it is never merged). The popcount reads `xn_q`.
- Accumulate stage, no clock enable: `acc <= first ? pc : acc + pc` every cycle. Safe because an image's (g, p) words issue back-to-back; junk between images is overwritten by the next first pass.
- Compare stage: `out_y <= (acc >= thr)` and `out_acc_q <= acc`, one cycle after `acc`; `out_valid` aligned with them.
- `f_d` (first-pass delay line) is `(* keep = "true" *)` so each lane keeps its own copy.
- Side-band (`first`, `last`, `thr`, valid) delayed to match. `np_latency(w, lps) = popcount_latency(w, lps) + 3` (XNOR, accumulate, compare).

### 3.4 `popcount`
RTL unchanged. Default `POPCOUNT_LEVELS_PER_STAGE` becomes **2** (layer 0: 2 → 3 stages; layers 1, 2: 3 → 4). Value 1 is evaluated in §6 (layer 0 +2, layer 1 +3 more cycles: still within budget).

### 3.5 `bnn_layer`

**Read side.**
- `in_rd_idx` → `in_rd_adv` (= `issue`). `in_rd_data` → `[NG][PW]`; lane k uses copy `k / LG`.
- Per-lane address copies: `la_q[k] <= waddr_r`, `lg_q[k] <= g_r` every cycle (`(* keep = "true" *)`). Weight read `w_rd <= wmem[la_q]`, threshold read `t_rd <= tmem[lg_q]`. Both valid in t+2, aligned with `in_rd_data`.
- Side-band (`first`, `last`, valid pipe) gets one more stage so the NP inputs are aligned in t+2.

**Write side (config write tree).** Inputs from the writer unchanged (`w_we`, `w_lane`, `w_addr`, `w_data`, `t_we`, `t_lane`, `t_addr`, `t_data`).
- Level 1, per lane group gi: `gw_q[gi] <= w_we && w_lane / LG == gi`; `{glane_q, gaddr_q, gdata_q}[gi]` load when that condition holds. Same for `t_*` (`gt_q`, ...).
- Level 2, per lane k (group gi = k / LG): `lw_q[k] <= gw_q[gi] && glane_q[gi] == k % LG`; `laddr_q[k]` loads under the same condition. Data stays at group level (`gdata_q[gi]`). Same for thresholds.
- `wmem[laddr_q[k]] <= gdata_q[gi]` on `lw_q[k]`; likewise `tmem`.
- Copies load under different enables, so they are not equivalent and need no `keep`.
- New output `cfg_wr_busy = |gw_q | |gt_q | |lw_q | |lt_q`.

**Output side.** The output group counter `g_o` becomes a one-hot pointer `oh_o [GROUPS]`; `last_o = oh_o[GROUPS-1]`. `out_wr_idx` → `out_wr_oh [GROUPS]`. Everything else in the result stage is unchanged.

**Added latency (judged configuration).**

| | Read (2 cycles) | XNOR | Popcount (LPS 2) | Compare | Total | Budget |
|---|---|---|---|---|---|---|
| Input → layer 0 | — | — | — | — | 0 | 4 |
| Layer 0 | +1 | +1 | +1 | +1 | +4 | 8 |
| Layer 1 | +1 | +1 | +1 | +1 | +4 | 8 |
| Layer 2 (output) | +1 | +1 | +1 | +1 | +4 | none |

The next-buffer write register costs no handoff time (§3.1 `full` rule). Expected latency ≈ 322 cycles (judged), II 99. LPS 1 would make layers 0/1 +6/+7, still within budget.

### 3.6 `cfg_byte_serializer`
`m_byte`, `m_null`, `m_end_of_beat` and `m_beat_last` become registers loaded with the values of the next state (load from `s_keep`/`s_data`/`s_last`, or advance from `keep_rest`). Output values and handshake timing are identical to today's combinational outputs; `drop_rest` behaviour unchanged.

### 3.7 `cfg_parser`
- `msg_abort` is registered: it pulses one cycle after the handshake that carried the truncating TLAST. Safe: the parser returns to HEADER and forwards no payload bytes before the next `msg_start`, which needs 16 header bytes plus ≥ 2 WAIT cycles.
- Last-byte flag `last_q`: loaded with `h_total == 1` when `rem_q` is loaded, with `rem_q == 2` on each decrement. `last_cnt` (used by `done_now`, `pl_valid`) is `last_q`.
- Everything else unchanged.

### 3.8 `cfg_layer_writer`
- Input register holds `{in_v_q, in_byte_q}` only; `in_bn_q` and `in_last_q` are removed.
- `bn_q` (byte within neuron) advances on `in_v_q && !mt_q` (wrapping at BPN−1); the staging write uses `bn_q`; `em_start = in_v_q && !mt_q && bn_q == BPN-1`.
- `s_ready` (PW < 8): `mt_q || OVERLAP || (!(in_v_q && !mt_q && bn_q == BPN-1) && !em_q)` — still registered state only.
- Abort arrives one cycle later (§3.7); behaviour otherwise unchanged.

### 3.9 `bnn_fcc` (top)
- Wire the new ports (`wr_oh`, `rd_adv`, `rd_data` copies, `out_wr_oh`, `cfg_wr_busy`); layer `l` sets `RD_COPIES = cdiv(PN_L, 8)`.
- Writer idle term becomes `&cw_idle && !(|cfg_wr_busy)` in both `accept_new_q` and `pipeline_idle_q`.
- Default `POPCOUNT_LEVELS_PER_STAGE = 2` (final value from §6).

---

## 4. Verification

New checks are written first and must fail on the old RTL (where the old RTL can express the case).

| TB (ours) | Change |
|---|---|
| `tb_pingpong_buf` | Rewritten for the new contract: one-hot registered write; `full` rises the cycle after `commit_en` (as before) and a read started in that cycle returns the last-written data; rotating read (2-cycle latency); all `RD_COPIES` equal; pad bits read 0. Cases: NRD > 1 and NRD = 1; many back-to-back images where the next image's first advance is the cycle after the previous last; a write immediately after release (checks alignment after GROUPS turns); commit and release on the same bank. |
| `tb_input_packer` | One-hot pointer instead of `wr_idx`; content check one cycle later (writes land one cycle after the handshake); `full` timing unchanged; overflow at `oh_q[NCH-1]`. |
| `tb_neuron_processor` | New `np_latency`; idle cycles between images must not corrupt results (free-running `acc`); `out_acc` checked in the compare stage. |
| `tb_bnn_layer` | `in_rd_adv`/copies and 2-cycle read; `out_wr_oh`; write tree: writes land 2 cycles later; `cfg_wr_busy` covers every in-flight write. |
| `tb_cfg_parser` | `msg_abort` one cycle later; `last_q` with `total_bytes` 1 and 2 and with a null-item TLAST on the last byte. |
| `tb_cfg_layer_writer` | Delayed abort; byte counting from the input register; existing PW < 8 and PW ≥ 8 cases. |
| `tb_cfg_byte_serializer` | **Unchanged**; must pass as is (proves identical output timing). |
| `tb_cfg_sequences` | New case: config load immediately followed by an image (image must wait for the write tree to drain). |
| `tb_popcount`, `tb_argmax`, `tb_axis_out` | Unchanged except `popcount_latency` expectations if the default changes. |
| `tb_mnist` | 100/100. |

Provided TB: `sim/run_regress.sh` 11/11 PASS, files unchanged. Lint: `sim/lint.sh` clean.

**Throughput gate.** Scratch wrapper `judged_perf` (copy of `sim/cfg/sfc_perf.sv` with PN {32,3,1}, kept outside the repo) must show steady-state II 99: TB cycles/output ≤ 101.3 at latency ≈ 322. `sfc_perf` stays at 100.5 cycles/output (new latency reported). Over budget → first lower popcount stages (LPS 1 → 2 if 1 was chosen), then revisit the compare stage.

---

## 5. Implementation order
1. **Copies survive synthesis.** Small OOC synthesis of `pingpong_buf` (784, 8, 66, RD_COPIES 4) and a `bnn_layer` lane: count replicated registers (`adv_q`, `dat_q`, `bsel_q`, `la_q`, `f_d`) in the netlist. If `keep` does not prevent merging, switch to `dont_touch` before continuing.
2. `neuron_processor` (XNOR register, free-running `acc`, compare stage, `keep` on `f_d`) and `np_latency`; LPS default 2.
3. `pingpong_buf`, `input_packer`, `bnn_layer` read side and output one-hot, top wiring (§3.1, 3.2, 3.5 read/output, 3.9) — one task because the interfaces change together.
4. Config write tree and `cfg_wr_busy` (§3.5 write side, 3.9).
5. Serializer, parser, writer (§3.6–3.8).
6. Synthesis, LPS 2 vs 1 comparison, README.

---

## 6. Measurement and reporting
- Full openflex runs after task 3 and at the end; each with the per-class slack histogram (worst path per failing endpoint, grouped by family as in §1.3).
- Final comparison `POPCOUNT_LEVELS_PER_STAGE` 2 vs 1; keep the better fMax that also passes the throughput gate.
- Success: fMax ≥ 750 MHz (reported as measured, single-run noise ≈ ±5%); no §1.3 family left as the limiter without an explanation; LUT ≤ 19,275 (± noise); REG growth reported; II 99; all tests green.
- Out of scope: XDC, provided TB/package/wrappers, `doc/`, PI/PN changes, the 900 MHz architecture.
- Report: new bullet in `sim/README.md` "Latest results" with fMax, LUT/REG/CARRY8, per-family slack, latency (judged and `sfc_perf`), throughput.
