#!/usr/bin/env bash
# Usage: sim/run_regress.sh [cfg_name ...]   (default: every sim/cfg/*.sv)
# Runs the provided bnn_fcc_tb through the sim/cfg/<name>.sv wrapper (top cfg_<name>).
# Exit code = number of failing configurations.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"

cfgs=("$@")
if [ ${#cfgs[@]} -eq 0 ]; then
  for f in "$REPO"/sim/cfg/*.sv; do
    [ -e "$f" ] && cfgs+=("$(basename "$f" .sv)")
  done
fi

fails=0
for c in "${cfgs[@]}"; do
  run_xsim "$c" "cfg_$c" -i "$REPO/sim/cfg" $RTL_FILES $TBPKG_FILES \
    "$REPO/verification/bnn_fcc_tb.sv" "$REPO/sim/cfg/$c.sv" || fails=$((fails+1))
  if [ "$c" = "sfc_perf" ]; then
    # the TB's own latency line is always 0.0 under xsim (real-queue sum()); use the xsim_tb_fix monitor's
    grep -E 'xsim_tb_fix: Avg latency \(cycles\)|Avg throughput \(cycles/output\)' "$REPO/sim/build/$c/run.log"
  fi
done
exit $fails
