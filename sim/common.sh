#!/usr/bin/env bash
# Shared helpers for xsim-based simulation. Source this file; do not execute it.
SIM_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SIM_COMMON_DIR/.." && pwd)"

# shellcheck disable=SC1091
# Override the Vivado environment script with VIVADO_SETTINGS=/path/to/settings64.sh
source "${VIVADO_SETTINGS:-/tools/AMD/2025.2/Vivado/settings64.sh}" >/dev/null 2>&1

# Package first, then every other RTL file (arrays so paths with spaces work).
RTL_FILES=("$REPO/rtl/bnn_fcc_pkg.sv")
for _f in "$REPO"/rtl/*.sv; do
  [ "$_f" = "$REPO/rtl/bnn_fcc_pkg.sv" ] && continue
  RTL_FILES+=("$_f")
done
unset _f
TBPKG_FILES=("$REPO/verification/axi4_stream_if.sv" "$REPO/verification/bnn_fcc_tb_pkg.sv")

# run_xsim <name> <top> <files...> [-- <xelab extra args>]
# Builds in sim/build/<name>/, logs to run.log. Returns 0 iff the run passes.
run_xsim() {
  local name="$1" top="$2"
  shift 2
  local files=() xelab_args=()
  while [ $# -gt 0 ]; do
    if [ "$1" = "--" ]; then shift; xelab_args=("$@"); break; fi
    files+=("$1"); shift
  done
  local dir="$REPO/sim/build/$name"
  local log="$dir/run.log"
  rm -rf "$dir"; mkdir -p "$dir"
  local rc=0
  (
    cd "$dir" || exit 1
    xvlog -sv "${files[@]}" || exit 11
    xelab -debug off --timescale 1ns/100ps "${xelab_args[@]}" "$top" -s "$name" || exit 12
    xsim "$name" -R || exit 13
  ) >"$log" 2>&1 || rc=$?
  if [ "$rc" -eq 0 ] \
     && grep -q 'SUCCESS' "$log" \
     && ! grep -qE 'Error:|Fatal:|FAILURE|ERROR:' "$log"; then
    echo "PASS $name"
    return 0
  fi
  echo "FAIL $name ($log)"
  return 1
}
