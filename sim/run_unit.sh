#!/usr/bin/env bash
# Run one UVM unit test under Vivado xsim.
#
#   [STUB=1] sim/run_unit.sh <env> <pset> <test> [seed]
#
# Compiles   <pset.svh> <deps.f> <dut.f | stub/*.sv> <tb.f>   (paths relative to
# the repo root), elaborates <env>_tb and runs it with UVM_TESTNAME=<test>.
# The last line printed is always
#   RESULT: PASS env=<env> pset=<pset> test=<test> seed=<seed>               (exit 0)
#   RESULT: FAIL reason=<compile|elab|timeout|scoreboard> env=... (exit 1)
# Usage errors exit 2.
#
# Environment:
#   STUB=1          compile verification/unit/<env>/stub/*.sv instead of dut.f
#                   (stub dir first on the include path, rtl/ not on it)
#   SIM_TIMEOUT=<s> wall-clock limit for xsim (default 1800); expiry => timeout
#   VIVADO_SETTINGS settings64.sh to source if xvlog is not on PATH

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ $# -lt 3 ] || [ $# -gt 4 ]; then
  echo "usage: [STUB=1] sim/run_unit.sh <env> <pset> <test> [seed]" >&2
  exit 2
fi
ENV_NAME=$1
PSET=$2
TEST=$3
SEED=${4:-1}
STUB=${STUB:-0}
SIM_TIMEOUT=${SIM_TIMEOUT:-1800}

UNIT=$ROOT/verification/unit/$ENV_NAME
BUILD=$ROOT/sim/build/$ENV_NAME/$PSET
TAG="env=$ENV_NAME pset=$PSET test=$TEST seed=$SEED"

if [ ! -d "$UNIT" ]; then
  echo "unknown env '$ENV_NAME' ($UNIT does not exist)" >&2
  exit 2
fi

fail() {  # fail <reason> [detail]
  echo "RESULT: FAIL reason=$1 $TAG${2:+ detail=$2}"
  exit 1
}

if ! command -v xvlog >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source "${VIVADO_SETTINGS:-/tools/AMD/2025.2/Vivado/settings64.sh}" >/dev/null 2>&1
fi
command -v xvlog >/dev/null 2>&1 || fail compile "xvlog-not-found"

mkdir -p "$BUILD"
cd "$BUILD" || exit 2
rm -rf xsim.dir "$TEST".* xvlog.* xelab.* *.pb *.jou 2>/dev/null

# ---- assemble the compile list ------------------------------------------
read_list() {  # print the entries of a .f file as absolute paths
  local f=$1 line
  [ -f "$f" ] || return 0
  while IFS= read -r line || [ -n "$line" ]; do
    line=${line%%#*}
    line=$(echo "$line" | tr -d '\r' | xargs)
    [ -n "$line" ] && echo "$ROOT/$line"
  done < "$f"
}

PSET_FILE=$UNIT/psets/$PSET.svh
[ -f "$PSET_FILE" ] || fail compile "missing-pset-file:$PSET_FILE"

FILES=("$PSET_FILE")
mapfile -t DEPS < <(read_list "$UNIT/deps.f")
FILES+=("${DEPS[@]}")
if [ "$STUB" = "1" ]; then
  mapfile -t DUT < <(compgen -G "$UNIT/stub/*.sv" | sort)
  [ ${#DUT[@]} -gt 0 ] || fail compile "no-stub-files:$UNIT/stub"
  INCS=(-i "$UNIT/stub" -i "$ROOT/verification/unit/common" -i "$UNIT")
else
  mapfile -t DUT < <(read_list "$UNIT/dut.f")
  INCS=(-i "$ROOT/rtl" -i "$ROOT/verification/unit/common" -i "$UNIT")
fi
FILES+=("${DUT[@]}")
mapfile -t TBF < <(read_list "$UNIT/tb.f")
FILES+=("${TBF[@]}")

for f in "${FILES[@]}"; do
  [ -f "$f" ] || fail compile "missing-file:${f#"$ROOT"/}"
done

# ---- compile -------------------------------------------------------------
if ! xvlog -sv -L uvm "${INCS[@]}" "${FILES[@]}" > "$TEST.xvlog.log" 2>&1 \
   || grep -q '^ERROR:' "$TEST.xvlog.log"; then
  grep -m5 '^ERROR:' "$TEST.xvlog.log"
  fail compile "log=${BUILD#"$ROOT"/}/$TEST.xvlog.log"
fi

# ---- elaborate -----------------------------------------------------------
if ! xelab -L uvm -timescale 1ns/1ps "${ENV_NAME}_tb" -s snap > "$TEST.xelab.log" 2>&1 \
   || grep -q '^ERROR:' "$TEST.xelab.log"; then
  grep -m5 '^ERROR:' "$TEST.xelab.log"
  fail elab "log=${BUILD#"$ROOT"/}/$TEST.xelab.log"
fi

# ---- simulate ------------------------------------------------------------
LOG=$BUILD/$TEST.log
timeout "$SIM_TIMEOUT" xsim snap -R -sv_seed "$SEED" \
    -testplusarg "UVM_TESTNAME=$TEST" \
    -testplusarg "BASE_DIR=$ROOT/python" \
    -cov_db_dir "$BUILD" -cov_db_name "${TEST}_${SEED}" \
    -log "$LOG" > "$TEST.xsim.stdout" 2>&1
SIM_RC=$?

echo "--- ${LOG#"$ROOT"/}"
grep -E 'SCB: ' "$LOG"
grep -E 'SCB_MISMATCH|^UVM_(ERROR|FATAL) [^:]' "$LOG" | head -10
grep -E '^UVM_(ERROR|FATAL) :' "$LOG"

if [ "$SIM_RC" -eq 124 ]; then
  fail timeout "wallclock-${SIM_TIMEOUT}s"
fi
if grep -q 'UVM_FATAL .*\[WATCHDOG\]' "$LOG" 2>/dev/null; then
  fail timeout "watchdog"
fi

if [ -f "$LOG" ] \
   && grep -Eq '^UVM_ERROR :[[:space:]]+0$' "$LOG" \
   && grep -Eq '^UVM_FATAL :[[:space:]]+0$' "$LOG" \
   && grep -Eq 'SCB: .* checked=[0-9]+ mismatches=[0-9]+' "$LOG"; then
  echo "RESULT: PASS $TAG"
  exit 0
fi

fail scoreboard "log=${LOG#"$ROOT"/}"
