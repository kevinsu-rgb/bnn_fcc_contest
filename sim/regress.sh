#!/usr/bin/env bash
# Run every line of sim/regress.list (<env> <pset> <test>) with sim/run_unit.sh,
# then merge all functional-coverage databases into sim/cov_report/.
#
#   [STUB=1] sim/regress.sh [list-file] [seed]
#
# Prints one RESULT line per test and finally
#   REGRESS: <p> passed, <f> failed
# Exit status is 1 if any test failed.

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
LIST=${1:-$ROOT/sim/regress.list}
SEED=${2:-1}
REPORT=$ROOT/sim/cov_report

if [ ! -f "$LIST" ]; then
  echo "regress list not found: $LIST" >&2
  exit 2
fi

if ! command -v xcrg >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source "${VIVADO_SETTINGS:-/tools/AMD/2025.2/Vivado/settings64.sh}" >/dev/null 2>&1
fi

# Drop stale coverage so the report only reflects this run.
find "$ROOT/sim/build" -name xsim.covdb -type d -prune -exec rm -rf {} + 2>/dev/null
rm -rf "$REPORT"

pass=0
fail=0
while read -r env pset test _rest; do
  case "$env" in ''|'#'*) continue ;; esac
  out=$("$ROOT/sim/run_unit.sh" "$env" "$pset" "$test" "$SEED" 2>&1)
  rc=$?
  line=$(printf '%s\n' "$out" | tail -n 1)
  echo "$line"
  if [ "$rc" -eq 0 ]; then pass=$((pass + 1)); else fail=$((fail + 1)); fi
done < "$LIST"

# ---- coverage merge -------------------------------------------------------
mapfile -t COVDIRS < <(find "$ROOT/sim/build" -name xsim.covdb -type d 2>/dev/null | sort)
if [ ${#COVDIRS[@]} -gt 0 ] && command -v xcrg >/dev/null 2>&1; then
  mkdir -p "$REPORT"
  ARGS=()
  for d in "${COVDIRS[@]}"; do ARGS+=(-cov_db_dir "$(dirname "$d")"); done
  (cd "$REPORT" && xcrg "${ARGS[@]}" -merge_dir "$REPORT" -merge_db_name regress \
      -report_dir "$REPORT" -report_format all -log "$REPORT/xcrg.log" > "$REPORT/xcrg.stdout" 2>&1) \
    && echo "coverage: merged ${#COVDIRS[@]} database(s) into sim/cov_report/" \
    || echo "coverage: xcrg failed, see sim/cov_report/xcrg.stdout"
else
  echo "coverage: no functional coverage databases found"
fi

echo "REGRESS: $pass passed, $fail failed"
[ "$fail" -eq 0 ]
