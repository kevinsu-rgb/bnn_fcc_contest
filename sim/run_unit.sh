#!/usr/bin/env bash
# Usage: sim/run_unit.sh [tb_name ...]   (default: every verification/unit/tb_*.sv)
# Exit code = number of failing testbenches.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"

tbs=("$@")
if [ ${#tbs[@]} -eq 0 ]; then
  for f in "$REPO"/verification/unit/tb_*.sv; do
    [ -e "$f" ] && tbs+=("$(basename "$f" .sv)")
  done
fi

fails=0
for tb in "${tbs[@]}"; do
  file="$REPO/verification/unit/$tb.sv"
  extra=()
  if grep -q 'BASE_DIR' "$file"; then
    extra=(-- -generic_top "BASE_DIR=$REPO/python")
  fi
  run_xsim "$tb" "$tb" $RTL_FILES $TBPKG_FILES "$file" "${extra[@]}" || fails=$((fails+1))
done
exit $fails
