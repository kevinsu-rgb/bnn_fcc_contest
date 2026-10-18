#!/usr/bin/env bash
# Lint one RTL top with Verilator: sim/lint.sh <top>
# Prints nothing and exits 0 when clean.
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 2
if [ $# -ne 1 ]; then
  echo "usage: sim/lint.sh <top>" >&2
  exit 2
fi
FILES=()
while IFS= read -r line || [ -n "$line" ]; do
  line=$(echo "${line%%#*}" | tr -d '\r' | xargs)
  [ -n "$line" ] && FILES+=("$line")
done < synth/files.f
verilator --lint-only -Wall -Irtl --top-module "$1" "${FILES[@]}"
