#!/usr/bin/env bash
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"
cd "$REPO" && verilator --lint-only -Wall --top-module bnn_fcc $RTL_FILES
