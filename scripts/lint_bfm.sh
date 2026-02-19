#!/usr/bin/env bash

# ------------------------------------------------------------------------------
# Force C locale to suppress Perl locale warnings inside Verilator
# (No impact on RTL semantics or lint results)
# ------------------------------------------------------------------------------
export LC_ALL=C
export LANG=C

set -euo pipefail

# ------------------------------------------------------------------------------
# Color codes for readable console output
# ------------------------------------------------------------------------------
Red='\033[0;31m'
Blue='\033[0;34m'
NC='\033[0m'

# ------------------------------------------------------------------------------
# Check Verilator availability
# ------------------------------------------------------------------------------
if ! command -v verilator >/dev/null 2>&1; then
  printf "%bERROR:%b verilator not found in PATH\n" "$Red" "$NC"
  exit 1
fi

VERILATOR_PATH=$(command -v verilator)

# ------------------------------------------------------------------------------
# Resolve project paths
#   - SCRIPT_DIR : location of this script
#   - BFM_PATH   : top-level BFM directory
#   - LOG_FILE   : lint output log
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BFM_PATH="$(cd "$SCRIPT_DIR/../bfm" && pwd)"
RTL_PATH="$(cd "$SCRIPT_DIR/../rtl" && pwd)"
LOG_FILE="$SCRIPT_DIR/lint_bfm.log"

if [ ! -d "$BFM_PATH" ]; then
  printf "%bERROR:%b RTL directory not found: %s\n" "$Red" "$NC" "$RTL_PATH"
  exit 1
fi 

printf "%bINFO:%b Verilator found at %s\n" "$Blue" "$NC" "$VERILATOR_PATH"
printf "%bINFO:%b Memory Controller Lint START\n" "$Blue" "$NC"

# ------------------------------------------------------------------------------
# Run lint from RTL root
#   - Required so that relative paths in filelist.f are resolved correctly
# ------------------------------------------------------------------------------
cd "$PROJECT_ROOT"

# ------------------------------------------------------------------------------
# Lint policy note:
#
# The following warnings are intentionally suppressed:
#   - UNUSEDPARAM / UNUSEDSIGNAL:
#       Parameterized RTL and PHY/NoC stub signals
#   - WIDTHTRUNC / WIDTHEXPAND:
#       Width adaptation from timing parameters, counters, and indices
#   - UNDRIVEN / UNSUPPORTED:
#       BFM utilizes 'z' state for DDR BUS and don't drive command signals.
#
# These warnings are expected by design and do not indicate functional issues.
# ------------------------------------------------------------------------------
if ! "$VERILATOR_PATH" --lint-only +1800-2017ext+sv \
    -Wall -Wpedantic \
    -Wno-WIDTHEXPAND \
    -Wno-UNUSEDPARAM \
    -Wno-UNUSEDSIGNAL \
    -Wno-WIDTHTRUNC \
    -Wno-UNDRIVEN \
    -I"$BFM_PATH" \
    -I"$RTL_PATH/common" \
    -f "$SCRIPT_DIR/lint_bfm_filelist.f" \
    --top-module MemoryBFM \
    2> "$LOG_FILE"; then
  printf "%bERROR:%b Lint failed. See %s\n" "$Red" "$NC" "$LOG_FILE"
  exit 1;
else 
  rm $LOG_FILE
  printf "%bINFO:%b Lint Succeeded! Check %s\n" "$Blue" "$NC" "$LOG_FILE"
fi
