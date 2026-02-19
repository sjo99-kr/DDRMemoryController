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
#   - RTL_PATH   : top-level RTL directory
#   - LOG_FILE   : lint output log
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RTL_PATH="$PROJECT_ROOT/rtl"
LOG_FILE="$SCRIPT_DIR/lint_rtl.log"

if [ ! -d "$RTL_PATH" ]; then
  printf "%bERROR:%b RTL directory not found: %s\n" "$Red" "$NC" "$RTL_PATH"
  exit 1
fi 

printf "%bINFO:%b Verilator found at %s\n" "$Blue" "$NC" "$VERILATOR_PATH"
printf "%bINFO:%b Memory Controller Lint START\n" "$Blue" "$NC"


# ------------------------------------------------------------------------------
# Lint policy note:
#
# The following warnings are intentionally suppressed:
#   - UNUSEDPARAM / UNUSEDSIGNAL:
#       Parameterized RTL and PHY/NoC stub signals
#   - WIDTHTRUNC / WIDTHEXPAND:
#       Width adaptation from timing parameters, counters, and indices
#
# These warnings are expected by design and do not indicate functional issues.
# ------------------------------------------------------------------------------
if ! "$VERILATOR_PATH" --lint-only +1800-2017ext+sv \
    -Wall -Wpedantic \
    -Wno-WIDTHEXPAND \
    -Wno-UNUSEDPARAM \
    -Wno-UNUSEDSIGNAL \
    -Wno-WIDTHTRUNC \
    -I"$RTL_PATH" \
    -I"$RTL_PATH/common" \
    -I"$RTL_PATH/backend" \
    -I"$RTL_PATH/frontend" \
    -f "$SCRIPT_DIR/lint_rtl_filelist.f" \
    --top-module MemoryController \
    2> "$LOG_FILE"; then
  printf "%bERROR:%b Lint failed. See %s\n" "$Red" "$NC" "$LOG_FILE"
else 
  rm $LOG_FILE
  printf "%bINFO:%b Lint Succeeded! Check %s\n" "$Blue" "$NC" "$LOG_FILE"
fi
