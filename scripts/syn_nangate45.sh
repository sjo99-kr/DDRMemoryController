#!/usr/bin/env bash

#set -e -o pipefail

# ------------------------------------------------------------------------------
# Color codes for readable console output
# ------------------------------------------------------------------------------
Red='\033[0;31m'
Blue='\033[0;34m'
NC='\033[0m'




# ------------------------------------------------------------------------------
# Check yosys
# ------------------------------------------------------------------------------
if ! command -v yosys >/dev/null 2>&1; then
  printf "%bERROR:%b Yosys not found \n" "$Red" "$NC"
  exit 1
fi

YOSYS_PATH=$(command -v yosys)
YOSYS_VERSION=$(yosys -V)

printf "%bINFO%b: Using Yosys at %s\n" "$Blue" "$NC" "$YOSYS_PATH"
printf "%bINFO%b: %s\n" "$Blue" "$NC" "$YOSYS_VERSION"

# ------------------------------------------------------------------------------
# Run synthesis
# ------------------------------------------------------------------------------
printf "%bINFO%b: MEMORY CONTROLLER SYNTHESIS START\n" "$Blue" "$NC"

cd "$(dirname "$0")"

yosys -s syn_nangate45.ys

printf "%bINFO%b: SYNTHESIS DONE\n" "$Blue" "$NC"
