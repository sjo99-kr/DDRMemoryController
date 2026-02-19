#!/usr/bin/env bash

export LC_ALL=C
export LANG=C
set -euo pipefail

Red='\033[0;31m'
Blue='\033[0;34m'
Green='\033[0;32m'
NC='\033[0m'

# ------------------------------------------------
# Tool Check
# ------------------------------------------------
for tool in xvlog xelab xsim; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    printf "%bERROR:%b %s not found in PATH\n" "$Red" "$NC" "$tool"
    exit 1
  fi
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"



TOP="Top_xsim"
SNAPSHOT="sim_snapshot"

DFINES="-d DISPLAY"



printf "%b[XSIM]%b Compiling...\n" "$Blue" "$NC"
xvlog -sv $DFINES -f xsim_filelist.f

printf "%b[XSIM]%b Elaborating...\n" "$Blue" "$NC"
xelab "$TOP" -s "$SNAPSHOT"

printf "%b[XSIM]%b Running simulation...\n" "$Blue" "$NC"

if [[ "${1:-}" == "gui" ]]; then # ./xsim_uvm.sh  Or ./xsim_uvm.sh gui 
  xsim "$SNAPSHOT" --gui
else
  xsim "$SNAPSHOT" -runall
fi

printf "%b[XSIM]%b Done.\n" "$Green" "$NC"



rm -rf xsim.dir *.log *.jou *.pb
