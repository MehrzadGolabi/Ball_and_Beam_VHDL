#!/usr/bin/env bash
# Compile the HDL Coder output + board wrapper with GHDL and run the tests.
#   scripts/run_ghdl.sh            -> closed-loop system test (6 s simulated)
#   scripts/run_ghdl.sh hdlcoder   -> HDL Coder generated testbench (RTL vs Simulink)
#   STOP_MS=2000 STEP_MS=1000 scripts/run_ghdl.sh   -> shorter closed-loop run
set -euo pipefail
GHDL="${GHDL:-/c/Users/Thanotos/AppData/Local/Microsoft/WinGet/Packages/ghdl.ghdl.ucrt64.mcode_Microsoft.Winget.Source_8wekyb3d8bbwe/bin/ghdl.exe}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HDL="$ROOT/hdl/bb_core"
B="$ROOT/build"
mkdir -p "$B"; cd "$B"; rm -f work-obj08.cf
OPTS="--std=08 --workdir=. -frelaxed"
for f in $(grep -o '[A-Za-z0-9_]*\.vhd' "$HDL/BallBeamCore_compile.do"); do
  "$GHDL" -a $OPTS "$HDL/$f"
done
"$GHDL" -a $OPTS "$ROOT/rtl/top_ball_beam.vhd"

if [[ "${1:-}" == "hdlcoder" ]]; then
  for f in $(grep -o '[A-Za-z0-9_]*\.vhd' "$HDL/BallBeamCore_tb_compile.do" | grep -i tb); do
    "$GHDL" -a $OPTS "$HDL/$f"
  done
  cp "$HDL"/*.dat . 2>/dev/null || true
  "$GHDL" -e $OPTS BallBeamCore_tb
  "$GHDL" -r $OPTS BallBeamCore_tb --ieee-asserts=disable 2>&1 | tail -20
else
  "$GHDL" -a $OPTS "$ROOT/tb/tb_closed_loop.vhd"
  "$GHDL" -e $OPTS tb_closed_loop
  "$GHDL" -r $OPTS tb_closed_loop -gSTOP_MS="${STOP_MS:-6000}" -gSTEP_MS="${STEP_MS:-3000}" \
      --ieee-asserts=disable
fi
