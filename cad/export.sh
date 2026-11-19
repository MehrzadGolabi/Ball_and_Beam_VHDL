#!/usr/bin/env bash
# Export every printable part to cad/stl/*.stl, the base drilling template to
# cad/stl/base.dxf, run the interference checks, and render docs/cad/*.png.
set -euo pipefail
O="${OPENSCAD:-/c/Program Files/OpenSCAD/openscad.com}"
cd "$(dirname "$0")"
mkdir -p stl ../docs/cad
for p in beam_near beam_mid beam_far pivot_hub link_bracket tower_left tower_right servo_bracket horn_arm; do
  echo "== $p"
  "$O" --render -D "part=\"$p\"" -o "stl/$p.stl" bb_print.scad 2>&1 | grep -E "WARNING|ERROR|Volumes" || true
  "$O" -D "part=\"$p\"" --imgsize=800,600 --viewall --autocenter -o "../docs/cad/part_$p.png" bb_print.scad >/dev/null 2>&1
done
"$O" -D 'part="base_dxf"' -o stl/base.dxf bb_print.scad >/dev/null 2>&1

echo "== interference checks (beam vs fixed parts, horn vs fixed parts)"
fail=0
for th in -67.5 -45 0 30 45; do
  for p in 1 2 3 4; do
    log=$("$O" --render -D theta=$th -D pair=$p -o _c.stl check_clearance.scad 2>&1 || true)
    if grep -qi "empty" <<<"$log"; then r=CLEAR; else r=COLLISION; fail=1; fi
    echo "  theta=$th pair=$p $r"; rm -f _c.stl
  done
done
"$O" -o ../docs/cad/assembly.png --imgsize=1400,800 --viewall --autocenter -D theta=-30 bb_assembly.scad >/dev/null 2>&1
exit $fail
