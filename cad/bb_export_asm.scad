// Printed parts in their ASSEMBLY frames (for FreeCAD / other CAD):
//   beam parts : beam frame (origin on the pivot axis, x along the beam)
//   ground parts: world frame (origin on the base top under the pivot)
//   horn_arm   : horn frame (origin on the servo shaft, arm along -x, z = shaft)
//   openscad --render -D 'part="beam_seg1"' -o asm_stl/beam_seg1.stl bb_export_asm.scad
include <bb_design.scad>
use <bb_parts.scad>

part = "beam_seg1";

if (part == "beam_seg0") beam_segment(0);
if (part == "beam_seg1") beam_segment(1);
if (part == "beam_seg2") beam_segment(2);
if (part == "pivot_hub") pivot_hub();
if (part == "link_bracket") link_bracket();
if (part == "tower_left") tower(1);
if (part == "tower_right") tower(-1);
if (part == "servo_bracket") servo_bracket();
if (part == "horn_arm") horn_arm();
if (part == "base") base_plate();
