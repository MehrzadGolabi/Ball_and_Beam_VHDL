// Printable parts in print orientation. Select with -D part="..." or the Customizer.
//   openscad -D 'part="beam_mid"' -o stl/beam_mid.stl bb_print.scad
include <bb_design.scad>
use <bb_parts.scad>

part = "beam_mid"; // [beam_near, beam_mid, beam_far, pivot_hub, link_bracket, tower_left, tower_right, servo_bracket, horn_arm, base_dxf]

function seg_x0(i) = -beam_len/2 + i*seg_len;

if (part == "beam_near")  translate([-seg_x0(0), 0, -cradle_bot]) beam_segment(0);
if (part == "beam_mid")   translate([-seg_x0(1), 0, -cradle_bot]) beam_segment(1);
if (part == "beam_far")   translate([-seg_x0(2), 0, -cradle_bot]) beam_segment(2);
if (part == "pivot_hub")  translate([0, 0, -hub_bot]) pivot_hub();
if (part == "link_bracket") translate([0, 0, link_y + 5 + link_plate_t]) rotate([-90, 0, 0]) link_bracket();
if (part == "tower_left")  tower(1);
if (part == "tower_right") tower(-1);
if (part == "servo_bracket") servo_bracket();
if (part == "horn_arm")   horn_arm();
if (part == "base_dxf")   projection(cut = true) translate([0, 0, base_size[2]/2]) base_plate();
