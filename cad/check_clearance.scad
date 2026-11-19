// Interference check: intersection of the moving beam with the fixed parts.
// openscad --render -D theta=<deg> -D pair=<n> -o out.stl check_clearance.scad
// An empty result ("top level object is empty") means no collision.
include <bb_vitamins.scad>
use <bb_parts.scad>
theta = 0;
pair  = 1;
alpha = beam_alpha(theta);
module beam_moving() {
    translate([0, 0, pivot_h]) rotate([0, alpha, 0]) {
        for (i = [0:n_segments - 1]) beam_segment(i);
        pivot_hub(); link_bracket();
    }
}
module servo_fixed() {
    translate([servo_x, servo_tip_y, servo_z]) rotate([90, 0, 0]) rotate([0, 0, 180]) mg996r();
}
module horn_moving() {
    translate([servo_x, servo_tip_y, servo_z]) rotate([0, theta, 0])
        translate([0, -horn_disc_t, 0]) rotate([90, 0, 0]) horn_arm();
}
if (pair == 1) intersection() { beam_moving(); union() { tower(1); tower(-1); } }
if (pair == 2) intersection() { beam_moving(); union() { servo_bracket(); servo_fixed(); } }
if (pair == 3) intersection() { beam_moving(); horn_moving(); }
if (pair == 4) intersection() { horn_moving(); union() { servo_bracket(); base_plate(); } }
if (pair == 9) intersection() { beam_moving(); translate([0,-40,pivot_h-20]) cube([20,80,20]); }   // self-test: must collide
