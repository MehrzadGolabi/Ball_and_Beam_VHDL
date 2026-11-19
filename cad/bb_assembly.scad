// Ball & Beam - full assembly. Open in OpenSCAD and use View > Animate
// (FPS 10, Steps 60) to sweep the servo; the beam follows through the solved
// four-bar linkage and the ball rolls along the track.
include <bb_vitamins.scad>
use <bb_parts.scad>

theta  = 45 * sin(360 * $t);                      // servo horn angle, deg (+ = horn tip up)
s_ball = (track_s_min + track_s_max)/2 + 0.45*(track_s_max - track_s_min)*sin(360*$t + 90);

bb_assembly(theta, s_ball);

module bb_assembly(theta = 0, s = 300, explode = 0) {
    alpha = beam_alpha(theta);
    echo(str("servo ", theta, " deg -> beam ", alpha, " deg"));

    // ---- ground ----
    color("burlywood") base_plate();
    color("SteelBlue") { tower(1); tower(-1); }
    color("SteelBlue") servo_bracket();
    // axle (M5 bolt)
    translate([0, -axle_len/2, pivot_h]) rotate([-90, 0, 0]) color("silver") cylinder(d = axle_d, h = axle_len);

    // ---- servo + horn ----
    // servo frame: shaft +z -> world -y, body (+x) -> world -x
    translate([servo_x, servo_tip_y, servo_z]) rotate([90, 0, 0]) rotate([0, 0, 180]) mg996r();
    translate([servo_x, servo_tip_y, servo_z]) rotate([0, theta, 0]) {
        rotate([90, 0, 0]) servo_disc_horn();
        translate([0, -horn_disc_t, 0]) rotate([90, 0, 0]) color("orange") horn_arm();
    }

    // ---- beam ----
    translate([0, 0, pivot_h]) rotate([0, alpha, 0]) {
        for (i = [0:n_segments - 1]) color(i == 1 ? "DodgerBlue" : "CornflowerBlue") beam_segment(i);
        color("RoyalBlue") pivot_hub();
        color("RoyalBlue") link_bracket();
        for (sy = [-1, 1]) translate([0, sy*rail_y, rail_z]) rotate([0, 90, 0]) rod(rail_d, rail_len);
        for (sy = [-1, 1]) translate([0, sy*(hub_w/2 - bearing[3]), 0]) rotate([sy*-90, 0, 0]) ball_bearing(bearing);
        translate([sensor_x_face, 0, ball_zc]) hcsr04();
        translate([sensor_x_face + s, 0, ball_zc]) pingpong_ball();
    }

    // ---- pushrod with ball links ----
    H = horn_tip(theta);
    A = [0, pivot_h] + rot_y([link_attach_x, link_attach_z], alpha);
    v = A - H;
    tilt = atan2(v[0], v[1]);
    translate([H[0], link_y, H[1]]) rotate([0, tilt, 0]) { ball_link(); pushrod(); }
    translate([A[0], link_y, A[1]]) rotate([0, tilt + 180, 0]) ball_link();
}
