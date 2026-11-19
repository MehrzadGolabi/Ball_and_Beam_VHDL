// Ball & Beam - printable parts, modelled in their ASSEMBLED position.
//   beam parts : beam frame (origin on the pivot axis, x along the beam, z up)
//   ground parts: world frame (origin on the base top, under the pivot)
// print/*.scad re-orient each part for printing.
include <bb_design.scad>

BIG = 200;
m3_tap   = 2.5;     // M3 thread-forming / tapped hole
m3_clear = 3.3;
m3_insert_d = 4.2;  // M3 x 5 heat-set insert hole
m3_insert_h = 5.5;

// ------------------------------------------------------------------ cradle
// 2-D cross-section in (y, z). Rods sit in grooves that wrap 225 deg (captured),
// open towards the ball so the ball touches only the rods.
module cradle_profile() {
    y_in = rail_y - rod_hole_d/2;                       // inner edge of the groove
    lip_y = rail_y + rod_hole_d/2*cos(180 - lip_ang);   // outer lip point
    difference() {
        translate([-cradle_w2, cradle_bot]) square([2*cradle_w2, lip_z - cradle_bot]);
        // channel between the rails (ball clearance)
        polygon([[-y_in, BIG], [-y_in, rail_z], [-6, cradle_top], [6, cradle_top],
                 [y_in, rail_z], [y_in, BIG]]);
        for (s = [-1, 1]) {
            translate([s*rail_y, rail_z]) circle(d = rod_hole_d);
            polygon([[s*rail_y, rail_z], [s*y_in, rail_z], [s*y_in, BIG],
                     [s*lip_y, BIG], [s*lip_y, lip_z]]);
        }
    }
}

module cradle_extrusion(x0, x1) {
    translate([x0, 0, 0]) rotate([90, 0, 90]) linear_extrude(x1 - x0) cradle_profile();
}

// grub screws (M3, from below) lock each segment to the rods
module rod_grub_holes(x) {
    for (s = [-1, 1]) translate([x, s*rail_y, cradle_bot - eps]) cylinder(d = m3_tap, h = rail_z - cradle_bot);
}

function seg_x0(i) = -beam_len/2 + i*seg_len;
function seg_x1(i) = seg_x0(i) + seg_len;

// heat-set inserts under the middle segment for the pivot hub
hub_screw_pos = [for (sx = [-1, 1], sy = [-1, 1]) [sx*12, sy*7]];
// inserts in the far segment side face for the link bracket
bracket_screw_pos = [for (dx = [-9, 9]) [link_attach_x + dx, 3]];

module beam_segment(i) {
    x0 = seg_x0(i); x1 = seg_x1(i);
    difference() {
        union() {
            cradle_extrusion(x0, x1);
            if (i == 0) sensor_holder();
            if (i == n_segments - 1) end_stop();
        }
        rod_grub_holes((x0 + x1)/2 + (i == 1 ? 45 : 0));
        if (i == 1) {
            // axle passes through the middle of the cradle
            rotate([90, 0, 0]) cylinder(d = axle_d + 0.4, h = BIG, center = true);
            for (p = hub_screw_pos) translate([p[0], p[1], cradle_bot - eps]) cylinder(d = m3_insert_d, h = m3_insert_h);
        }
        if (i == n_segments - 1)
            for (p = bracket_screw_pos) translate([p[0], cradle_w2 + eps, p[1]]) rotate([90, 0, 0])
                cylinder(d = m3_insert_d, h = m3_insert_h);
        // rods run through the end features as well
        for (s = [-1, 1]) translate([x0 - 1, s*rail_y, rail_z]) rotate([0, 90, 0]) cylinder(d = rod_hole_d, h = seg_len + 2);
    }
}

// HC-SR04 holder: transducer faces flush with the plate front at x = sensor_x_face
module sensor_holder() {
    bw = hcsr04_board[0]; bh = hcsr04_board[1];
    pw = bw + 8; ph_top = ball_zc + bh/2 + 4;
    x_f = sensor_x_face;
    difference() {
        union() {
            translate([x_f - sensor_plate_t, -pw/2, cradle_bot]) cube([sensor_plate_t, pw, ph_top - cradle_bot]);
            // gussets on the outer lips, below the transducers (ultrasonic cones stay clear)
            g_top = ball_zc - hcsr04_can_d/2 - 1.5;
            for (s = [-1, 1]) translate([x_f, s > 0 ? cradle_w2 - 4 : -cradle_w2, lip_z - eps])
                rotate([90, 0, 0]) translate([0, 0, -4]) linear_extrude(4)
                    polygon([[0, 0], [22, 0], [0, g_top - lip_z]]);
            // standoffs for the PCB (4x M2, approx. hole positions)
            for (sy = [-1, 1], sz = [-1, 1])
                translate([x_f - sensor_plate_t, sy*(bw/2 - 2.5), ball_zc + sz*(bh/2 - 2.5)])
                    rotate([0, -90, 0]) cylinder(d = 5, h = hcsr04_can_h - sensor_plate_t);
        }
        for (sy = [-1, 1]) translate([x_f - sensor_plate_t - 1, sy*hcsr04_can_pitch/2, ball_zc])
            rotate([0, 90, 0]) cylinder(d = hcsr04_can_d + 0.6, h = sensor_plate_t + 30);
        for (sy = [-1, 1], sz = [-1, 1])
            translate([x_f - hcsr04_can_h - 1, sy*(bw/2 - 2.5), ball_zc + sz*(bh/2 - 2.5)])
                rotate([0, 90, 0]) cylinder(d = 1.8, h = hcsr04_can_h);
        // keep the ball channel clear behind the plate
        translate([x_f, -(rail_y - rod_hole_d/2), cradle_top]) cube([30, 2*(rail_y - rod_hole_d/2), BIG]);
    }
}

module end_stop() {
    x = beam_len/2 - endstop_t;
    difference() {
        translate([x, -cradle_w2, cradle_bot]) cube([endstop_t, 2*cradle_w2, ball_zc + 6 - cradle_bot]);
        // soft-landing notch: the ball hits a concave face, centred on the track
        translate([x - ball_d/2 + 1.5, 0, ball_zc]) sphere(d = ball_d);
    }
}

// ------------------------------------------------------------------ pivot hub
// U-shaped block bolted under the middle segment; carries 2x F625 bearings.
module pivot_hub() {
    cheek_in = cradle_w2 + clr;
    bt = bearing[3];                 // width 5
    difference() {
        union() {
            translate([-hub_len/2, -hub_w/2, hub_bot]) cube([hub_len, hub_w, cradle_bot - hub_bot]);
            for (s = [-1, 1]) translate([-hub_len/2, s > 0 ? cheek_in : -hub_w/2, hub_bot])
                cube([hub_len, hub_w/2 - cheek_in, 12 - hub_bot]);
        }
        // axle + bearing pockets (flange outside)
        rotate([90, 0, 0]) cylinder(d = axle_d + 1, h = BIG, center = true);
        for (s = [-1, 1]) {
            translate([0, s*(hub_w/2 - bt + eps), 0]) rotate([s*-90, 0, 0]) cylinder(d = bearing[2] + 0.15, h = bt + 1);
            translate([0, s*(hub_w/2 - 1 + eps), 0]) rotate([s*-90, 0, 0]) cylinder(d = bearing[7] + 0.6, h = 2);
        }
        for (p = hub_screw_pos) {
            translate([p[0], p[1], hub_bot - eps]) cylinder(d = m3_clear, h = BIG);
            translate([p[0], p[1], hub_bot - eps]) cylinder(d = 6, h = 3.5);   // counterbore
        }
    }
}

// ------------------------------------------------------------------ link bracket
// bolts to the far segment's +y side face; the upper ball link screws into it.
module link_bracket() {
    py0 = link_y + 5;                       // plate inner face (ball link body clearance)
    difference() {
        union() {
            translate([link_attach_x - 10, cradle_w2 + clr, cradle_bot]) cube([20, py0 + link_plate_t - cradle_w2 - clr, lip_z - cradle_bot]);
            translate([link_attach_x - 10, py0, link_attach_z - 8]) cube([20, link_plate_t, lip_z - link_attach_z + 8]);
        }
        translate([link_attach_x, py0 - 1, link_attach_z]) rotate([-90, 0, 0]) cylinder(d = m3_clear, h = 10);
        for (p = bracket_screw_pos) {
            translate([p[0], 0, p[1]]) rotate([-90, 0, 0]) cylinder(d = m3_clear, h = BIG);
            translate([p[0], py0 + link_plate_t - 3, p[1]]) rotate([-90, 0, 0]) cylinder(d = 6, h = 10);
        }
    }
}

// ------------------------------------------------------------------ towers
module tower(side = 1) {
    y0 = tower_gap/2; t = tower_w; d = tower_d;
    mirror([0, side < 0 ? 1 : 0, 0]) difference() {
        union() {
            hull() {
                translate([-d/2, y0, 0]) cube([d, t, 1]);
                translate([0, y0, pivot_h]) rotate([-90, 0, 0]) cylinder(d = 24, h = t);
            }
            translate([-d/2 - 10, y0, 0]) cube([d + 20, t + tower_foot_l, 5]);     // foot
            for (sx = [-1, 1]) translate([sx*(d/2 - 4) - 2, y0 + t, 0])             // gussets
                rotate([90, 0, 90]) linear_extrude(4) polygon([[0, 0], [tower_foot_l - 2, 0], [0, 50]]);
        }
        translate([0, y0 - 1, pivot_h]) rotate([-90, 0, 0]) cylinder(d = axle_d + 0.3, h = t + 2);
        for (sx = [-1, 1]) translate([sx*(d/2 + 3), y0 + t + tower_foot_l/2 + 4, -1]) cylinder(d = 4.5, h = 10);
    }
}

// ------------------------------------------------------------------ servo bracket
// MG996R lies on its side, shaft along -y; the ears screw onto this plate.
module servo_bracket() {
    L = mg996r_body[0]; W = mg996r_body[1];
    bx1 = servo_x + servo_shaft_off;   // case end near the shaft
    bx0 = bx1 - L;                     // case far end (towards the pivot)
    ex0 = bx0 - (mg996r_ear_span - L)/2; ex1 = bx1 + (mg996r_ear_span - L)/2;
    py = servo_ear_y;
    ztop = servo_z + W/2 + 9;
    difference() {
        union() {
            translate([ex0 - 5, py, 0]) cube([ex1 - ex0 + 10, servo_plate_t, ztop]);
            translate([ex0 - 5, py - 24, 0]) cube([ex1 - ex0 + 10, 24 + servo_plate_t, 5]);   // foot (inboard)
            for (x = [ex0 - 5, ex1 + 1]) translate([x, py, 0])                           // gussets
                rotate([90, 0, 90]) linear_extrude(4) polygon([[0, 0], [-22, 0], [0, 30]]);
        }
        translate([bx0 - 0.4, py - 1, servo_z - W/2 - 0.4]) cube([L + 0.8, servo_plate_t + 2, W + 0.8]);
        for (sx = [-1, 1], sz = [-1, 1])
            translate([(ex0 + ex1)/2 + sx*mg996r_hole_pitch[0]/2, py - 1, servo_z + sz*mg996r_hole_pitch[1]/2])
                rotate([-90, 0, 0]) cylinder(d = m3_tap + 0.5, h = servo_plate_t + 2);
        for (sx = [-1, 1]) translate([(ex0 + ex1)/2 + sx*22, py - 13, -1]) cylinder(d = 4.5, h = 10);
    }
}

// ------------------------------------------------------------------ horn arm
// Printed arm screwed onto the servo's stock round horn. Modelled in the
// horn frame: shaft axis = z (pointing away from the servo), arm along -x.
module horn_arm() {
    difference() {
        hull() {
            cylinder(d = 22, h = horn_t);
            translate([-horn_r, 0, 0]) cylinder(d = 10, h = horn_t);
        }
        translate([0, 0, -1]) cylinder(d = 3.2, h = horn_t + 2);          // horn screw
        for (a = [0:90:270]) rotate(a) translate([7, 0, -1]) cylinder(d = 2, h = horn_t + 2);  // disc horn holes (check yours)
        translate([-horn_r, 0, -1]) cylinder(d = m3_clear, h = horn_t + 2);   // ball link stud
    }
}

// ------------------------------------------------------------------ base (plywood / acrylic template)
module base_plate() {
    difference() {
        translate([-base_size[0]/2, -base_size[1]/2, -base_size[2]]) cube(base_size);
        for (s = [-1, 1], sx = [-1, 1]) translate([sx*(tower_d/2 + 3), s*(tower_gap/2 + tower_w + tower_foot_l/2 + 4), -10])
            cylinder(d = 4.5, h = 20);
    }
}
