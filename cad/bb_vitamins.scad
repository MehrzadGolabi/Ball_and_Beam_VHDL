// Bought parts that NopSCADlib does not provide. Dimensions are typical
// datasheet values - measure your own parts and adjust.
include <bb_design.scad>

// 40 mm ping-pong ball
module pingpong_ball() {
    color("orange") sphere(d = ball_d);
}

// MG996R, modelled with the output shaft on +z at the origin,
// case top face at z = -servo_spline_h, body extending in +x from the shaft.
module mg996r() {
    L = mg996r_body[0]; W = mg996r_body[1]; H = mg996r_body[2];
    top = -servo_spline_h;
    translate([-servo_shaft_off, -W/2, top - H]) {
        color([0.1, 0.1, 0.1]) cube([L, W, H]);
        // mounting ears
        color([0.1, 0.1, 0.1])
        translate([-(mg996r_ear_span - L)/2, 0, servo_ear_h - servo_ear_t])
            difference() {
                cube([mg996r_ear_span, W, servo_ear_t]);
                for (sx = [-1, 1], sy = [-1, 1])
                    translate([mg996r_ear_span/2 + sx*mg996r_hole_pitch[0]/2,
                               W/2 + sy*mg996r_hole_pitch[1]/2, -1])
                        cylinder(d = 4.5, h = servo_ear_t + 2);
            }
    }
    // gear boss and spline
    color([0.15, 0.15, 0.15]) translate([0, 0, top - eps]) cylinder(d = 12, h = 0.5);
    color("gold") translate([0, 0, top]) cylinder(d = 5.8, h = servo_spline_h);
}

// stock round horn (disc) that ships with the servo, on the spline
module servo_disc_horn() {
    color("white") cylinder(d = 24, h = horn_disc_t);
}

// HC-SR04: origin at the centre of the transducer FRONT faces, looking +x.
module hcsr04() {
    bw = hcsr04_board[0]; bh = hcsr04_board[1];
    translate([-hcsr04_can_h, 0, 0]) {
        color("royalblue") translate([-1.6, -bw/2, -bh/2]) cube([1.6, bw, bh]);
        for (sy = [-1, 1])
            translate([0, sy*hcsr04_can_pitch/2, 0]) rotate([0, 90, 0]) {
                color("silver") cylinder(d = hcsr04_can_d, h = hcsr04_can_h);
                color([0.2, 0.2, 0.2]) translate([0, 0, hcsr04_can_h - 0.5]) cylinder(d = hcsr04_can_d - 3, h = 0.6);
            }
        // header pins (bottom edge, pointing back)
        color("gold") for (i = [0:3]) translate([-1.6 - 6, -3.81 + i*2.54, -bh/2 + 1.5]) cube([6, 0.64, 0.64]);
    }
}

// M3 ball link (rod end with ball stud). Origin at the ball centre,
// threaded socket along +z (towards the pushrod), stud along +y (outboard).
module ball_link() {
    color("silver") sphere(d = 6.4);
    color([0.2, 0.2, 0.2]) {
        translate([0, 0, 2]) cylinder(d = 8, h = 8);              // socket body
    }
    color("silver") rotate([-90, 0, 0]) cylinder(d = 3, h = 12);  // M3 stud
}

// M3 threaded pushrod between the two ball links
module pushrod() {
    color("silver") translate([0, 0, 9]) cylinder(d = 3, h = rod_len - 18);
}
