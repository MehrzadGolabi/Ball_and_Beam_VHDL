// Ball & Beam - hand-written design rules on top of the simulated geometry.
// bb_params.scad is GENERATED from simscape/bb_mech_params.m: change geometry
// there, re-run export_cad_params, and every part follows.
include <bb_params.scad>
include <NopSCADlib/core.scad>
include <NopSCADlib/vitamins/ball_bearings.scad>
include <NopSCADlib/vitamins/rod.scad>
include <NopSCADlib/vitamins/screws.scad>
include <NopSCADlib/vitamins/nuts.scad>
include <NopSCADlib/vitamins/washers.scad>

$fn = 64;
eps = 0.01;

// ---- printing ----
clr        = 0.25;      // general sliding clearance
wall       = 3;
rod_hole_d = rail_d + 0.2;     // rods slide through the cradle segments
n_segments = 3;                // beam printed in 3 pieces (fits a 220 mm bed)
seg_len    = beam_len / n_segments;

// ---- cradle cross-section (beam frame: pivot axis = origin, z up) ----
cradle_bot  = cradle_z - cradle_h/2;           // -2
cradle_top  = cradle_z + cradle_h/2;           //  8  (between the rails)
rail_y      = rail_spacing/2;                  // 12
lip_ang     = 135;                             // groove wraps 225 deg -> rods are captured
lip_z       = rail_z + rail_d/2*sin(lip_ang);  // outer lip height
cradle_w2   = rail_y + rail_d/2 + 5;           // half width (5 mm outer wall)

// ---- pivot hub (middle segment) ----
bearing     = BBF625;                          // flanged 625: 5 x 16 x 5, flange 18
hub_w       = tower_gap - 2*1.0;               // 1 mm washer each side
hub_len     = 34;
hub_bot     = -13;

// ---- link bracket / end stop / sensor holder ----
link_plate_t = 5;
endstop_t    = 8;
sensor_plate_t = 3;
hcsr04_face_gap = 0;    // transducer face flush with the plate front

// ---- towers ----
tower_top    = pivot_h + 12;
tower_foot_l = 30;      // foot flange (outwards)
axle_len     = 90;

// ---- servo bracket (MG996R lying on its side, shaft along -y) ----
horn_t       = 3.5;     // printed horn arm thickness
horn_disc_t  = 2.5;     // stock round horn under the arm
servo_spline_h = 4.0;   // spline above the case top
servo_ear_h  = 29.6;    // case bottom -> top of ears   (measure your servo)
servo_ear_t  = 2.5;
servo_shaft_off = 10.0; // shaft centre from the near case end
servo_plate_t = 5;
horn_y       = link_y + 6.0;   // horn arm centre plane: outboard of the ball-link balls (y = link_y)
servo_top_y  = horn_y + horn_t/2 + horn_disc_t + servo_spline_h;   // case top face (y)
servo_tip_y  = servo_top_y - servo_spline_h;                       // spline tip (shaft points -y)
servo_ear_y  = servo_top_y + (mg996r_body[2] - servo_ear_h) + servo_ear_t;  // ear underside (y)

// ---- motion ----
// four-bar: ground pivots O (beam pivot) and S (servo shaft), in the x-z plane
function rot_y(p, a) = [p[0]*cos(a) + p[1]*sin(a), -p[0]*sin(a) + p[1]*cos(a)];
function horn_tip(theta) = [servo_x - horn_r*cos(theta), servo_z + horn_r*sin(theta)];
// beam angle (deg, + = far end down) for servo angle theta (deg, + = horn tip up)
function beam_alpha(theta) =
    let(O  = [0, pivot_h],
        Al = [link_attach_x, link_attach_z],           // attach point, beam frame
        rho = norm(Al), phi0 = atan2(Al[1], Al[0]),
        H  = horn_tip(theta),
        d  = norm(H - O),
        a  = (rho*rho - rod_len*rod_len + d*d) / (2*d),
        h  = sqrt(max(rho*rho - a*a, 0)),
        u  = (H - O) / d,
        P1 = O + a*u + h*[-u[1], u[0]],
        P  = P1,                                       // upper solution (the real linkage)
        ang = atan2(P[1] - O[1], P[0] - O[0]))
    -(ang - phi0);
