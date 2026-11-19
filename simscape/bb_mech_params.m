function M = bb_mech_params()
%BB_MECH_PARAMS  Mechanical design of the ball-and-beam rig (SI units, m/kg).
%   Single source of truth for the Simscape model AND (via export_mech_json)
%   for the OpenSCAD CAD in the next phase.
%
%   World frame: origin on the top face of the base plate, directly under the
%   beam pivot. x along the beam (towards the far end), y lateral, z up.
%
%   Concept
%   * Track: two 6 mm steel rods (24 mm apart) held by a printed cradle.
%     A 40 mm ping-pong ball rolls on the rods: light, cheap, and a good
%     ultrasonic target.
%   * Beam pivots at its centre on a 5 mm axle (2x 625ZZ bearings) carried by
%     two printed towers.
%   * The HC-SR04 is mounted ON the beam at the near end, so it always measures
%     distance along the track, whatever the beam angle.
%   * MG996R servo on the base, near the far end. A printed horn arm
%     (34 mm) drives a 70 mm pushrod with M3 ball links to a bracket under the
%     beam, 250 mm from the pivot -> beam angle ~ 0.136 x servo angle.
%     With +/-45 deg of servo travel the beam tilts about +/-6 deg.

% ---- ball (ITTF ping-pong ball) ----
M.ball.r     = 0.020;
M.ball.m     = 0.0027;
M.ball.I     = 2/3*M.ball.m*M.ball.r^2;          % thin spherical shell

% ---- track ----
M.rail.r       = 0.003;                          % 6 mm steel rod
M.rail.len     = 0.560;
M.rail.spacing = 0.024;                          % centre to centre
M.rail.z       = 0.012;                          % rail centres above the pivot axis
M.rail.rho     = 7850;
d = M.ball.r + M.rail.r; h = M.rail.spacing/2;
M.ball.zc   = M.rail.z + sqrt(d^2 - h^2);        % ball centre height above pivot axis
M.ball.reff = sqrt(M.ball.r^2 - (M.ball.r*h/d)^2); % rolling radius on the two rods
M.ball.meff = M.ball.m + M.ball.I/M.ball.reff^2;   % translational equivalent mass

% ---- beam cradle (PLA, ~50 % infill) ----
M.beam.len   = M.rail.len;
M.beam.w     = 0.036;
M.beam.h     = 0.010;
M.beam.z     = 0.003;                            % cradle centre above pivot axis
M.beam.rho   = 620;
M.beam.pivot_r = 0.0025;                         % 5 mm axle
M.pivot.h    = 0.130;                            % pivot axis height above base
M.tower.w    = 0.012; M.tower.d = 0.040; M.tower.gap = 0.060;   % towers at y = +/-gap/2

% ---- sensor (HC-SR04, on the beam) ----
M.sensor.x_face = -0.270;                        % beam-frame x of the transducer face
M.sensor.board  = [0.045 0.020 0.015];
M.sensor.m      = 0.020;                         % board + bracket
M.sensor.c      = 343;                           % speed of sound, m/s
M.sensor.noise_mm = 2;                           % 1-sigma noise on a ball
M.sensor.lat_us = 200;                           % trigger -> echo start
M.track.s_min  = M.ball.r + 0.020;               % ball centre limits measured from the face
M.track.s_max  = (M.rail.len/2 - 0.008) - M.sensor.x_face - M.ball.r;   % far end stop

% ---- linkage ----
M.link.attach_x = 0.250;                         % bracket on the beam, beam frame
M.link.attach_z = -0.015;
M.link.y        = 0.030;                         % linkage plane (beam side)
M.horn.r        = 0.034;                         % servo axis -> ball link
M.servo.z       = 0.045;                         % servo shaft height above base
M.servo.x       = M.link.attach_x + M.horn.r;    % horn horizontal (pointing -x) at neutral
M.rod.len       = M.pivot.h + M.link.attach_z - M.servo.z;
M.rod.r         = 0.0015;
M.rod.m         = 0.006;

% ---- MG996R ----
M.servo.body    = [0.0407 0.0197 0.0429];
M.servo.tau_max = 0.92;          % N*m stall @ 6 V (9.4 kg*cm)
M.servo.w_max   = 7.0;           % rad/s no-load (0.14 s/60 deg @ 6 V)
M.servo.kp      = 10;            % N*m/rad internal position loop
M.servo.J       = 2e-5;          % reflected rotor inertia, kg*m^2
M.servo.deg_per_us = 0.09;       % pulse -> angle
M.servo.center_us  = 1750;       % horn horizontal = beam level
M.servo.dir     = -1;            % +pulse lowers the far end (ball rolls away from sensor)

% ---- base ----
M.base.size = [0.640 0.160 0.006];
M.base.x    = 0.000;
end
