function s = bb_plant_step(s, servo_us, dt, pl)
%BB_PLANT_STEP  Ball-and-beam plant used by every closed-loop test.
%   State s.x [m from sensor], s.v [m/s], s.th [servo deflection, us-equiv].
%   The servo is a first-order lag with a slew limit; the beam angle is
%   proportional to the servo deflection from the level position.
%   Positive deflection tilts the beam so the ball rolls AWAY from the sensor.
%   The same equations are mirrored in tb/tb_closed_loop.vhd.
if nargin < 4, pl = bb_plant_defaults(); end
target = servo_us - pl.center_us;
dth    = (target - s.th) / pl.tau_servo * dt;
maxd   = pl.slew_us_per_s * dt;
dth    = max(-maxd, min(maxd, dth));
s.th   = s.th + dth;
alpha  = pl.rad_per_us * s.th;                 % beam angle [rad]
a      = (5/7) * 9.81 * sin(alpha);            % solid ball rolling
s.v    = s.v + a*dt;
s.x    = s.x + s.v*dt;
% end stops
if s.x < pl.xmin, s.x = pl.xmin; s.v = 0; end
if s.x > pl.xmax, s.x = pl.xmax; s.v = 0; end
end
