function pl = bb_plant_defaults()
%BB_PLANT_DEFAULTS  Assumed geometry of the physical rig (edit to match yours).
%   MG996R: ~0.09 deg per us of pulse. Lever ratio (servo horn radius /
%   distance from beam pivot to linkage) assumed 0.1, so the beam moves
%   0.009 deg per us. Beam usable from 40 mm to 600 mm from the sensor.
pl.center_us     = 1750;
pl.rad_per_us    = 0.1 * 0.09*pi/180;
pl.tau_servo     = 0.03;          % s
pl.slew_us_per_s = 6000;          % ~ 0.17 s / 60 deg
pl.xmin          = 0.04;
pl.xmax          = 0.60;
pl.noise_mm      = 2;             % HC-SR04 1-sigma noise on a ball
end
