function scn = bb_scenario()
%BB_SCENARIO  Default verification scenario for bb_simscape.
%   Ball released near the sensor (100 mm surface distance) with the FPGA
%   power-up registers (setpoint 330 mm). At 7 s the setpoint moves to 250 mm
%   (as a UART write would do); at 13 s the ball gets a push along the track.
scn.T        = 19;                       % s
scn.s0       = 0.120;                    % ball centre from sensor face at t=0 [m]
scn.sp       = [0 330; 7 250];           % [t  setpoint_mm]
scn.dist_t   = 13;                       % s
scn.dist_N   = 0.006;                    % N along +track
scn.dist_dur = 0.1;                      % s
scn.reg      = struct();                 % register overrides, e.g. struct('KP', 512)
end
