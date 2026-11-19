function mdl = build_bb_simscape(scn)
%BUILD_BB_SIMSCAPE  Simscape Multibody model of the ball-and-beam rig driven
%   by the FPGA controller logic.
%
%   Controller (discrete, 10 kHz):   the SAME MATLAB Function sources that HDL
%     Coder turns into VHDL (simulink/mlfb/pos_filter.m, pid_ctrl.m), with
%     the power-up register values from simulink/bb_params.m, fed by an
%     HC-SR04 model (hcsr04_model.m). The servo pulse is latched every 20 ms
%     like servo_pwm.
%   Plant (Simscape Multibody):  base, pivot towers, beam (two steel rails +
%     PLA cradle) with the HC-SR04 on it, rolling ping-pong ball, MG996R servo
%     with an internal position loop (torque- and speed-limited), printed horn,
%     and a pushrod with ball links (closed four-bar loop).
%   Geometry/masses: bb_mech_params.m.  Scenario: bb_scenario.m (or scn).
if nargin < 1, scn = bb_scenario(); end
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(fileparts(here), 'simulink'));
live = isfield(scn, 'live') && scn.live;    % interactive model (see build_bb_live.m)
if live, mdl = 'bb_live'; else, mdl = 'bb_simscape'; end
M = bb_mech_params();
P = bb_params();
Ts = 1e-4;
if isfield(scn, 'Ts'), Ts = scn.Ts; end

if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(here, [mdl '.slx']);
if exist(f, 'file'), delete(f); end
new_system(mdl);
set_param(mdl, 'Solver', 'ode23t', 'StopTime', num2str(scn.T), 'MaxStep', '2e-3', ...
    'RelTol', '1e-4', 'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on', ...
    'AlgebraicLoopMsg', 'error');

% ---------------------------------------------------------------- plant
pl = [mdl '/Plant'];
add_block('built-in/Subsystem', pl, 'Position', [520 100 700 300]);
build_plant(pl, M, scn);

% ---------------------------------------------------------------- controller
ct = [mdl '/FPGA_Controller'];
add_block('built-in/Subsystem', ct, 'Position', [200 100 380 300]);
build_controller(ct, M, P, scn, Ts);

% ---------------------------------------------------------------- top wiring
% servo_pwm: absolute clamp, then the width is latched once per 20 ms frame
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/us_dbl'], ...
    'OutDataTypeStr', 'double', 'Position', [400 110 430 130]);
add_block('simulink/Discontinuities/Saturation', [mdl '/abs_clamp'], ...
    'UpperLimit', num2str(P.ABS_MAX_US), 'LowerLimit', num2str(P.ABS_MIN_US), 'Position', [440 110 470 130]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/pwm_frame'], ...
    'SampleTime', num2str(P.PWM_PERIOD_US*1e-6), 'Position', [480 110 510 130]);
lnk(mdl, 'FPGA_Controller', 1, 'us_dbl', 1); lnk(mdl, 'us_dbl', 1, 'abs_clamp', 1);
lnk(mdl, 'abs_clamp', 1, 'pwm_frame', 1); lnk(mdl, 'pwm_frame', 1, 'Plant', 1);
lnk(mdl, 'Plant', 1, 'FPGA_Controller', 1);          % ball distance s (m)
% disturbance force on the ball (along the track)
if live   % force set by a dashboard push button
    add_block('simulink/Sources/Constant', [mdl '/Disturbance'], 'Value', '0', ...
        'SampleTime', '0.001', 'Position', [380 330 420 360]);
else
    add_block('simulink/Sources/Pulse Generator', [mdl '/Disturbance'], ...
        'PulseType', 'Time based', 'Amplitude', num2str(scn.dist_N), 'Period', '1000', ...
        'PulseWidth', num2str(100*scn.dist_dur/1000), 'PhaseDelay', num2str(scn.dist_t), ...
        'Position', [380 330 420 360]);
end
lnk(mdl, 'Disturbance', 1, 'Plant', 2);

% logging
names = {'s_m','alpha_rad','theta_rad','tau_Nm','pulse_us'};
for k = 1:numel(names)
    tw = [mdl '/log_' names{k}];
    add_block('simulink/Sinks/To Workspace', tw, 'VariableName', names{k}, ...
        'SaveFormat', 'Timeseries', 'Position', [800 60*k 870 60*k+25]);
    lnk(mdl, 'Plant', k, ['log_' names{k}], 1);
end
cn = {'servo_us','pos_mm','setpoint','sflags'};   % = controller outport order
for k = 1:numel(cn)
    tw = [mdl '/log_' cn{k}];
    add_block('simulink/Sinks/To Workspace', tw, 'VariableName', cn{k}, ...
        'SaveFormat', 'Timeseries', 'Decimation', '100', 'Position', [450 400+50*k 520 425+50*k]);
    lnk(mdl, 'FPGA_Controller', k, ['log_' cn{k}], 1);
end
try, set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off'); catch, end
save_system(mdl, f);
fprintf('built %s\n', f);
end

% =========================================================================
function build_controller(ct, M, P, scn, Ts)
add_block('simulink/Sources/In1', [ct '/s_ball'], 'Position', [20 100 50 114]);
outs = {'servo_us','pos_mm','setpoint','sflags'};
for k = 1:numel(outs)
    add_block('simulink/Sinks/Out1', [ct '/' outs{k} '_o'], 'Position', [1100 60*k 1130 60*k+14]);
end
mlfb = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'simulink', 'mlfb');
here = fileparts(mfilename('fullpath'));
ws = get_param(bdroot(ct), 'ModelWorkspace');
prm = struct('P_TS', Ts, 'P_R', M.ball.r, 'P_C', M.sensor.c, 'P_NOISE_MM', M.sensor.noise_mm, ...
    'P_LAT_US', M.sensor.lat_us, 'P_SEED', 12345, 'P_MM_PER_US_Q16', P.MM_PER_US_Q16, ...
    'P_MIN_MM', P.MIN_MM, 'P_MAX_MM', P.MAX_MM);
fn = fieldnames(prm);
for k = 1:numel(fn), assignin(ws, fn{k}, prm.(fn{k})); end

info = struct('path', {}, 'in', {}, 'out', {});
info(end+1) = bb_add_mlfb(bdroot(ct), [ct '/hcsr04_model'], fileread(fullfile(here, 'hcsr04_model.m')), [200 60]);
info(end+1) = bb_add_mlfb(bdroot(ct), [ct '/pos_filter'], fileread(fullfile(mlfb, 'pos_filter.m')), [420 60]);
info(end+1) = bb_add_mlfb(bdroot(ct), [ct '/pid_ctrl'], fileread(fullfile(mlfb, 'pid_ctrl.m')), [640 60]);
for k = 1:numel(info), set_param(info(k).path, 'SystemSampleTime', num2str(Ts)); end
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [ct '/hcsr04_model']);
d = ch.find('-isa', 'Stateflow.Data', 'Name', 's_ball'); d.DataType = 'double';

% register file = constants (power-up defaults, overridable from the scenario)
R = P.DEF;
f = fieldnames(scn.reg);
for k = 1:numel(f), R.(f{k}) = scn.reg.(f{k}); end
regs = {'kp','KP'; 'ki','KI'; 'kd','KD'; 'center','CENTER'; 'smin','SMIN'; 'smax','SMAX'; ...
        'mode','MODE'; 'manual_us','MANUAL'; 'dshift','DSHIFT'; 'ilim','ILIM'; 'trig_ms','TRIG_MS'};
y = 300;
for k = 1:size(regs, 1)
    b = [ct '/' regs{k,1}];
    add_block('simulink/Sources/Constant', b, 'Value', num2str(R.(regs{k,2})), ...
        'OutDataTypeStr', 'uint16', 'SampleTime', num2str(Ts), 'Position', [40 y 110 y+20]);
    info(end+1) = struct('path', b, 'in', {{}}, 'out', {regs(k,1)}); %#ok<AGROW>
    y = y + 35;
end
b = [ct '/int_rst'];
add_block('simulink/Sources/Constant', b, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
    'SampleTime', num2str(Ts), 'Position', [40 y 110 y+20]);
info(end+1) = struct('path', b, 'in', {{}}, 'out', {{'int_rst'}});
% setpoint schedule: piecewise constant from scn.sp = [t0 v0; t1 v1; ...]
b = [ct '/setpoint_sched'];
live = isfield(scn, 'live') && scn.live;
if live   % live setpoint: a tunable constant driven by the dashboard slider
    add_block('simulink/Sources/Constant', b, 'Value', num2str(scn.sp(1,2)), ...
        'SampleTime', '0.01', 'Position', [80 y+60 130 y+80]);
else
    add_block('simulink/Sources/Clock', [ct '/clk'], 'Position', [20 y+60 50 y+80]);
    add_block('simulink/Lookup Tables/1-D Lookup Table', b, 'Table', mat2str(scn.sp(:,2)'), ...
        'BreakpointsForDimension1', mat2str(scn.sp(:,1)'), 'InterpMethod', 'Flat', ...
        'ExtrapMethod', 'Clip', 'Position', [80 y+60 130 y+80]);
end
add_block('simulink/Signal Attributes/Data Type Conversion', [ct '/sp_u16'], ...
    'OutDataTypeStr', 'uint16', 'Position', [160 y+60 200 y+80]);
add_block('simulink/Signal Attributes/Rate Transition', [ct '/sp_rt'], ...
    'OutPortSampleTime', num2str(Ts), 'Position', [220 y+60 250 y+80]);
if ~live, lnk(ct, 'clk', 1, 'setpoint_sched', 1); end
lnk(ct, 'setpoint_sched', 1, 'sp_u16', 1);
lnk(ct, 'sp_u16', 1, 'sp_rt', 1);
info(end+1) = struct('path', [ct '/sp_rt'], 'in', {{}}, 'out', {{'setpoint'}});
% plant input: ball distance (continuous) -> controller rate
add_block('simulink/Signal Attributes/Rate Transition', [ct '/s_rt'], ...
    'OutPortSampleTime', num2str(Ts), 'Position', [90 100 120 114]);
lnk(ct, 's_ball', 1, 's_rt', 1);
info(end+1) = struct('path', [ct '/s_rt'], 'in', {{}}, 'out', {{'s_ball'}});
% outputs
info(end+1) = struct('path', [ct '/servo_us_o'], 'in', {{'servo_us'}}, 'out', {{}});
info(end+1) = struct('path', [ct '/pos_mm_o'],   'in', {{'pos_mm'}},   'out', {{}});
info(end+1) = struct('path', [ct '/setpoint_o'], 'in', {{'setpoint'}}, 'out', {{}});
info(end+1) = struct('path', [ct '/sflags_o'],   'in', {{'sflags'}},   'out', {{}});
% unused controller outputs -> terminators
for k = 1:numel(info)
    for j = 1:numel(info(k).out)
        nm = info(k).out{j};
        if any(strcmp(nm, {'err_mm','iterm','dterm','ctl_done'}))
            t = [ct '/term_' nm];
            add_block('simulink/Sinks/Terminator', t, 'Position', [900 500+30*j 920 515+30*j]);
            info(end+1) = struct('path', t, 'in', {{nm}}, 'out', {{}}); %#ok<AGROW>
        end
    end
end
autowire(ct, info);
end

% =========================================================================
function build_plant(pl, M, scn)
s = pl;
add_block('simulink/Sources/In1', [s '/pulse_us'], 'Position', [20 600 50 614]);
add_block('simulink/Sources/In1', [s '/F_dist'],   'Position', [20 760 50 774]);
outs = {'s_m','alpha_rad','theta_rad','tau_Nm','pulse_o'};
for k = 1:numel(outs)
    add_block('simulink/Sinks/Out1', [s '/' outs{k}], 'Position', [1500 60*k 1530 60*k+14]);
end
c = @(a, ap, b, bp) smh.c(s, [s '/' a], ap, [s '/' b], bp);
grey = [0.6 0.6 0.65]; pla = [0.15 0.45 0.8]; steel = [0.75 0.75 0.78];

% ---- world, config ----
smh.add(s, 'world', 'W', [80 80]);
smh.add(s, 'mech', 'Mech', [80 10]);
smh.add(s, 'solver', 'Solv', [80 160]);
c('W','R1','Mech','R1'); c('W','R1','Solv','R1');

% ---- base and towers (fixed) ----
smh.rt(s, 'BaseFr', [200 20], [M.base.x 0 -M.base.size(3)/2]);
smh.solid(s, 'brick', 'Base', [300 20], M.base.size, struct('rho', 700), [0.85 0.8 0.7]);
c('W','R1','BaseFr','L1'); c('BaseFr','R1','Base','R1');
for k = [-1 1]
    nm = sprintf('Tower%d', (k+3)/2);
    y = k*(M.tower.gap/2 + M.tower.w/2);
    smh.rt(s, [nm 'Fr'], [200 60+40*k], [0 y M.pivot.h/2]);
    smh.solid(s, 'brick', nm, [300 60+40*k], [M.tower.d M.tower.w M.pivot.h+0.01], struct('rho', 620), pla);
    c('W','R1',[nm 'Fr'],'L1'); c([nm 'Fr'],'R1',nm,'R1');
end

% ---- beam pivot ----
smh.rt(s, 'PivotFr', [200 220], [0 0 M.pivot.h], 'X', -90);      % joint z = world y
smh.add(s, 'revolute', 'Pivot', [300 220], 'SensePosition', 'on', ...
    'DampingCoefficient', '0.002', 'DampingCoefficientUnits', 'N*m/(rad/s)');
smh.rt(s, 'BeamFr', [400 220], [0 0 0], 'X', 90);                 % beam frame: x along, z up
c('W','R1','PivotFr','L1'); c('PivotFr','R1','Pivot','L1'); c('Pivot','R1','BeamFr','L1');

% beam body: two rails + cradle + sensor + end stop + link bracket
for k = [-1 1]
    nm = sprintf('Rail%d', (k+3)/2);
    smh.rt(s, [nm 'Fr'], [500 160+40*k], [0 k*M.rail.spacing/2 M.rail.z], 'Y', 90);
    smh.solid(s, 'cyl', nm, [600 160+40*k], [M.rail.r M.rail.len], struct('rho', M.rail.rho), steel);
    c('BeamFr','R1',[nm 'Fr'],'L1'); c([nm 'Fr'],'R1',nm,'R1');
end
smh.rt(s, 'CradleFr', [500 260], [0 0 M.beam.z]);
smh.solid(s, 'brick', 'Cradle', [600 260], [M.beam.len M.beam.w M.beam.h], struct('rho', M.beam.rho), pla);
c('BeamFr','R1','CradleFr','L1'); c('CradleFr','R1','Cradle','R1');
sb = M.sensor.board;
smh.rt(s, 'SensorFr', [500 300], [M.sensor.x_face-sb(3)/2 0 M.ball.zc], 'Y', 90);
smh.solid(s, 'brick', 'HC_SR04', [600 300], [sb(2) sb(1) sb(3)], struct('m', M.sensor.m), [0.1 0.3 0.7]);
c('BeamFr','R1','SensorFr','L1'); c('SensorFr','R1','HC_SR04','R1');
smh.rt(s, 'StopFr', [500 340], [M.rail.len/2-0.004 0 M.ball.zc-0.01]);
smh.solid(s, 'brick', 'EndStop', [600 340], [0.008 M.beam.w 0.03], struct('rho', 620), pla);
c('BeamFr','R1','StopFr','L1'); c('StopFr','R1','EndStop','R1');
smh.rt(s, 'BracketFr', [500 380], [M.link.attach_x M.link.y/2 M.link.attach_z/2]);
smh.solid(s, 'brick', 'LinkBracket', [600 380], [0.012 M.link.y+0.006 abs(M.link.attach_z)+0.006], struct('rho', 620), pla);
c('BeamFr','R1','BracketFr','L1'); c('BracketFr','R1','LinkBracket','R1');

% ---- ball on the track ----
smh.rt(s, 'TrackFr', [500 440], [M.sensor.x_face 0 M.ball.zc], 'Y', 90);   % joint z along +x
smh.add(s, 'prismatic', 'BallJ', [620 440], 'SensePosition', 'on', 'SenseVelocity', 'on', ...
    'TorqueActuationMode', 'InputTorque', ...
    'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', ...
    'PositionTargetValue', num2str(scn.s0), 'PositionTargetValueUnits', 'm', ...
    'VelocityTargetSpecify', 'on', 'VelocityTargetValue', '0', 'VelocityTargetValueUnits', 'm/s', ...
    'DampingCoefficient', '1e-4', 'DampingCoefficientUnits', 'N/(m/s)', ...
    'LowerLimitSpecify', 'on', 'LowerLimitBound', num2str(M.track.s_min), 'LowerLimitBoundUnits', 'm', ...
    'LowerLimitStiffness', '500', 'LowerLimitStiffnessUnits', 'N/m', ...
    'LowerLimitDamping', '0.5', 'LowerLimitDampingUnits', 'N/(m/s)', ...
    'UpperLimitSpecify', 'on', 'UpperLimitBound', num2str(M.track.s_max), 'UpperLimitBoundUnits', 'm', ...
    'UpperLimitStiffness', '500', 'UpperLimitStiffnessUnits', 'N/m', ...
    'UpperLimitDamping', '0.5', 'UpperLimitDampingUnits', 'N/(m/s)');
smh.solid(s, 'sphere', 'Ball', [740 440], M.ball.r, struct('mc', M.ball.meff, 'I', M.ball.I*[1 1 1]), [1 0.55 0]);
c('BeamFr','R1','TrackFr','L1'); c('TrackFr','R1','BallJ','L1'); c('BallJ','R1','Ball','R1');
% rolling: gravity acts only on the real mass (see test_rolling.m)
fc = smh.add(s, 'extforce', 'Fcomp', [740 520], 'EnableForce', 'on', 'ForceResolutionFrame', 'World');
add_block('simulink/Sources/Constant', [s '/Fcomp_val'], 'Value', ...
    mat2str([0 0 (M.ball.meff - M.ball.m)*9.80665], 10), 'Position', [560 520 610 540]);
cv = smh.add(s, 'sl2ps', 'Fcomp_ps', [640 520]); set_param(cv, 'Unit', 'N');
lnkp(s, 'Fcomp_val', cv); smh.c(s, cv, 'R1', fc, 'L1'); smh.c(s, fc, 'R1', [s '/Ball'], 'R1');
% disturbance force along the track
cv = smh.add(s, 'sl2ps', 'Fdist_ps', [200 760]); set_param(cv, 'Unit', 'N');
lnkp(s, 'F_dist', cv);

% ---- servo: internal position loop (torque limited) ----
smh.rt(s, 'ServoFr', [200 600], [M.servo.x M.link.y M.servo.z], 'X', -90);
smh.add(s, 'revolute', 'Servo', [320 600], 'SensePosition', 'on', 'SenseVelocity', 'on', ...
    'TorqueActuationMode', 'InputTorque', ...
    'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', 'PositionTargetValue', '0', ...
    'DampingCoefficient', num2str(M.servo.tau_max/M.servo.w_max), ...
    'DampingCoefficientUnits', 'N*m/(rad/s)');
smh.rt(s, 'HornFr', [440 600], [0 0 0], 'X', 90);                 % back to world-aligned
smh.rt(s, 'HornC', [540 640], [-M.horn.r/2 0 0]);
smh.solid(s, 'brick', 'Horn', [640 640], [M.horn.r+0.010 0.006 0.005], ...
    struct('mc', 0.004, 'I', [1e-7 M.servo.J 1e-7]), [0.95 0.95 0.95]);
smh.rt(s, 'HornTip', [540 600], [-M.horn.r 0 0]);
c('W','R1','ServoFr','L1'); c('ServoFr','R1','Servo','L1'); c('Servo','R1','HornFr','L1');
c('HornFr','R1','HornC','L1'); c('HornC','R1','Horn','R1'); c('HornFr','R1','HornTip','L1');
% servo body (fixed)
smh.rt(s, 'ServoBodyFr', [200 680], [M.servo.x+0.010 M.link.y+0.005+M.servo.body(2)/2 M.servo.z-M.servo.body(3)/2+0.010]);
smh.solid(s, 'brick', 'MG996R', [320 680], M.servo.body, struct('m', 0.055), [0.1 0.1 0.1]);
c('W','R1','ServoBodyFr','L1'); c('ServoBodyFr','R1','MG996R','R1');

% ---- pushrod with ball links (closes the four-bar loop) ----
smh.add(s, 'spherical', 'LinkBot', [700 600]);
smh.rt(s, 'RodC', [800 600], [0 0 M.rod.len/2]);
smh.solid(s, 'cyl', 'Rod', [900 600], [M.rod.r M.rod.len], struct('m', M.rod.m), steel);
smh.rt(s, 'RodTop', [800 680], [0 0 M.rod.len]);
smh.add(s, 'spherical', 'LinkTop', [900 680]);
smh.rt(s, 'AttachFr', [500 480], [M.link.attach_x M.link.y M.link.attach_z]);
c('HornTip','R1','LinkBot','L1'); c('LinkBot','R1','RodC','L1'); c('RodC','R1','Rod','R1');
c('LinkBot','R1','RodTop','L1'); c('RodTop','R1','LinkTop','R1');
c('BeamFr','R1','AttachFr','L1'); c('AttachFr','R1','LinkTop','L1');

% ---- servo electronics model (Simulink side) ----
%   theta_ref = dir * (pulse - center) * deg/us ;  tau = sat(kp*(theta_ref - theta))
add_block('simulink/Math Operations/Bias', [s '/p0'], 'Bias', num2str(-M.servo.center_us), 'Position', [90 600 120 614]);
add_block('simulink/Math Operations/Gain', [s '/us2rad'], 'Gain', ...
    num2str(M.servo.dir*M.servo.deg_per_us*pi/180, 10), 'Position', [140 600 170 614]);
add_block('simulink/Math Operations/Sum', [s '/err'], 'Inputs', '+-', 'Position', [190 540 210 560]);
add_block('simulink/Math Operations/Gain', [s '/kp'], 'Gain', num2str(M.servo.kp), 'Position', [230 540 260 560]);
add_block('simulink/Discontinuities/Saturation', [s '/tau_lim'], 'UpperLimit', num2str(M.servo.tau_max), ...
    'LowerLimit', num2str(-M.servo.tau_max), 'Position', [280 540 310 560]);
cvt = smh.add(s, 'sl2ps', 'tau_ps', [330 520]); set_param(cvt, 'Unit', 'N*m');
lnk(s, 'pulse_us', 1, 'p0', 1); lnk(s, 'p0', 1, 'us2rad', 1); lnk(s, 'us2rad', 1, 'err', 1);
lnk(s, 'err', 1, 'kp', 1); lnk(s, 'kp', 1, 'tau_lim', 1); lnkp(s, 'tau_lim', cvt);
lnk(s, 'pulse_us', 1, 'pulse_o', 1);

% sensing outputs
qS  = smh.add(s, 'ps2sl', 'theta_sl', [450 700]); set_param(qS, 'Unit', 'rad');
qP  = smh.add(s, 'ps2sl', 'alpha_sl', [450 240]); set_param(qP, 'Unit', 'rad');
qB  = smh.add(s, 'ps2sl', 's_sl',     [750 400]); set_param(qB, 'Unit', 'm');
ports_connect(s, M);
lnk(s, 'theta_sl', 1, 'err', 2);
lnk(s, 'theta_sl', 1, 'theta_rad', 1);
lnk(s, 'alpha_sl', 1, 'alpha_rad', 1);
lnk(s, 's_sl', 1, 's_m', 1);
lnk(s, 'tau_lim', 1, 'tau_Nm', 1);              % commanded = applied motor torque
end

function ports_connect(s, ~)
% joint actuation / sensing ports (looked up by count to stay version-robust)
J = get_param([s '/Servo'], 'PortHandles');   % L: B, t ; R: F, q, w
B = get_param([s '/BallJ'], 'PortHandles');   % L: B, f ; R: F, p, v
P = get_param([s '/Pivot'], 'PortHandles');   % L: B    ; R: F, q
g = @(n) get_param([s '/' n], 'PortHandles');
add_line(s, g('tau_ps').RConn(1), J.LConn(2), 'autorouting', 'on');
add_line(s, J.RConn(2), g('theta_sl').LConn(1), 'autorouting', 'on');
add_line(s, g('Fdist_ps').RConn(1), B.LConn(2), 'autorouting', 'on');
add_line(s, B.RConn(2), g('s_sl').LConn(1), 'autorouting', 'on');
add_line(s, P.RConn(2), g('alpha_sl').LConn(1), 'autorouting', 'on');
end

% =========================================================================
function lnk(sys, a, ap, b, bp)
A = get_param([sys '/' a], 'PortHandles'); Bh = get_param([sys '/' b], 'PortHandles');
add_line(sys, A.Outport(ap), Bh.Inport(bp), 'autorouting', 'on');
end

function lnkp(sys, a, cvPath)
% Simulink signal -> Simulink-PS converter input
A = get_param([sys '/' a], 'PortHandles'); C = get_param(cvPath, 'PortHandles');
add_line(sys, A.Outport(1), C.Inport(1), 'autorouting', 'on');
end

function autowire(sys, info)
drivers = containers.Map();
for k = 1:numel(info)
    for j = 1:numel(info(k).out)
        nm = info(k).out{j};
        assert(~isKey(drivers, nm), 'signal %s driven twice', nm);
        drivers(nm) = [k j];
    end
end
for k = 1:numel(info)
    ph = get_param(info(k).path, 'PortHandles');
    for j = 1:numel(info(k).in)
        nm = info(k).in{j};
        assert(isKey(drivers, nm), 'no driver for input %s of %s', nm, info(k).path);
        kj = drivers(nm);
        sph = get_param(info(kj(1)).path, 'PortHandles');
        l = add_line(sys, sph.Outport(kj(2)), ph.Inport(j), 'autorouting', 'on');
        set_param(l, 'Name', nm);
    end
end
end
