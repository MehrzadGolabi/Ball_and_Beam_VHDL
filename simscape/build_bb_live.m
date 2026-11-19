function mdl = build_bb_live()
%BUILD_BB_LIVE  Interactive version of bb_simscape: runs paced to real time
%   until you stop it. While it runs:
%     * Setpoint slider  (mm from the sensor)        -> controller setpoint
%     * Kp knob          (raw Q8.8, 768 = 3.0 us/mm) -> controller Kp register
%     * "Push ball"      button                      -> 6 mN force on the ball
%     * Dashboard scopes : ball position vs setpoint, servo pulse
%     * Mechanics Explorer shows the 3-D rig moving.
%   The controller blocks run at 2 kHz (instead of 10 kHz) for real-time
%   headroom; the 20 ms servo frame dominates the latency either way.
%
%   Open with:  build_bb_live; open_system('bb_live'); then press Run.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(fileparts(here), 'simulink'));
scn = bb_scenario();
scn.live = true;
scn.Ts   = 5e-4;
scn.T    = 1e9;
mdl = build_bb_simscape(scn);

set_param(mdl, 'StopTime', 'inf', 'EnablePacing', 'on', 'PacingRate', '1');
try, set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'on'); catch, end
% bounded logging for an endless run
tw = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'ToWorkspace');
for k = 1:numel(tw)
    set_param(tw{k}, 'MaxDataPoints', '20000');
    if ~strcmp(get_param(tw{k}, 'Decimation'), '100'), set_param(tw{k}, 'Decimation', '50'); end
end

L = 'simulink_hmi_blocks/';
ct = [mdl '/FPGA_Controller'];
% ---- controls ----
b = [mdl '/Setpoint (mm)'];
add_block([L 'Slider'], b, 'Position', [40 420 280 470]);
set_param(b, 'Limits', [60 -1 470]);
bind_param(b, [ct '/setpoint_sched'], 'Value');

b = [mdl '/Kp x256'];
add_block([L 'Knob'], b, 'Position', [40 500 160 620]);
set_param(b, 'Limits', [0 -1 2048]);
bind_param(b, [ct '/kp'], 'Value');

b = [mdl '/Push ball'];
add_block([L 'Push Button'], b, 'Position', [200 520 290 570], 'ButtonText', 'Push ball', 'OnValue', '0.006');
bind_param(b, [mdl '/Disturbance'], 'Value');

% ---- displays ----
b = [mdl '/Position vs setpoint'];
add_block([L 'Dashboard Scope'], b, 'Position', [900 400 1300 620], 'TimeSpan', '15');
set_param(b, 'YLimits', [0 500]);
bind_signals(b, ct, [2 3]);                 % pos_mm, setpoint
b = [mdl '/Servo pulse'];
add_block([L 'Dashboard Scope'], b, 'Position', [900 640 1300 800], 'TimeSpan', '15');
set_param(b, 'YLimits', [1000 2500]);
bind_signals(b, mdl, 0, 'pwm_frame');
b = [mdl '/pos_mm'];
add_block([L 'Display'], b, 'Position', [320 420 420 460]);
bind_signals(b, ct, 2);

save_system(mdl);
fprintf('built %s: open it and press Run (real-time paced, runs until Stop)\n', mdl);
end

function bind_param(dash, blk, prm)
s = Simulink.HMI.ParamSourceInfo;
s.BlockPath = Simulink.BlockPath(blk);
s.ParamName = prm;
set_param(dash, 'Binding', s);
end

function bind_signals(dash, src, ports, blkName)
if nargin >= 4, src = [src '/' blkName]; ports = 1; end
specs = Simulink.HMI.SignalSpecification.empty;
for p = ports
    sp = Simulink.HMI.SignalSpecification;
    sp.BlockPath = Simulink.BlockPath(src);
    sp.OutputPortIndex = p;
    specs(end+1) = sp; %#ok<AGROW>
    ph = get_param(src, 'PortHandles');
    set_param(ph.Outport(p), 'DataLogging', 'on');   % dashboard blocks stream logged signals
end
% scopes take a cell array, single-signal widgets a scalar specification
if strcmp(get_param(dash, 'BlockType'), 'DashboardScope') || contains(get_param(dash, 'Name'), {'vs', 'pulse'})
    set_param(dash, 'Binding', num2cell(specs));
else
    set_param(dash, 'Binding', specs);
end
end
