function test_bb_live()
%TEST_BB_LIVE  Hands-off test of the interactive model: starts bb_live with
%   the Run command (real-time paced, like pressing Run) and uses MATLAB
%   timers to do what a user would do on the dashboard, writing the same block
%   parameters the widgets are bound to:
%     t =  6 s  setpoint slider 330 -> 220 mm
%     t = 12 s  Kp knob 768 -> 512
%     t = 16 s  "Push ball" pressed for 0.2 s
%     t = 24 s  stop, then check pacing and tracking, and plot docs/live_test.png
%   Returns immediately; the result is printed when the last timer fires.
here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(fileparts(here), 'simulink'));
mdl = 'bb_live';
if ~bdIsLoaded(mdl), open_system(fullfile(here, [mdl '.slx'])); end
ct = [mdl '/FPGA_Controller'];
set_param([ct '/setpoint_sched'], 'Value', '330');
set_param([ct '/kp'], 'Value', num2str(bb_params().DEF.KP));
set_param([mdl '/Disturbance'], 'Value', '0');

ev = {6,    @() set_param([ct '/setpoint_sched'], 'Value', '220')
      12,   @() set_param([ct '/kp'], 'Value', '512')
      16,   @() set_param([mdl '/Disturbance'], 'Value', '0.006')
      16.2, @() set_param([mdl '/Disturbance'], 'Value', '0')
      24,   @() finish(mdl)};
set_param(mdl, 'SimulationCommand', 'start');
% wait (non-blocking) for the first simulation step, then arm the timers on sim time
t = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.1, 'TasksToExecute', 2000, ...
    'TimerFcn', @(tm, ~) poll(tm, mdl, ev));
t.UserData = struct('k', 1, 'wall0', [], 'sim0', []);
start(t);
fprintf('bb_live started; scripted dashboard actions at 6/12/16 s, result at 24 s\n');
end

function poll(tm, mdl, ev)
u = tm.UserData;
if ~strcmp(get_param(mdl, 'SimulationStatus'), 'running'), return; end
ts = get_param(mdl, 'SimulationTime');
if isempty(u.wall0) && ts > 0.5, u.wall0 = tic; u.sim0 = ts; end
while u.k <= size(ev, 1) && ts >= ev{u.k, 1}
    if u.k == size(ev, 1)
        rt = (ts - u.sim0) / toc(u.wall0);
        assignin('base', 'bb_live_rt', rt);
        stop(tm); delete(tm);
    end
    feval(ev{u.k, 2});
    u.k = u.k + 1;
end
if isvalid(tm), tm.UserData = u; end
end

function finish(mdl)
set_param(mdl, 'SimulationCommand', 'stop');
t = timer('StartDelay', 2, 'TimerFcn', @(tm, ~) report(tm, mdl));
start(t);
end

function report(tm, mdl)
delete(tm);
o = evalin('base', 'out');
M = bb_mech_params();
t = o.s_m.Time; x = (squeeze(o.s_m.Data) - M.ball.r)*1000;
w = @(a, b) t > a & t < b;
rt = evalin('base', 'bb_live_rt');
e220 = mean(x(w(10, 12))) - 220;
kick = max(x(w(16, 20))) - 220;
fin  = mean(x(w(22, 24))) - 220;
ok = abs(rt - 1) < 0.05 && abs(e220) < 10 && kick > 30 && abs(fin) < 15;
fprintf(['bb_live: real-time factor %.3f | offset after live setpoint change %+.1f mm | ' ...
    'push deflection %.0f mm | final offset %+.1f mm -> %s\n'], rt, e220, kick, fin, verdict(ok));
assignin('base', 'bb_live_result', struct('rt', rt, 'e220', e220, 'kick', kick, 'fin', fin, 'pass', ok));
f = figure('Visible', 'off', 'Position', [100 100 900 520]);
subplot(2, 1, 1); plot(t, x); hold on; grid on
stairs(o.setpoint.Time, double(squeeze(o.setpoint.Data)), 'k--');
xline(12, ':', 'Kp 3\rightarrow2'); xline(16, ':', 'push');
ylabel('mm from sensor'); legend('ball (Simscape)', 'setpoint (changed live)', 'Location', 'best');
title(sprintf('bb\\_live: dashboard actions during a real-time paced run (%.2fx)', rt));
subplot(2, 1, 2); stairs(o.pulse_us.Time, squeeze(o.pulse_us.Data)); grid on
ylabel('servo pulse, us'); xlabel('s');
root = fileparts(fileparts(mfilename('fullpath')));
exportgraphics(f, fullfile(root, 'docs', 'live_test.png'), 'Resolution', 110);
close(f);
end

function s = verdict(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
