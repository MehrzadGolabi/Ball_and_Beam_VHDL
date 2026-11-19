function m = run_verification(scn)
%RUN_VERIFICATION  Build bb_simscape, run the scenario, check pass criteria,
%   save docs/simscape_verification.png and export the CAD parameters.
%   Pass criteria (surface distance):
%     |offset| < 10 mm before the setpoint change, before the push, at the end
%     end-of-run noise std < 3 mm, no end-stop hit, beam within +/-10 deg,
%     servo torque saturated < 5 % of the time.
if nargin < 1, scn = bb_scenario(); end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(here, fullfile(root, 'simulink'));
bdclose('all');
build_bb_simscape(scn);
o = sim('bb_simscape');
m = bb_metrics(o, scn);
M = bb_mech_params();
chk = {
  'settled at first setpoint',  abs(m.off1) < 10
  'settled at second setpoint', abs(m.off2) < 10
  'recovered from push',        abs(m.off3) < 10
  'steady noise std < 3 mm',    m.std < 3
  'no end-stop contact',        m.s(1) > 1000*M.track.s_min + 1 && m.s(2) < 1000*M.track.s_max - 1
  'beam angle within 10 deg',   all(abs(m.alpha) < 10)
  'servo torque sat < 5 %',     m.tausat < 0.05
};
fprintf('offsets %+5.1f / %+5.1f / %+5.1f mm, overshoot %.1f / %.1f mm, std %.2f mm\n', ...
    m.off1, m.off2, m.off3, m.ov1, m.ov2, m.std);
fprintf('beam %.1f..%.1f deg, servo %.1f..%.1f deg, torque saturated %.1f %%\n', ...
    m.alpha, m.theta, 100*m.tausat);
ok = true;
for k = 1:size(chk, 1)
    fprintf('  [%s] %s\n', ternary(chk{k,2}, 'PASS', 'FAIL'), chk{k,1});
    ok = ok && chk{k,2};
end
m.pass = ok;
fprintf('SIMSCAPE VERIFICATION %s\n', ternary(ok, 'PASSED', 'FAILED'));
plot_run(o, scn, M, fullfile(root, 'docs', 'simscape_verification.png'));
export_cad_params();
end

function plot_run(o, scn, M, file)
f = figure('Visible', 'off', 'Position', [100 100 1000 800]);
tl = tiledlayout(f, 4, 1, 'TileSpacing', 'compact');
nexttile; hold on; grid on
t = o.s_m.Time; plot(t, (squeeze(o.s_m.Data) - M.ball.r)*1000, 'LineWidth', 1.2);
stairs(o.pos_mm.Time, double(squeeze(o.pos_mm.Data)), 'Color', [0.85 0.33 0.1 0.6]);
stairs(o.setpoint.Time, double(squeeze(o.setpoint.Data)), 'k--');
xline(scn.dist_t, ':', 'push'); ylabel('mm from sensor');
legend('ball (Simscape)', 'pos\_mm (controller)', 'setpoint', 'Location', 'best');
nexttile; grid on; hold on
stairs(o.pulse_us.Time, squeeze(o.pulse_us.Data)); ylabel('servo pulse, us');
nexttile; grid on; hold on
plot(o.alpha_rad.Time, squeeze(o.alpha_rad.Data)*180/pi);
plot(o.theta_rad.Time, squeeze(o.theta_rad.Data)*180/pi);
ylabel('deg'); legend('beam', 'servo horn', 'Location', 'best');
nexttile; grid on
plot(o.tau_Nm.Time, squeeze(o.tau_Nm.Data)); ylabel('servo torque, N m'); xlabel('s');
title(tl, 'Simscape Multibody plant + FPGA controller logic (pos\_filter, pid\_ctrl)');
if ~exist(fileparts(file), 'dir'), mkdir(fileparts(file)); end
exportgraphics(f, file, 'Resolution', 110);
close(f);
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
