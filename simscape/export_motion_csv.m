function export_motion_csv(scn, fps)
%EXPORT_MOTION_CSV  Run bb_simscape and write the rig motion at `fps` frames
%   per second to cad/freecad/bb_motion_simscape.csv for the FreeCAD replay:
%     t [s], theta_deg (servo horn, + = tip up), alpha_deg (beam, + = far end
%     down), s_mm (ball centre from the sensor face), setpoint_mm
if nargin < 1, scn = bb_scenario(); end
if nargin < 2, fps = 30; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(here, fullfile(root, 'simulink'));
bdclose('all');
build_bb_simscape(scn);
o = sim('bb_simscape');
t = (0:1/fps:scn.T)';
th = interp1(o.theta_rad.Time, squeeze(o.theta_rad.Data), t, 'linear', 'extrap')*180/pi;
al = interp1(o.alpha_rad.Time, squeeze(o.alpha_rad.Data), t, 'linear', 'extrap')*180/pi;
s  = interp1(o.s_m.Time, squeeze(o.s_m.Data), t, 'linear', 'extrap')*1000;
sp = interp1(o.setpoint.Time, double(squeeze(o.setpoint.Data)), t, 'previous', 'extrap');
T = table(t, th, al, s, sp, 'VariableNames', {'t', 'theta_deg', 'alpha_deg', 's_mm', 'setpoint_mm'});
f = fullfile(root, 'cad', 'freecad', 'bb_motion_simscape.csv');
writetable(T, f);
fprintf('wrote %s (%d frames)\n', f, height(T));
end
