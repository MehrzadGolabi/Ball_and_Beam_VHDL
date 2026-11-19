function m = bb_metrics(o, scn)
%BB_METRICS  Performance numbers from a bb_simscape run (surface distance, mm).
M = bb_mech_params();
t = o.s_m.Time; x = (squeeze(o.s_m.Data) - M.ball.r)*1000;
sp1 = scn.sp(1,2); sp2 = scn.sp(end,2); t2 = scn.sp(end,1); td = scn.dist_t;
win = @(a, b) t > a & t < b;
m.off1 = mean(x(win(t2-2, t2))) - sp1;              % before the setpoint change
m.off2 = mean(x(win(td-2, td))) - sp2;              % before the push
m.off3 = mean(x(win(scn.T-2, scn.T))) - sp2;        % end
m.ov1  = max(x(t < t2)) - sp1;
m.ov2  = sp2 - min(x(t > t2 & t < td));
m.std  = std(x(win(scn.T-2, scn.T)));
ref = sp1*(t < t2) + sp2*(t >= t2);
m.iae  = trapz(t, abs(x - ref));
tau = squeeze(o.tau_Nm.Data);
m.tausat = mean(abs(tau) > 0.999*bb_mech_params().servo.tau_max);
a = squeeze(o.alpha_rad.Data)*180/pi;
m.alpha = [min(a) max(a)];
m.s = [min(x) max(x)] + M.ball.r*1000;                % centre range, mm from face
th = squeeze(o.theta_rad.Data)*180/pi;
m.theta = [min(th) max(th)];
end
