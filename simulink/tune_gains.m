function best = tune_gains(doPlot)
%TUNE_GAINS  Sample-level closed-loop sim to choose default PID gains.
%   Ball starts at 150 mm, setpoint 330 mm, then setpoint steps to 250 mm.
%   Includes servo lag, sensor noise, 1 mm quantisation and median-of-3.
if nargin < 1, doPlot = true; end
P  = bb_params();
pl = bb_plant_defaults();
kps = [1 1.5 2 3 4 5 6];
kds = [20 30 40 50 70 90];
best.cost = inf;
rng(1);
for kp = kps
    for kd = kds
        R = P.DEF; R.KP = round(kp*256); R.KD = round(kd*256);
        [J, ok] = run(R, pl, P, false);
        if ok && J < best.cost
            best.cost = J; best.kp = kp; best.kd = kd; best.R = R;
        end
    end
end
fprintf('best Kp=%.2f Kd=%.1f cost=%.1f\n', best.kp, best.kd, best.cost);
rng(1);
[J, ok, log] = run(best.R, pl, P, true); %#ok<ASGLU>
best.log = log;
R = P.DEF; rng(1); [Jd, okd, logd] = run(R, pl, P, true);
fprintf('defaults (Kp=%.2f Kd=%.1f Ki=%.3f): cost=%.1f stable=%d, final err=%.1f mm, servo p-p last 2 s=%d us\n', ...
    R.KP/256, R.KD/256, R.KI/256, Jd, okd, logd.x(end)*1000-250, ...
    max(logd.u(end-50:end))-min(logd.u(end-50:end)));
if doPlot
    f = figure('Visible','off');
    subplot(2,1,1); plot(logd.t, logd.x*1000, logd.t, logd.sp); ylabel('mm'); grid on
    legend('ball','setpoint'); title('Sample-level closed loop with default gains')
    subplot(2,1,2); plot(logd.t, logd.u); ylabel('servo us'); xlabel('s'); grid on
    exportgraphics(f, fullfile(fileparts(mfilename('fullpath')), 'tune_defaults.png'));
    close(f);
end
end

function [J, ok, log] = run(R, pl, P, keep)
Ts = R.TRIG_MS/1000; dt = 1e-3; T = 12;
s.x = 0.15; s.v = 0; s.th = 0;
c = []; servo = R.CENTER; h = [150 150 150];
n = round(T/Ts); log.t = zeros(n,1); log.x = log.t; log.u = log.t; log.sp = log.t;
J = 0;
for k = 1:n
    t = k*Ts;
    if t > 6, R.SETPOINT = 250; end
    meas = round(s.x*1000 + pl.noise_mm*randn);
    h = [meas h(1:2)];
    pos = median(h);
    [servo, c] = bb_ctrl_ref(c, pos, R);
    for j = 1:round(Ts/dt)
        s = bb_plant_step(s, servo, dt, pl);
    end
    log.t(k) = t; log.x(k) = s.x; log.u(k) = servo; log.sp(k) = R.SETPOINT;
    J = J + abs(s.x*1000 - R.SETPOINT)*Ts + 0.01*abs(servo - R.CENTER)*Ts;
end
tail = log.x(end-round(2/Ts):end)*1000;
ok = all(abs(tail - 250) < 15) && all(log.x > pl.xmin + 0.005);
end
