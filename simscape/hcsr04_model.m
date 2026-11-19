function [echo_us, meas_valid, meas_timeout] = hcsr04_model(s_ball, trig_ms, P_TS, P_R, P_C, P_NOISE_MM, P_LAT_US, P_SEED)
%HCSR04_MODEL  Behaviour of HC-SR04 + the FPGA hcsr04_if block, at the
%   controller step P_TS. Every trig_ms (>= 10 ms) a measurement starts;
%   it completes 12 us (trigger) + P_LAT_US + echo width later, exactly when
%   hcsr04_if would raise meas_valid. echo_us is quantised to 1 us like the
%   FPGA counter. Noise: deterministic Gaussian (LCG + Box-Muller).
%   s_ball: distance sensor face -> ball centre [m].
%#codegen
persistent t t_next t_done pend val lcg
if isempty(t)
    t = 0; t_next = 0; t_done = 0; pend = false; val = uint16(0); lcg = uint32(P_SEED);
end
echo_us = val;
meas_valid = false;
meas_timeout = false;

period = max(10, double(trig_ms)) * 1e-3;
if ~pend && t >= t_next
    [n, lcg] = gauss(lcg);
    d = s_ball - P_R + P_NOISE_MM*1e-3*n;           % to the ball surface
    w = 2*d/P_C;
    t_done = t + 12e-6 + P_LAT_US*1e-6 + w;
    val = uint16(max(0, floor(w*1e6)));
    pend = true;
    t_next = t + period;
end
if pend && t >= t_done
    echo_us = val;
    meas_valid = true;
    pend = false;
end
t = t + P_TS;
end

function [n, s] = gauss(s)
s = uint32(mod(1664525*double(s) + 1013904223, 4294967296));
u1 = (double(s) + 1) / 4294967297;
s = uint32(mod(1664525*double(s) + 1013904223, 4294967296));
u2 = double(s) / 4294967296;
n = sqrt(-2*log(u1)) * cos(2*pi*u2);
end
