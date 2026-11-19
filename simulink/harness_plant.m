function [echo_in, uart_rxd, ball_mm, pulse_us] = harness_plant(trig_out, servo_out, P_DT, P_CMD, P_CMD_T0, P_UART_DIV)
%HARNESS_PLANT  Test-bench only (not converted to HDL).
%   * HC-SR04 model: 200 us after the trigger falls, echo goes high for
%     2*x/343 m/s.
%   * Ball-and-beam plant (bb_plant_step equations) driven by the measured
%     servo pulse width.
%   * UART host: sends the byte list P_CMD at 115200 8N1 starting at P_CMD_T0.
%#codegen
persistent t trig_d echo_t0 echo_t1 x v th hi_t pw bi bitpos bitcnt line
if isempty(t)
    t = 0; trig_d = false; echo_t0 = -1; echo_t1 = -1;
    x = 0.15; v = 0; th = 0; hi_t = 0; pw = 1750;
    bi = 1; bitpos = -1; bitcnt = 0; line = true;
end
pl.center_us = 1750; pl.rad_per_us = 0.1*0.09*pi/180; pl.tau_servo = 0.03;
pl.slew_us_per_s = 6000; pl.xmin = 0.04; pl.xmax = 0.60;

% outputs from current state
echo_in  = (t >= echo_t0) && (t < echo_t1);
uart_rxd = line;
ball_mm  = x*1000;
pulse_us = pw;

% ---- sensor ----
if trig_d && ~trig_out
    echo_t0 = t + 200e-6;
    echo_t1 = echo_t0 + 2*x/343;
end
trig_d = trig_out;

% ---- servo pulse measurement ----
if servo_out
    hi_t = hi_t + P_DT;
elseif hi_t > 0
    pw = hi_t*1e6; hi_t = 0;
end

% ---- plant (bb_plant_step inlined) ----
target = pw - pl.center_us;
dth = (target - th)/pl.tau_servo*P_DT;
maxd = pl.slew_us_per_s*P_DT;
dth = max(-maxd, min(maxd, dth));
th = th + dth;
a = (5/7)*9.81*sin(pl.rad_per_us*th);
v = v + a*P_DT; x = x + v*P_DT;
if x < pl.xmin, x = pl.xmin; v = 0; end
if x > pl.xmax, x = pl.xmax; v = 0; end

% ---- UART host transmitter (clock-count based, like the RTL) ----
if t >= P_CMD_T0 && bi <= numel(P_CMD)
    if bitpos < 0
        bitpos = 0; bitcnt = 0;
    end
    if bitcnt == 0
        if bitpos == 0
            line = false;                               % start
        elseif bitpos <= 8
            line = bitand(bitshift(P_CMD(bi), -(bitpos-1)), 1) ~= 0;
        else
            line = true;                                % stop + 1 bit gap
        end
    end
    bitcnt = bitcnt + 1;
    if bitcnt >= P_UART_DIV
        bitcnt = 0; bitpos = bitpos + 1;
        if bitpos > 10
            bitpos = -1; bi = bi + 1;
        end
    end
end
t = t + P_DT;
end
