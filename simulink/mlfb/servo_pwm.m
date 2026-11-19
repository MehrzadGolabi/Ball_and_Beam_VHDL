function servo_out = servo_pwm(servo_us, tick_us, P_PERIOD_US, P_ABS_MIN_US, P_ABS_MAX_US, P_INIT_US)
%SERVO_PWM  50 Hz servo pulse with 1 us resolution. The width is latched
%   at the start of each period (glitch-free) and hard-clamped to a safe range.
%#codegen
persistent cnt width out
if isempty(cnt)
    cnt = uint16(0); width = uint16(P_INIT_US); out = false;
end
servo_out = out;
if tick_us
    if cnt >= uint16(P_PERIOD_US - 1)
        cnt = uint16(0);
        w = servo_us;
        if w < uint16(P_ABS_MIN_US), w = uint16(P_ABS_MIN_US); end
        if w > uint16(P_ABS_MAX_US), w = uint16(P_ABS_MAX_US); end
        width = w;
    else
        cnt = cnt + uint16(1);
    end
end
out = cnt < width;
end
