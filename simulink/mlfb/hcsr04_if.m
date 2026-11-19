function [trig_out, echo_us, meas_valid, meas_timeout] = hcsr04_if(echo_s, tick_us, tick_ms, trig_ms, ...
    P_TRIG_US, P_RISE_TMO_US, P_ECHO_MAX_US)
%HCSR04_IF  Triggers the HC-SR04 every trig_ms (>=10 ms, and never before the
%   previous echo has finished) and measures the echo high time in us.
%   meas_valid pulses once per measurement attempt; meas_timeout marks a
%   missing / too-long echo (echo_us is then not a distance).
%#codegen
persistent st ms_cnt us_cnt echo_d trig_r out_us out_v out_to
if isempty(st)
    st = uint8(0); ms_cnt = uint16(0); us_cnt = uint16(0); echo_d = false;
    trig_r = false; out_us = uint16(0); out_v = false; out_to = false;
end
trig_out     = trig_r;
echo_us      = out_us;
meas_valid   = out_v;
meas_timeout = out_to;

out_v = false;
period = trig_ms;
if period < uint16(10)
    period = uint16(10);
end
if tick_ms && ms_cnt < uint16(1000)
    ms_cnt = ms_cnt + uint16(1);
end
rise = echo_s && ~echo_d;
fall = ~echo_s && echo_d;
echo_d = echo_s;

if st == uint8(0)                       % IDLE: wait for the period
    if ms_cnt >= period
        ms_cnt = uint16(0);
        us_cnt = uint16(0);
        trig_r = true;
        st = uint8(1);
    end
elseif st == uint8(1)                   % TRIG: hold trigger high
    if tick_us
        if us_cnt >= uint16(P_TRIG_US - 1)
            trig_r = false;
            us_cnt = uint16(0);
            st = uint8(2);
        else
            us_cnt = us_cnt + uint16(1);
        end
    end
elseif st == uint8(2)                   % WAIT for echo rising edge
    if rise
        us_cnt = uint16(0);
        st = uint8(3);
    elseif tick_us
        if us_cnt >= uint16(P_RISE_TMO_US)
            out_us = uint16(0);
            out_to = true;
            out_v  = true;
            st = uint8(0);
        else
            us_cnt = us_cnt + uint16(1);
        end
    end
elseif st == uint8(3)                   % MEASURE echo high time
    if fall
        out_us = us_cnt;
        out_to = false;
        out_v  = true;
        st = uint8(0);
    elseif tick_us
        if us_cnt >= uint16(P_ECHO_MAX_US)
            out_us = us_cnt;
            out_to = true;
            out_v  = true;
            st = uint8(4);
        else
            us_cnt = us_cnt + uint16(1);
        end
    end
else                                    % 4: echo stuck high, wait for low
    if ~echo_s
        st = uint8(0);
    end
end
end
