function [servo_us, err_mm, iterm, dterm, ctl_done] = pid_ctrl(pos_mm, sample, setpoint, kp, ki, kd, ...
    center, smin, smax, mode, manual_us, dshift, ilim, int_rst)
%PID_CTRL  Discrete PID, executed once per sensor sample (4-cycle sequence,
%   one shared multiplier).
%     e  = setpoint - pos                     [mm]
%     P  = Kp*e
%     I += Ki*e,  clamped to +/- ilim us      (anti-windup)
%     D  = Kd*filt(pos[k-1]-pos[k])           (derivative on measurement,
%                                              IIR alpha = 1/2^dshift)
%     servo = clamp(center +/- (P+I+D), smin, smax)
%   Gains are Q8.8; internal sums are in us*256.
%   mode bit0 = loop enable (else servo = manual_us), bit1 = invert output.
%   Bit-exact sample-level reference: bb_ctrl_ref.m.
%#codegen
persistent stg have pos_prev df e_r iacc pacc dacc servo_r err_r it_r dt_r done_r
if isempty(stg)
    stg = uint8(0); have = false; pos_prev = uint16(0); df = int32(0); e_r = int32(0);
    iacc = int32(0); pacc = int32(0); dacc = int32(0);
    servo_r = uint16(1750); err_r = int16(0); it_r = int16(0); dt_r = int16(0); done_r = false;
end
servo_us = servo_r;
err_mm   = err_r;
iterm    = it_r;
dterm    = dt_r;
ctl_done = done_r;

done_r = false;
enable = bitand(mode, uint16(1)) ~= uint16(0);
invert = bitand(mode, uint16(2)) ~= uint16(0);

% ---- shared multiplier: operand select by stage ----
if stg == uint8(1)
    ma = int32(kp);  mb = e_r;
elseif stg == uint8(2)
    ma = int32(ki);  mb = e_r;
else
    ma = int32(kd);  mb = df;
end
prod = ma * mb;

if stg == uint8(0)
    if sample
        e = int32(setpoint) - int32(pos_mm);
        if e > int32(2000), e = int32(2000); end
        if e < int32(-2000), e = int32(-2000); end
        if have
            dm = int32(pos_prev) - int32(pos_mm);
        else
            dm = int32(0);
        end
        if dm > int32(127), dm = int32(127); end
        if dm < int32(-127), dm = int32(-127); end
        have = true;
        pos_prev = pos_mm;
        x = dm*int32(256) - df;
        if dshift == uint16(0)
            df = df + x;
        elseif dshift == uint16(1)
            df = df + bitsra(x, 1);
        elseif dshift == uint16(2)
            df = df + bitsra(x, 2);
        elseif dshift == uint16(3)
            df = df + bitsra(x, 3);
        elseif dshift == uint16(4)
            df = df + bitsra(x, 4);
        elseif dshift == uint16(5)
            df = df + bitsra(x, 5);
        elseif dshift == uint16(6)
            df = df + bitsra(x, 6);
        else
            df = df + bitsra(x, 7);
        end
        e_r = e;
        stg = uint8(1);
    end
elseif stg == uint8(1)                  % P = Kp*e
    pacc = prod;
    stg = uint8(2);
elseif stg == uint8(2)                  % I += Ki*e
    lim = int32(ilim) * int32(256);
    inew = iacc + prod;
    if inew > lim, inew = lim; end
    if inew < -lim, inew = -lim; end
    if enable
        iacc = inew;
    else
        iacc = int32(0);
    end
    stg = uint8(3);
else                                    % 3: D = Kd*df, combine, clamp
    dacc = bitsra(prod, 8);
    u = bitsra(pacc + iacc + dacc, 8);
    if u > int32(4000), u = int32(4000); end
    if u < int32(-4000), u = int32(-4000); end
    if invert, u = -u; end
    if enable
        out = int32(center) + u;
    else
        out = int32(manual_us);
    end
    if out > int32(smax), out = int32(smax); end
    if out < int32(smin), out = int32(smin); end
    servo_r = uint16(out);
    err_r = int16(e_r);
    it_r  = int16(bitsra(iacc, 8));
    dv = bitsra(dacc, 8);
    if dv > int32(32767), dv = int32(32767); end
    if dv < int32(-32768), dv = int32(-32768); end
    dt_r = int16(dv);
    done_r = true;
    stg = uint8(0);
end
if int_rst
    iacc = int32(0);
end
end
