function [pos_mm, sample, sflags] = pos_filter(echo_us, meas_valid, meas_timeout, mode, ...
    P_MM_PER_US_Q16, P_MIN_MM, P_MAX_MM)
%POS_FILTER  Echo time -> millimetres, range check, optional median-of-3.
%   Emits one `sample` strobe per measurement attempt so the controller
%   runs at a fixed rate. Bad readings hold the last good position.
%   sflags: bit0 = echo timeout, bit1 = out of range.
%#codegen
persistent h0 h1 h2 last have out_pos out_s out_f
if isempty(h0)
    h0 = uint16(0); h1 = uint16(0); h2 = uint16(0); last = uint16(0);
    have = false; out_pos = uint16(0); out_s = false; out_f = uint8(0);
end
pos_mm = out_pos;
sample = out_s;
sflags = out_f;

out_s = false;
if meas_valid
    prod = uint32(echo_us) * uint32(P_MM_PER_US_Q16);
    mm   = uint16(bitshift(prod, -16));
    inrange = (mm >= uint16(P_MIN_MM)) && (mm <= uint16(P_MAX_MM));
    if meas_timeout
        out_f = uint8(1);
    elseif ~inrange
        out_f = uint8(2);
    else
        out_f = uint8(0);
        if have
            h2 = h1; h1 = h0; h0 = mm;
        else
            h2 = mm; h1 = mm; h0 = mm; have = true;
        end
        if bitand(mode, uint16(8)) ~= uint16(0)
            mn = min(h0, h1);
            mx = max(h0, h1);
            last = max(mn, min(mx, h2));
        else
            last = mm;
        end
    end
    out_pos = last;
    out_s = true;
end
end
