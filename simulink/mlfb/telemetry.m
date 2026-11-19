function [tx_data, tx_start] = telemetry(ctl_done, pos_mm, echo_us, setpoint, servo_us, err_mm, iterm, dterm, ...
    sflags, mode, rd_req, rd_addr, rd_data, tx_busy)
%TELEMETRY  Serialises frames to the UART transmitter.
%   Telemetry (19 bytes, after every control update when mode bit2 = 1):
%     5A 10 SEQ POS(2) ECHO_US(2) SETPOINT(2) SERVO_US(2) ERR(2) I(2) D(2) FLAGS CHK
%   Register read reply (6 bytes):  5A 20 ADDR HI LO CHK
%   Multi-byte fields are big-endian; ERR/I/D are int16. CHK = XOR of all
%   previous bytes of the frame. FLAGS: bit0 echo timeout, bit1 out of
%   range, bit2 loop enabled, bit3 inverted, bit4 median filter on.
%#codegen
persistent st idx len wt seq b data_r start_r prd pa pd
if isempty(st)
    st = uint8(0); idx = uint8(0); len = uint8(0); wt = uint8(0); seq = uint8(0);
    b = zeros(1, 19, 'uint8'); data_r = uint8(0); start_r = false;
    prd = false; pa = uint8(0); pd = uint16(0);
end
tx_data  = data_r;
tx_start = start_r;
start_r  = false;

if rd_req
    prd = true; pa = rd_addr; pd = rd_data;
end

if st == uint8(0)
    if ctl_done && bitand(mode, uint16(4)) ~= uint16(0)
        fl = bitand(sflags, uint8(3));
        if bitand(mode, uint16(1)) ~= uint16(0), fl = bitor(fl, uint8(4)); end
        if bitand(mode, uint16(2)) ~= uint16(0), fl = bitor(fl, uint8(8)); end
        if bitand(mode, uint16(8)) ~= uint16(0), fl = bitor(fl, uint8(16)); end
        e16 = s2u(err_mm); i16 = s2u(iterm); d16 = s2u(dterm);
        b(1) = uint8(90);  b(2) = uint8(16); b(3) = seq;
        b(4) = hb(pos_mm);   b(5) = lb(pos_mm);
        b(6) = hb(echo_us);  b(7) = lb(echo_us);
        b(8) = hb(setpoint); b(9) = lb(setpoint);
        b(10) = hb(servo_us); b(11) = lb(servo_us);
        b(12) = hb(e16); b(13) = lb(e16);
        b(14) = hb(i16); b(15) = lb(i16);
        b(16) = hb(d16); b(17) = lb(d16);
        b(18) = fl;
        c = uint8(0);
        for k = 1:18
            c = bitxor(c, b(k));
        end
        b(19) = c;
        seq = seq + uint8(1);
        len = uint8(19); idx = uint8(0); st = uint8(1);
    elseif prd
        prd = false;
        b(1) = uint8(90); b(2) = uint8(32); b(3) = pa; b(4) = hb(pd); b(5) = lb(pd);
        b(6) = bitxor(bitxor(bitxor(b(1), b(2)), bitxor(b(3), b(4))), b(5));
        len = uint8(6); idx = uint8(0); st = uint8(1);
    end
elseif st == uint8(1)                   % hand next byte to the UART
    if ~tx_busy
        data_r  = b(idx + 1);
        start_r = true;
        wt = uint8(3);
        st = uint8(2);
    end
else                                    % 2: let tx_busy assert
    if wt == uint8(0)
        idx = idx + uint8(1);
        if idx == len
            st = uint8(0);
        else
            st = uint8(1);
        end
    else
        wt = wt - uint8(1);
    end
end
end

function y = hb(x)
y = uint8(bitshift(x, -8));
end

function y = lb(x)
y = uint8(bitand(x, uint16(255)));
end

function y = s2u(x)
% int16 -> uint16 two's-complement bit pattern
if x < int16(0)
    y = uint16(int32(x) + int32(65536));
else
    y = uint16(x);
end
end
