function [setpoint, kp, ki, kd, center, smin, smax, mode, manual_us, dshift, ilim, trig_ms, ...
    int_rst, rd_req, rd_addr, rd_data] = cmd_regs(rx_byte, rx_valid, ...
    P_DEF_SETPOINT, P_DEF_KP, P_DEF_KI, P_DEF_KD, P_DEF_CENTER, P_DEF_SMIN, P_DEF_SMAX, ...
    P_DEF_MODE, P_DEF_MANUAL, P_DEF_DSHIFT, P_DEF_ILIM, P_DEF_TRIG_MS, P_VERSION)
%CMD_REGS  UART command parser + register file.
%   Host frame (5 bytes):  A5  ADDR  HI  LO  CHK,   CHK = A5^ADDR^HI^LO
%     ADDR bit7 = 0 : write HI:LO to register ADDR
%     ADDR bit7 = 1 : read register ADDR&7F (HI/LO ignored) -> reply frame
%   Registers: 00 setpoint mm | 01 Kp | 02 Ki | 03 Kd (Q8.8)
%     04 servo centre us | 05 servo min | 06 servo max | 07 mode
%     08 manual servo us | 09 D filter shift | 0A integrator limit us
%     0B trigger period ms | 0C write: bit0 = reset integrator
%     0D version (RO) | 0E good frames (RO) | 0F bad-checksum frames (RO)
%   A partial frame is discarded after 65535 clocks (~2.7 ms) of silence.
%#codegen
persistent st a hi lo tmo r_sp r_kp r_ki r_kd r_c r_mn r_mx r_md r_man r_ds r_il r_tr ...
    n_ok n_bad irst rq ra rdat
if isempty(st)
    st = uint8(0); a = uint8(0); hi = uint8(0); lo = uint8(0); tmo = uint16(0);
    r_sp = uint16(P_DEF_SETPOINT); r_kp = uint16(P_DEF_KP); r_ki = uint16(P_DEF_KI);
    r_kd = uint16(P_DEF_KD); r_c = uint16(P_DEF_CENTER); r_mn = uint16(P_DEF_SMIN);
    r_mx = uint16(P_DEF_SMAX); r_md = uint16(P_DEF_MODE); r_man = uint16(P_DEF_MANUAL);
    r_ds = uint16(P_DEF_DSHIFT); r_il = uint16(P_DEF_ILIM); r_tr = uint16(P_DEF_TRIG_MS);
    n_ok = uint16(0); n_bad = uint16(0); irst = false; rq = false; ra = uint8(0); rdat = uint16(0);
end
setpoint = r_sp; kp = r_kp; ki = r_ki; kd = r_kd; center = r_c; smin = r_mn; smax = r_mx;
mode = r_md; manual_us = r_man; dshift = r_ds; ilim = r_il; trig_ms = r_tr;
int_rst = irst; rd_req = rq; rd_addr = ra; rd_data = rdat;

irst = false;
rq = false;

if rx_valid
    tmo = uint16(0);
    if st == uint8(0)
        if rx_byte == uint8(165)        % 0xA5
            st = uint8(1);
        end
    elseif st == uint8(1)
        a = rx_byte; st = uint8(2);
    elseif st == uint8(2)
        hi = rx_byte; st = uint8(3);
    elseif st == uint8(3)
        lo = rx_byte; st = uint8(4);
    else
        st = uint8(0);
        chk = bitxor(bitxor(uint8(165), a), bitxor(hi, lo));
        if chk == rx_byte
            if n_ok < uint16(65535), n_ok = n_ok + uint16(1); end
            v = bitor(bitshift(uint16(hi), 8), uint16(lo));
            if bitand(a, uint8(128)) ~= uint8(0)
                ra = bitand(a, uint8(127));
                rq = true;
            else
                if a == uint8(0),  r_sp = v; end
                if a == uint8(1),  r_kp = v; end
                if a == uint8(2),  r_ki = v; end
                if a == uint8(3),  r_kd = v; end
                if a == uint8(4),  r_c  = v; end
                if a == uint8(5),  r_mn = v; end
                if a == uint8(6),  r_mx = v; end
                if a == uint8(7),  r_md = v; end
                if a == uint8(8),  r_man = v; end
                if a == uint8(9),  r_ds = bitand(v, uint16(7)); end
                if a == uint8(10), r_il = v; end
                if a == uint8(11), r_tr = v; end
                if a == uint8(12), irst = bitand(v, uint16(1)) ~= uint16(0); end
            end
        else
            if n_bad < uint16(65535), n_bad = n_bad + uint16(1); end
        end
    end
else
    if st ~= uint8(0)
        if tmo == uint16(65535)
            st = uint8(0);
            tmo = uint16(0);
        else
            tmo = tmo + uint16(1);
        end
    end
end

% read mux (registered; valid together with rd_req)
ad = bitand(a, uint8(127));
if ad == uint8(0),      rdat = r_sp;
elseif ad == uint8(1),  rdat = r_kp;
elseif ad == uint8(2),  rdat = r_ki;
elseif ad == uint8(3),  rdat = r_kd;
elseif ad == uint8(4),  rdat = r_c;
elseif ad == uint8(5),  rdat = r_mn;
elseif ad == uint8(6),  rdat = r_mx;
elseif ad == uint8(7),  rdat = r_md;
elseif ad == uint8(8),  rdat = r_man;
elseif ad == uint8(9),  rdat = r_ds;
elseif ad == uint8(10), rdat = r_il;
elseif ad == uint8(11), rdat = r_tr;
elseif ad == uint8(13), rdat = uint16(P_VERSION);
elseif ad == uint8(14), rdat = n_ok;
elseif ad == uint8(15), rdat = n_bad;
else,                   rdat = uint16(0);
end
end
