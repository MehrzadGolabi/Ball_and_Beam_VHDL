function [servo, c] = bb_ctrl_ref(c, pos_mm, R)
%BB_CTRL_REF  Bit-accurate sample-level reference of mlfb/pid_ctrl.m.
%   Used only for gain tuning (tune_gains.m); the HDL is generated from the
%   MATLAB Function blocks, and GHDL checks the HDL in closed loop.
%   c : controller state struct (empty -> initialise)
%   R : register struct with fields like bb_params().DEF
if isempty(c)
    c.have = false; c.prev = 0; c.df = 0; c.iacc = 0;
end
e = R.SETPOINT - pos_mm;
e = max(-2000, min(2000, e));
if c.have, dm = c.prev - pos_mm; else, dm = 0; end
dm = max(-127, min(127, dm));
c.have = true; c.prev = pos_mm;
c.df = c.df + floor((dm*256 - c.df) / 2^R.DSHIFT);
en = bitand(R.MODE,1) ~= 0;
p  = R.KP * e;
in = c.iacc + R.KI * e;
lim = R.ILIM * 256;
in = max(-lim, min(lim, in));
if en, c.iacc = in; else, c.iacc = 0; end
d  = floor(R.KD * c.df / 256);
u  = floor((p + c.iacc + d) / 256);
u  = max(-4000, min(4000, u));
if bitand(R.MODE,2) ~= 0, u = -u; end
if en, out = R.CENTER + u; else, out = R.MANUAL; end
servo = max(R.SMIN, min(R.SMAX, out));
end
