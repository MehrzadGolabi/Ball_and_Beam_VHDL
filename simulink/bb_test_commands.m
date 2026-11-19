function b = bb_test_commands()
%BB_TEST_COMMANDS  Host byte stream used by the Simulink harness:
%   1) trigger period 10 ms (speeds up the test)
%   2) read version register (expect 0xBB01)
%   3) frame with a corrupted checksum (must be ignored, counted as bad)
%   4) read bad-frame counter (expect 1)
b = [frame(11, 10), frame(128+13, 0), frame(0, 999, true), frame(128+15, 0)];
end

function f = frame(addr, val, corrupt)
if nargin < 3, corrupt = false; end
hi = floor(val/256); lo = mod(val, 256);
chk = bitxor(bitxor(165, addr), bitxor(hi, lo));
if corrupt, chk = bitxor(chk, 1); end
f = [165 addr hi lo chk];
end
