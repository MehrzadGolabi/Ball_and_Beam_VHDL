function [tel, rd, bytes] = bb_decode_uart(txd, div)
%BB_DECODE_UART  Decode a per-clock UART line into bytes and frames.
%   tel : struct array of telemetry frames (checksum verified)
%   rd  : Nx2 [addr value] register read replies
txd = logical(txd(:));
bytes = zeros(1, 0);
i = 2; n = numel(txd);
while i < n - 10*div
    if txd(i-1) && ~txd(i)                       % falling edge = start bit
        c = i + floor(div/2);
        if ~txd(c)
            v = 0;
            for b = 0:7
                v = v + txd(c + (b+1)*div) * 2^b;
            end
            if txd(c + 9*div)
                bytes(end+1) = v; %#ok<AGROW>
            end
            i = c + 9*div;
            continue
        end
    end
    i = i + 1;
end
tel = struct('seq', {}, 'pos', {}, 'echo', {}, 'sp', {}, 'servo', {}, ...
             'err', {}, 'i', {}, 'd', {}, 'flags', {});
rd = zeros(0, 2);
k = 1;
s16 = @(h, l) (h*256 + l) - 65536*((h*256 + l) >= 32768);
while k <= numel(bytes)
    if bytes(k) == 90 && k+1 <= numel(bytes)
        if bytes(k+1) == 16 && k+18 <= numel(bytes)
            f = bytes(k:k+18);
            if xorall(f(1:18)) == f(19)
                tel(end+1) = struct('seq', f(3), 'pos', f(4)*256+f(5), 'echo', f(6)*256+f(7), ...
                    'sp', f(8)*256+f(9), 'servo', f(10)*256+f(11), 'err', s16(f(12), f(13)), ...
                    'i', s16(f(14), f(15)), 'd', s16(f(16), f(17)), 'flags', f(18)); %#ok<AGROW>
                k = k + 19; continue
            end
        elseif bytes(k+1) == 32 && k+5 <= numel(bytes)
            f = bytes(k:k+5);
            if xorall(f(1:5)) == f(6)
                rd(end+1, :) = [f(3), f(4)*256+f(5)]; %#ok<AGROW>
                k = k + 6; continue
            end
        end
    end
    k = k + 1;
end
end

function c = xorall(v)
c = 0;
for x = v, c = bitxor(c, x); end
end
