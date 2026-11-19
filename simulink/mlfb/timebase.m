function [tick_us, tick_ms] = timebase(P_CLK_PER_US)
%TIMEBASE  1 us and 1 ms clock-enable strobes (one clock wide).
%#codegen
persistent c_clk c_us t_us t_ms
if isempty(c_clk)
    c_clk = uint8(0); c_us = uint16(0); t_us = false; t_ms = false;
end
tick_us = t_us;
tick_ms = t_ms;
if c_clk == uint8(P_CLK_PER_US - 1)
    c_clk = uint8(0);
    t_us  = true;
    if c_us == uint16(999)
        c_us = uint16(0);
        t_ms = true;
    else
        c_us = c_us + uint16(1);
        t_ms = false;
    end
else
    c_clk = c_clk + uint8(1);
    t_us  = false;
    t_ms  = false;
end
end
