function [rx_byte, rx_valid] = uart_rx(rxd_s, P_UART_DIV)
%UART_RX  8N1 receiver, samples each bit at its centre. rx_valid pulses
%   for one clock per good byte; bytes with a bad stop bit are dropped.
%#codegen
persistent st cnt bitn sh ob ov
if isempty(st)
    st = uint8(0); cnt = uint16(0); bitn = uint8(0); sh = uint8(0);
    ob = uint8(0); ov = false;
end
rx_byte  = ob;
rx_valid = ov;
ov = false;

if st == uint8(0)                       % idle, wait for start bit
    if ~rxd_s
        cnt = uint16(floor(P_UART_DIV/2));
        st = uint8(1);
    end
elseif st == uint8(1)                   % middle of start bit
    if cnt == uint16(0)
        if ~rxd_s
            cnt = uint16(P_UART_DIV - 1);
            bitn = uint8(0);
            st = uint8(2);
        else
            st = uint8(0);              % glitch
        end
    else
        cnt = cnt - uint16(1);
    end
elseif st == uint8(2)                   % data bits, LSB first
    if cnt == uint16(0)
        sh = bitshift(sh, -1);
        if rxd_s
            sh = bitor(sh, uint8(128));
        end
        cnt = uint16(P_UART_DIV - 1);
        if bitn == uint8(7)
            st = uint8(3);
        else
            bitn = bitn + uint8(1);
        end
    else
        cnt = cnt - uint16(1);
    end
elseif st == uint8(3)                   % stop bit
    if cnt == uint16(0)
        if rxd_s
            ob = sh;
            ov = true;
            st = uint8(0);
        else
            st = uint8(4);              % framing error
        end
    else
        cnt = cnt - uint16(1);
    end
else                                    % 4: wait for line idle
    if rxd_s
        st = uint8(0);
    end
end
end
