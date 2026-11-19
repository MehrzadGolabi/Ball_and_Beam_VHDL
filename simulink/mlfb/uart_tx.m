function [uart_txd, tx_busy] = uart_tx(tx_data, tx_start, P_UART_DIV)
%UART_TX  8N1 transmitter. tx_start is accepted only while idle.
%#codegen
persistent st cnt bitn sh line busy
if isempty(st)
    st = uint8(0); cnt = uint16(0); bitn = uint8(0); sh = uint8(0);
    line = true; busy = false;
end
uart_txd = line;
tx_busy  = busy;

if st == uint8(0)
    line = true;
    if tx_start
        sh   = tx_data;
        line = false;                   % start bit
        cnt  = uint16(P_UART_DIV - 1);
        st   = uint8(1);
        busy = true;
    else
        busy = false;
    end
elseif st == uint8(1)                   % end of start bit -> bit 0
    if cnt == uint16(0)
        line = bitand(sh, uint8(1)) ~= uint8(0);
        sh   = bitshift(sh, -1);
        bitn = uint8(0);
        cnt  = uint16(P_UART_DIV - 1);
        st   = uint8(2);
    else
        cnt = cnt - uint16(1);
    end
elseif st == uint8(2)                   % data bits 1..7, then stop
    if cnt == uint16(0)
        if bitn == uint8(6)
            line = bitand(sh, uint8(1)) ~= uint8(0);
            bitn = uint8(7);
        elseif bitn == uint8(7)
            line = true;                % stop bit
            st   = uint8(3);
        else
            line = bitand(sh, uint8(1)) ~= uint8(0);
            bitn = bitn + uint8(1);
        end
        sh  = bitshift(sh, -1);
        cnt = uint16(P_UART_DIV - 1);
    else
        cnt = cnt - uint16(1);
    end
else                                    % 3: stop bit
    if cnt == uint16(0)
        st = uint8(0);
    else
        cnt = cnt - uint16(1);
    end
end
end
