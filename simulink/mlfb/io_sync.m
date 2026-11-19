function [echo_s, rxd_s] = io_sync(echo_in, uart_rxd)
%IO_SYNC  Two-flop synchronisers for the asynchronous inputs.
%#codegen
persistent e1 e2 r1 r2
if isempty(e1)
    e1 = false; e2 = false; r1 = true; r2 = true;
end
echo_s = e2;
rxd_s  = r2;
e2 = e1; e1 = echo_in;
r2 = r1; r1 = uart_rxd;
end
