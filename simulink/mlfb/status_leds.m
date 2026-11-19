function [led0, led1, led2, led3] = status_leds(sample, sflags, err_mm, mode, tick_ms)
%STATUS_LEDS  led0: 30 ms flash per sensor sample   led1: |error| <= 10 mm
%             led2: last reading bad (timeout / out of range)
%             led3: 1 Hz heartbeat (fast 4 Hz blink when loop disabled)
%#codegen
persistent fl_cnt hb_cnt hb l0 l1 l2
if isempty(fl_cnt)
    fl_cnt = uint8(0); hb_cnt = uint16(0); hb = false; l0 = false; l1 = false; l2 = false;
end
led0 = l0; led1 = l1; led2 = l2; led3 = hb;

if sample
    fl_cnt = uint8(30);
    l2 = sflags ~= uint8(0);
elseif tick_ms && fl_cnt ~= uint8(0)
    fl_cnt = fl_cnt - uint8(1);
end
l0 = fl_cnt ~= uint8(0);
l1 = (err_mm <= int16(10)) && (err_mm >= int16(-10)) && ~l2;
if bitand(mode, uint16(1)) ~= uint16(0)
    half = uint16(499);
else
    half = uint16(124);
end
if tick_ms
    if hb_cnt >= half
        hb_cnt = uint16(0);
        hb = ~hb;
    else
        hb_cnt = hb_cnt + uint16(1);
    end
end
end
