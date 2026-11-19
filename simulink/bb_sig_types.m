function T = bb_sig_types()
%BB_SIG_TYPES  Data type of every named signal in the controller.
T = containers.Map();
b = {'echo_in','uart_rxd','echo_s','rxd_s','tick_us','tick_ms','meas_valid','meas_timeout', ...
     'sample','ctl_done','rx_valid','int_rst','rd_req','tx_start','tx_busy','trig_out', ...
     'servo_out','uart_txd','led0','led1','led2','led3'};
u16 = {'echo_us','pos_mm','setpoint','kp','ki','kd','center','smin','smax','mode', ...
       'manual_us','dshift','ilim','trig_ms','servo_us','rd_data'};
s16 = {'err_mm','iterm','dterm'};
u8  = {'sflags','rx_byte','rd_addr','tx_data'};
for k = 1:numel(b),   T(b{k})   = 'boolean'; end
for k = 1:numel(u16), T(u16{k}) = 'uint16';  end
for k = 1:numel(s16), T(s16{k}) = 'int16';   end
for k = 1:numel(u8),  T(u8{k})  = 'uint8';   end
T('ball_mm') = 'double'; T('pulse_us') = 'double';
end
