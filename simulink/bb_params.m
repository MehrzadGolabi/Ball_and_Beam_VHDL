function P = bb_params()
%BB_PARAMS  Single source of truth for the ball-and-beam FPGA design.
%   All hardware constants (clock, UART, sensor limits) and the power-up
%   register defaults live here. build_bb_model.m pushes them into the
%   Simulink model workspace; HDL Coder folds them into constants.

% ---- Clock / timing -------------------------------------------------------
P.CLK_HZ        = 24e6;               % Posedge-One on-board oscillator (p50)
P.CLK_PER_US    = P.CLK_HZ/1e6;       % 24 clocks per microsecond
P.BAUD          = 115200;
P.UART_DIV      = round(P.CLK_HZ/P.BAUD);   % 208 -> 115385 baud (+0.16 %)

% ---- HC-SR04 ----------------------------------------------------------------
P.TRIG_US       = 12;                 % trigger pulse width (>=10 us)
P.RISE_TMO_US   = 10000;              % echo must start within 10 ms
P.ECHO_MAX_US   = 30000;              % >30 ms high = no object (sensor gives 38 ms)
P.MM_PER_US_Q16 = 11239;              % mm = us*343/2000 = us*0.17150 -> *11239>>16
P.MIN_MM        = 20;                 % below this the sensor is unreliable
P.MAX_MM        = 1000;               % anything farther is not on the beam

% ---- Servo (MG996R) ------------------------------------------------------
P.PWM_PERIOD_US = 20000;              % 50 Hz
P.ABS_MIN_US    = 500;                % hard safety clamp, independent of registers
P.ABS_MAX_US    = 2600;

% ---- Power-up register defaults (all runtime-writable over UART) ------------
% Gains are Q8.8 unsigned: value/256 = real gain.
%   Kp : us of servo pulse per mm of position error
%   Ki : us per (mm * sample)
%   Kd : us per (mm/sample) of ball velocity (derivative on measurement)
P.DEF.SETPOINT  = 330;                % mm from sensor
P.DEF.KP        = round(3*256);
P.DEF.KI        = 2;                  % 0.0078 (plant is integrating; I only trims beam offset)
P.DEF.KD        = round(50*256);
P.DEF.CENTER    = 1750;               % servo pulse that levels the beam
P.DEF.SMIN      = 1000;
P.DEF.SMAX      = 2500;
P.DEF.MODE      = 1 + 4 + 8;          % loop enable | telemetry | median filter
P.DEF.MANUAL    = 1750;
P.DEF.DSHIFT    = 2;                  % derivative IIR: alpha = 1/2^DSHIFT
P.DEF.ILIM      = 200;                % integrator clamp, us
P.DEF.TRIG_MS   = 40;                 % sensor/control period (25 Hz)
P.VERSION       = hex2dec('BB01');
end
