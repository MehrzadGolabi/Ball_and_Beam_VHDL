## Ball and Beam on FPGA

This design implements a Ball-and-Beam control loop on an Spartan-6 FPGA using:

- HC-SR04 ultrasonic sensor interface (trigger + echo-time + distance in cm)
- Discrete PID controller with a fixed, hard-coded setpoint (can be updated at runtime via UART)
- Servo PWM for MG996 @ 50 Hz with 1.0 ms to 2.5 ms pulse width
- UART telemetry (115200) to monitor distance and servo position, and to tune PID at runtime
- LEDs for simple status

### Top Module

`top_ball_beam.vhd` integrates all IPs and exposes: `clk`, `reset`, `hcsr04_echo`, `hcsr04_trig`, `servo`, `uart_rxd`, `uart_txd`, and `leds[3:0]`.

### Clock

- System clock: 24 MHz, If different, change `CLK_FREQ_HZ` generic on `top_ball_beam`.

### HC-SR04 Timing and Distance Calculation

- Sensor requires trigger pulse of 10 µs minimum every ≥60 ms. `trigger_generator.vhd` produces ~10 µs high once every 60 ms at the system clock.
- Echo time is captured by `counter.vhd` as number of system clock cycles while `echo=1`.
- Distance calculation in `measurement_cal.vhd` uses:
  - `time_ns = echo_cycles * ns_per_cycle`
  - `distance_cm ≈ time_us / 58` (round trip divided by speed of sound; design uses a constant-multiplier division scheme)
- In `top_ball_beam`, `measurement_cal` generic `ns_cycel` is set to 42 (ns) for 24 MHz clock (41.666.. ns ≈ 42 ns). For other clock frequencies, set `ns_cycel = round(1e9 / Fclk)`.

References for HC-SR04 math: `distance_cm = pulse_width_us / 58`.

### PID Controller

- File: `pid.vhdl`. Uses 16-bit unsigned input and output.
- Default setpoint is 33 (cm). PID gains default: Kp=10, Ki=1, Kd=20. All can be changed live via UART.

### Servo PWM (MG996)

- Requirements: 50 Hz (20 ms period), pulse width 1.0 ms to 2.5 ms.
- The top-level produces PWM directly from the `clk` using integer math:
  - `ticks_per_period = Fclk / 50`
  - `ticks_1ms = Fclk / 1000`
  - `ticks_2_5ms = (Fclk * 25) / 10000`
  - Pulse width linearly interpolates across 0..127 servo positions.

Reference on servo PWM timing: `Servomotor Control with PWM and VHDL` on CodeProject (`https://www.codeproject.com/Articles/513169/Servomotor-Control-with-PWM-and-VHDL`).

### UART

- Baud: 115200 bps (divisor ≈ `Fclk/baud`, for 24 MHz use 208)
  - For 24 MHz, divisor = 24_000_000 / 115200 ≈ 208.33 → use 208.
- Telemetry per measurement: `D=hhh S=ppp\r\n`
  - `D` is distance in cm (3 digits)
  - `S` is servo position 0..127 (3 digits)

#### Runtime Tuning Protocol (HEX framing)

All command bytes are raw 8-bit values. Two command types are supported:

- Set PID gains (Kp, Ki, Kd)
  - Frame (8 bytes total):
    - `0`: 0x50 ('P')
    - `1`: Kp_hi
    - `2`: Kp_lo
    - `3`: Ki_hi
    - `4`: Ki_lo
    - `5`: Kd_hi
    - `6`: Kd_lo
    - `7`: 0x0A (LF)
  - Example (Kp=10, Ki=1, Kd=20): `50 00 0A 00 01 00 14 0A`

- Set Setpoint (centimeters, unsigned 16-bit)
  - Frame (8 bytes total):
    - `0`: 0x53 ('S')
    - `1`: SP_hi
    - `2`: SP_lo
    - `3`: 0x00 (reserved)
    - `4`: 0x00 (reserved)
    - `5`: 0x00 (reserved)
    - `6`: 0x00 (reserved)
    - `7`: 0x0A (LF)
  - Example (Setpoint=33): `53 00 21 00 00 00 00 0A`

Notes:
- Values are big-endian within each 16-bit field.
- Controller latches new parameters on receipt of the LF byte.
- RX protocol is tolerant to gaps between bytes (buffered one-byte-at-a-time).

### LEDs

- `leds(0)`: strobe on new distance measurement
- `leds(1)`: near setpoint (|distance-33| ≤ 1 cm)
- `leds(3:2)`: slow heartbeat / activity


## Opensource Projects Used:

 - [pid-fpga-vhdl](https://github.com/deepc94/pid-fpga-vhdl/tree/master)
 - [HC_SR04_interface_VHDL](https://github.com/S81f/HC_SR04_interface_VHDL)

