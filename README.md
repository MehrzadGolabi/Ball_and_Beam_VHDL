## Ball and Beam on FPGA

This design implements a Ball-and-Beam control loop on an Spartan-6 FPGA using:

- HC-SR04 ultrasonic sensor interface (trigger + echo-time + distance in cm)
- Discrete PID controller with a fixed, hard-coded setpoint
- Servo PWM for MG996 @ 50 Hz with 1.0 ms to 2.5 ms pulse width
- UART telemetry (115200) to monitor distance and servo position
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
- Hard-coded setpoint `SetVal` was changed to 33 (cm) in this design. 
- Mapping from sensor distance to PID input in `top_ball_beam` scales the centimeter reading by 1000 to better utilize the 16-bit resolution.
- PID gains (`Kp`, `Ki`, `Kd`)

### Servo PWM (MG996)

- Requirements: 50 Hz (20 ms period), pulse width 1.0 ms to 2.5 ms.
- The top-level produces PWM directly from the `clk` using integer math:
  - `ticks_per_period = Fclk / 50`
  - `ticks_1ms = Fclk / 1000`
  - `ticks_2_5ms = (Fclk * 25) / 10000`
  - Pulse width linearly interpolates across 0..127 servo positions.

Reference on servo PWM timing: `Servomotor Control with PWM and VHDL` on [CodeProject](https://www.codeproject.com/Articles/513169/Servomotor-Control-with-PWM-and-VHDL).

### UART Telemetry

- UART module integrated at 115200 buadrate by setting divisor ≈ `Fclk/baud`.
  - For 24 MHz, divisor = 24_000_000 / 115200 ≈ 208.33 → use 208.
- Telemetry format per measurement: `D=hhh S=ppp\r\n`
  - `D` is distance in cm (3 digits)
  - `S` is servo position 0..127 (3 digits)

### LEDs

- `leds(0)`: strobe on new distance measurement
- `leds(1)`: near setpoint (|distance-33| ≤ 1 cm)
- `leds(3:2)`: slow heartbeat / activity


## Opensource Projects Used:

 - [pid-fpga-vhdl](https://github.com/deepc94/pid-fpga-vhdl/tree/master)
 - [HC_SR04_interface_VHDL](https://github.com/S81f/HC_SR04_interface_VHDL)

