library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Top-level for Ball and Beam on FPGA
-- Integrates: HC-SR04 ultrasonic, PID, Servo PWM (50 Hz), UART, and LEDs
-- Notes:
-- - Uses provided counter and trigger for HC-SR04; measurement_cal with adjusted clock ns generic
-- - PID setpoint is hardcoded in pid.vhdl (SetVal)
-- - Generates servo PWM directly from system clock at 50 Hz and 1.0..2.5 ms pulse width

entity top_ball_beam is
  generic (
    CLK_FREQ_HZ : integer := 24000000; -- system clock frequency (Hz)
    UART_DIVISOR: integer := 208       -- clk cycles per UART bit (24e6/115200 ≈ 208.33)
  );
  port (
    clk       : in  std_logic;            -- system clock
    reset     : in  std_logic;            -- active-high reset

    -- HC-SR04 signals
    hcsr04_echo : in  std_logic;
    hcsr04_trig : out std_logic;

    -- Servo output
    servo      : out std_logic;

    -- UART
    uart_rxd   : in  std_logic;
    uart_txd   : out std_logic;

    -- LEDs (optional: simple status)
    leds       : out std_logic_vector(3 downto 0)
  );
end top_ball_beam;

architecture rtl of top_ball_beam is

  -- Components from HC_SR04_interface_VHDL
  component counter
    port (
      i_Clock           : in  std_logic;
      i_Reset_n         : in  std_logic;
      i_Echo            : in  std_logic;
      o_DV_n            : out std_logic;
      o_Echo_pulse_time : out std_logic_vector(23 downto 0)
    );
  end component;

  component trigger_generator
    port (
      i_Clock   : in  std_logic;
      i_Reset_n : in  std_logic;
      o_Trigger : out std_logic
    );
  end component;

  component measurement_cal
    generic (
      ns_cycel      : unsigned(4 downto 0) := to_unsigned(20,5);
      division_cons : unsigned(29 downto 0) := to_unsigned(606627105, 30)
    );
    port (
      i_Clock            : in  std_logic;
      i_Reset_n          : in  std_logic;
      i_Echo_pulse_time  : in  std_logic_vector(23 downto 0);
      i_DV_n             : in  std_logic;
      o_Distance         : out std_logic_vector(13 downto 0);
      o_DV_n             : out std_logic
    );
  end component;

  component binary_to_bcd
    port (
      i_Clock    : in  std_logic;
      i_Reset_n  : in  std_logic;
      i_Binary   : in  std_logic_vector(13 downto 0);
      i_DV_n     : in  std_logic;
      o_Ones     : out std_logic_vector(3 downto 0);
      o_Tens     : out std_logic_vector(3 downto 0);
      o_Hundreds : out std_logic_vector(3 downto 0);
      o_DV_n     : out std_logic
    );
  end component;

  -- PID controller
  component pid
    port (
      ADC_DATA : in  std_logic_vector(15 downto 0);
      DAC_DATA : out std_logic_vector(15 downto 0);
      CLK1     : in  std_logic
    );
  end component;

  -- UART
  component uart
    generic (
      divisor : integer := 208
    );
    port (
      clk       : in  std_logic;
      reset     : in  std_logic;
      txdata    : in  std_logic_vector(7 downto 0);
      rxdata    : out std_logic_vector(7 downto 0);
      wr        : in  std_logic;
      rd        : in  std_logic;
      tx_avail  : out std_logic;
      tx_busy   : out std_logic;
      rx_avail  : out std_logic;
      rx_full   : out std_logic;
      rx_error  : out std_logic;
      uart_rxd  : in  std_logic;
      uart_txd  : out std_logic
    );
  end component;

  ---------------------------------------------------------------------------
  -- Local signals
  signal reset_n      : std_logic;

  -- HC-SR04 wiring
  signal cnt_dv_n     : std_logic;
  signal echo_time    : std_logic_vector(23 downto 0);
  signal meas_distance: std_logic_vector(13 downto 0);
  signal meas_dv_n    : std_logic;
  signal meas_dv_n_q  : std_logic := '1';
  signal meas_strobe  : std_logic := '0'; -- one clk when new measurement latched
  signal dist_cm_corr : std_logic_vector(13 downto 0);

  -- PID wiring
  signal adc_data     : std_logic_vector(15 downto 0);
  signal dac_data     : std_logic_vector(15 downto 0);

  -- Servo position (0..127)
  signal servo_pos    : std_logic_vector(6 downto 0);

  -- UART wiring
  signal txdata       : std_logic_vector(7 downto 0);
  signal rxdata       : std_logic_vector(7 downto 0);
  signal wr           : std_logic := '0';
  signal rd           : std_logic := '0';
  signal tx_avail     : std_logic;
  signal tx_busy      : std_logic;
  signal rx_avail     : std_logic;
  signal rx_full      : std_logic;
  signal rx_error     : std_logic;

  -- Telemetry formatting
  signal bcd_ones     : std_logic_vector(3 downto 0);
  signal bcd_tens     : std_logic_vector(3 downto 0);
  signal bcd_hund     : std_logic_vector(3 downto 0);
  signal bcd_dv_n     : std_logic;

  type tx_state_t is (TX_IDLE, TX_D, TX_EQ,
                      TX_H, TX_T, TX_O, TX_N, TX_ES, -- hundreds, tens, ones
                      TX_SPACE,
                      TX_S, TX_EQ2,
                      TX_P2, TX_P1, TX_P0,
                      TX_CR, TX_LF);
  signal tx_state : tx_state_t := TX_IDLE;
  signal pos_u    : unsigned(6 downto 0);

  -- Servo PWM: 50 Hz @ system clock
  signal pwm_count     : unsigned(19 downto 0) := (others => '0'); -- enough for 480000 < 2^19? 2^19=524288 OK
  signal period_ticks  : integer := 0;
  signal high_ticks    : unsigned(19 downto 0) := (others => '0');

begin
  reset_n <= not reset;

  -- Map LED status
  -- leds(0): measurement strobe, leds(1): near setpoint (|error|<=1cm), leds(3..2): heartbeat divider
  leds(0) <= meas_strobe;
  leds(2) <= pwm_count(18);
  leds(3) <= pwm_count(19);

  ---------------------------------------------------------------------------
  -- HC-SR04 interface (direct instantiation to control generics)
  u_counter: counter
    port map (
      i_Clock           => clk,
      i_Reset_n         => reset_n,
      i_Echo            => hcsr04_echo,
      o_DV_n            => cnt_dv_n,
      o_Echo_pulse_time => echo_time
    );

  u_trigger: trigger_generator
    port map (
      i_Clock   => clk,
      i_Reset_n => reset_n,
      o_Trigger => hcsr04_trig
    );

  -- Measurement calculation with adjusted ns per clock (approximate 42ns @ 24MHz)
  u_meas: measurement_cal
    port map (
      i_Clock           => clk,
      i_Reset_n         => reset_n,
      i_Echo_pulse_time => echo_time,
      i_DV_n            => cnt_dv_n,
      o_Distance        => meas_distance,
      o_DV_n            => meas_dv_n
    );

  -- Detect rising-edge of meas_dv_n (low-active valid). New data when returning to '1'.
  process(clk)
  begin
    if rising_edge(clk) then
      meas_dv_n_q <= meas_dv_n;
      if (meas_dv_n_q = '0' and meas_dv_n = '1') then
        meas_strobe <= '1';
      else
        meas_strobe <= '0';
      end if;
    end if;
  end process;

  ---------------------------------------------------------------------------
  -- Correct distance scaling for 24 MHz clock when measurement_cal assumes 20 ns cycles (50 MHz)
  -- dist_cm_corr ≈ meas_distance * (period_actual / 20ns) = meas_distance * (41.6667/20) = * 25/12
  process(clk)
    variable du      : unsigned(13 downto 0);
    variable mult    : unsigned(23 downto 0);
    variable divided : unsigned(23 downto 0);
  begin
    if rising_edge(clk) then
      du := unsigned(meas_distance);
      mult := resize(du, 24) * to_unsigned(25, 24);
      divided := mult / to_unsigned(12, 24);
      if divided > to_unsigned(16383, 24) then
        dist_cm_corr <= (others => '1');
      else
        dist_cm_corr <= std_logic_vector(resize(divided, 14));
      end if;
    end if;
  end process;

  -- PID input: 16-bit value with centimeters in lower 14 bits
  adc_data <= "00" & dist_cm_corr;

  u_pid: pid
    port map (
      ADC_DATA => adc_data,
      DAC_DATA => dac_data,
      CLK1     => clk
    );

  -- Map PID output (16-bit) to servo_pos (7-bit). Simple linear map with inversion for direction if needed.
  -- Use the top 7 bits to cover full range.
  servo_pos <= std_logic_vector(unsigned(dac_data(15 downto 9)));
  pos_u <= unsigned(servo_pos);

  ---------------------------------------------------------------------------
  -- Servo PWM (50Hz) from system clock
  -- Period = 20ms => ticks_per_period = CLK_FREQ_HZ / 50
  -- High  = 1.0ms..2.5ms => high_ticks = 0.001*CLK + pos*(0.0015*CLK)/127
  -- Implement integer math using constants derived from CLK_FREQ_HZ

  -- precompute period ticks once
  process(clk, reset)
    constant ticks_per_period : integer := CLK_FREQ_HZ / 50; -- 20ms
    constant ticks_1ms        : integer := CLK_FREQ_HZ / 1000; -- 1ms
    constant ticks_2_5ms      : integer := (CLK_FREQ_HZ * 25) / 10000; -- 2.5ms
    constant delta_ticks      : integer := ticks_2_5ms - ticks_1ms; -- span for 0..127
    variable hticks           : integer;
    variable interp           : integer;
  begin
    if reset = '1' then
      pwm_count   <= (others => '0');
      high_ticks  <= (others => '0');
      period_ticks <= 0;
      servo       <= '0';
    elsif rising_edge(clk) then
      -- latch constants into integer register for synthesis friendliness
      period_ticks <= ticks_per_period;
      -- compute high_ticks based on pos (0..127)
      interp := (delta_ticks * to_integer(pos_u) + 63) / 127; -- rounded
      hticks := ticks_1ms + interp; -- in [1ms .. 2.5ms]
      if hticks < 0 then hticks := 0; end if;
      if hticks > ticks_per_period-1 then hticks := ticks_per_period-1; end if;
      high_ticks <= to_unsigned(hticks, high_ticks'length);

      -- counter
      if pwm_count = to_unsigned(ticks_per_period-1, pwm_count'length) then
        pwm_count <= (others => '0');
      else
        pwm_count <= pwm_count + 1;
      end if;

      -- output compare
      if pwm_count < high_ticks then
        servo <= '1';
      else
        servo <= '0';
      end if;
    end if;
  end process;

  ---------------------------------------------------------------------------
  -- UART instance and simple telemetry state machine
  u_uart: uart
    generic map (
      divisor => UART_DIVISOR
    )
    port map (
      clk       => clk,
      reset     => reset,
      txdata    => txdata,
      rxdata    => rxdata,
      wr        => wr,
      rd        => rd,
      tx_avail  => tx_avail,
      tx_busy   => tx_busy,
      rx_avail  => rx_avail,
      rx_full   => rx_full,
      rx_error  => rx_error,
      uart_rxd  => uart_rxd,
      uart_txd  => uart_txd
    );

  -- Convert distance to BCD for ASCII transmission
  u_b2b: binary_to_bcd
    port map (
      i_Clock    => clk,
      i_Reset_n  => reset_n,
      i_Binary   => dist_cm_corr,
      i_DV_n     => meas_dv_n,
      o_Ones     => bcd_ones,
      o_Tens     => bcd_tens,
      o_Hundreds => bcd_hund,
      o_DV_n     => bcd_dv_n
    );

  -- Telemetry: D=hhh S=ppp\r\n (send on new measurement)
  process(clk, reset)
    function to_ascii(d : std_logic_vector(3 downto 0)) return std_logic_vector is
      variable u : unsigned(3 downto 0) := unsigned(d);
      variable a : unsigned(7 downto 0);
    begin
      a := to_unsigned(48,8) + resize(u, 8); -- '0' + digit
      return std_logic_vector(a);
    end function;
    -- simple helper to split pos into 3 decimal digits (0..127)
    variable pos_val  : integer;
    variable p_h, p_t, p_o : integer;
  begin
    if reset = '1' then
      tx_state <= TX_IDLE;
      wr <= '0';
      txdata <= (others => '0');
    elsif rising_edge(clk) then
      wr <= '0';

      case tx_state is
        when TX_IDLE =>
          if meas_strobe = '1' then
            tx_state <= TX_D;
          end if;

        when TX_D =>
          if tx_avail = '1' then
            txdata <= x"44"; -- 'D'
            wr <= '1';
            tx_state <= TX_EQ;
          end if;

        when TX_EQ =>
          if tx_avail = '1' then
            txdata <= x"3D"; -- '='
            wr <= '1';
            tx_state <= TX_H;
          end if;

        when TX_H =>
          if tx_avail = '1' then
            txdata <= to_ascii(bcd_hund);
            wr <= '1';
            tx_state <= TX_T;
          end if;

        when TX_T =>
          if tx_avail = '1' then
            txdata <= to_ascii(bcd_tens);
            wr <= '1';
            tx_state <= TX_O;
          end if;

        when TX_O =>
          if tx_avail = '1' then
            txdata <= to_ascii(bcd_ones);
            wr <= '1';
            tx_state <= TX_SPACE;
          end if;

        when TX_SPACE =>
          if tx_avail = '1' then
            txdata <= x"20"; -- space
            wr <= '1';
            tx_state <= TX_S;
          end if;

        when TX_S =>
          if tx_avail = '1' then
            txdata <= x"53"; -- 'S'
            wr <= '1';
            tx_state <= TX_EQ2;
          end if;

        when TX_EQ2 =>
          if tx_avail = '1' then
            txdata <= x"3D"; -- '='
            wr <= '1';
            tx_state <= TX_P2;
          end if;

        when TX_P2 =>
          if tx_avail = '1' then
            pos_val := to_integer(pos_u);
            p_h := pos_val / 100;
            p_t := (pos_val / 10) mod 10;
            p_o := pos_val mod 10;
            txdata <= std_logic_vector(to_unsigned(48 + p_h, 8));
            wr <= '1';
            tx_state <= TX_P1;
          end if;

        when TX_P1 =>
          if tx_avail = '1' then
            pos_val := to_integer(pos_u);
            p_t := (pos_val / 10) mod 10;
            txdata <= std_logic_vector(to_unsigned(48 + p_t, 8));
            wr <= '1';
            tx_state <= TX_P0;
          end if;

        when TX_P0 =>
          if tx_avail = '1' then
            pos_val := to_integer(pos_u);
            p_o := pos_val mod 10;
            txdata <= std_logic_vector(to_unsigned(48 + p_o, 8));
            wr <= '1';
            tx_state <= TX_CR;
          end if;

        when TX_CR =>
          if tx_avail = '1' then
            txdata <= x"0D"; -- CR
            wr <= '1';
            tx_state <= TX_LF;
          end if;

        when TX_LF =>
          if tx_avail = '1' then
            txdata <= x"0A"; -- LF
            wr <= '1';
            tx_state <= TX_IDLE;
          end if;
        when others =>
          -- default recovery
          tx_state <= TX_IDLE;
      end case;
    end if;
  end process;

  -- LED[1] indicates near setpoint (|Error| <= 1 cm): compare corrected distance to 33 cm
  process(clk)
    variable dist_cm : integer;
  begin
    if rising_edge(clk) then
      dist_cm := to_integer(unsigned(dist_cm_corr));
      if (dist_cm >= 32) and (dist_cm <= 34) then
        leds(1) <= '1';
      else
        leds(1) <= '0';
      end if;
    end if;
  end process;

end rtl;


