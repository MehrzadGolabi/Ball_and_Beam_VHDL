library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_top_ball_beam is
end tb_top_ball_beam;

architecture sim of tb_top_ball_beam is

  -- DUT component
  component top_ball_beam
    generic (
      CLK_FREQ_HZ : integer := 24000000;
      UART_DIVISOR: integer := 208
    );
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      hcsr04_echo : in  std_logic;
      hcsr04_trig : out std_logic;
      servo       : out std_logic;
      uart_rxd    : in  std_logic;
      uart_txd    : out std_logic;
      leds        : out std_logic_vector(3 downto 0)
    );
  end component;

  -- Signals
  signal clk         : std_logic := '0';
  signal reset       : std_logic := '1';
  signal hcsr04_echo : std_logic := '0';
  signal hcsr04_trig : std_logic;
  signal servo       : std_logic;
  signal uart_rxd    : std_logic := '1'; -- idle high
  signal uart_txd    : std_logic;
  signal leds        : std_logic_vector(3 downto 0);

  constant clk_period : time := 42 ns; -- ~23.81 MHz (approx 24 MHz)

begin

  -- Clock generation
  clk_proc: process
  begin
    clk <= '1';
    wait for clk_period/2;
    clk <= '0';
    wait for clk_period/2;
  end process;

  -- DUT
  dut: top_ball_beam
    port map (
      clk         => clk,
      reset       => reset,
      hcsr04_echo => hcsr04_echo,
      hcsr04_trig => hcsr04_trig,
      servo       => servo,
      uart_rxd    => uart_rxd,
      uart_txd    => uart_txd,
      leds        => leds
    );

  -- Stimulus: generate echo pulses for different distances
  stim: process
    procedure emit_distance(distance_cm : integer; gap : time) is
      variable pulse : time;
    begin
      -- wait gap before sending new echo
      wait for gap;
      -- HC-SR04 echo high width ≈ distance_cm * 58 us
      pulse := (distance_cm * 58) * 1 us;
      hcsr04_echo <= '1';
      wait for pulse;
      hcsr04_echo <= '0';
    end procedure;
  begin
    -- Reset
    reset <= '1';
    wait for 1 ms;
    reset <= '0';
    
    -- Sequence of distances to exercise the controller around setpoint (33 cm)
    emit_distance(50,  1 ms);  -- far
    emit_distance(45,  4 ms);
    emit_distance(40,  4 ms);
    emit_distance(35,  4 ms);
    emit_distance(33,  4 ms);  -- near setpoint
    emit_distance(30,  4 ms);
    emit_distance(25,  4 ms);
    emit_distance(33,  4 ms);
    emit_distance(20,  4 ms);  -- close
    emit_distance(33,  4 ms);
    emit_distance(28,  4 ms);
    emit_distance(33,  4 ms);

    -- Let it run for a while to observe PWM and UART
    wait for 60 ms;
    
    -- Hold simulation (finish externally)
    wait;
  end process;

end sim;


