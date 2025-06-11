library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity servo_pwm_clk18kHz is
    PORT(
        clk  : IN  STD_LOGIC;
        reset: IN  STD_LOGIC;
        pos  : IN  STD_LOGIC_VECTOR(6 downto 0);
        servo: OUT STD_LOGIC
    );
end servo_pwm_clk18kHz;

architecture Behavioral of servo_pwm_clk18kHz is
    COMPONENT clk18kHz
        PORT(
            clk    : in  STD_LOGIC;
            reset  : in  STD_LOGIC;
            clk_out: out STD_LOGIC
        );
    END COMPONENT;
    
    COMPONENT servo_pwm
        PORT (
            clk   : IN  STD_LOGIC;
            reset : IN  STD_LOGIC;
            pos   : IN  STD_LOGIC_VECTOR(6 downto 0);
            servo : OUT STD_LOGIC
        );
    END COMPONENT;
    
    signal clk_out : STD_LOGIC := '0';
begin
    clk18kHz_map: clk18kHz PORT MAP(
        clk, reset, clk_out
    );
    
    servo_pwm_map: servo_pwm PORT MAP(
        clk_out, reset, pos, servo
    );
end Behavioral;