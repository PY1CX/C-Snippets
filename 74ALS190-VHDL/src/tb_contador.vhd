-- TestBench Contador


entity teste is end;

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

use std.textio.all;

architecture counter of teste is
  
component counter_190
   port( enable: in std_logic;
 	 clock: in std_logic;
 	 reset: in std_logic;
 	 output: out std_logic_vector(0 to 3));
end component;

-- Sinais auxiliares

signal clk_1    : std_logic;
signal enable_1 : std_logic;
signal reset_1  : std_logic;
signal output_1 : std_logic_vector(0 to 3);
  
begin 
  
counter1: counter_190 port map ( enable => enable_1, clock => clk_1, reset => reset_1, output => output_1 );

-- estimulo

estimulo : process
begin
  reset_1 <= '1'; enable_1 <= '1';
  wait for 5 ns;   
  enable_1 <= '0'; reset_1 <= '0';
  wait for 80 ns;
end process estimulo;

clock_estimulo : process

begin
  clk_1 <= '0';
  wait for 1 ns;
  clk_1 <= '1';
  wait for 1 ns;

end process clock_estimulo;

end counter;