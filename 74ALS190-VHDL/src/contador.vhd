library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;


entity counter_190 is
   port( enable: in std_logic;
 	 clock: in std_logic;
 	 reset: in std_logic;
 	 output: out std_logic_vector(0 to 3));
end counter_190;
 
architecture Behavioral of counter_190 is
   signal temp: std_logic_vector(0 to 3);
begin   process(clock,reset)
   begin
      if reset='1' then
         temp <= "0000";
      elsif(rising_edge(clock)) then
         if enable='0' then
            if temp="1001" then
               temp<="0000";
            else
               temp <= temp + 1;
            end if;
         end if;
      end if;
   end process;
   output <= temp;
end Behavioral;
