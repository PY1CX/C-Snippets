library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;


entity counter_190 is
   port(
	 enable: 	in 	std_logic;
	 direction: in 	std_logic;
 	 clock: 		in 	std_logic;
 	 reset: 		in 	std_logic;
 	 output: 	out 	std_logic_vector(0 to 3));
end counter_190;
 
architecture Behavioral of counter_190 is
   signal temp: std_logic_vector(0 to 3);
	
begin   process(clock, reset, direction, enable)
   begin
      --if reset='1' then
      --   temp <= "0000";	
		---end if;
		
		if reset='1' then
			temp <= "0000";
			-- Clock Rising Edge Detection
			elsif(rising_edge(clock)) then
					-- When direction is LOW ALS190 counts UP
					if enable = '0' and direction = '0' then
						if temp="1010" then
							temp<="0000";
						else
							temp <= temp + 1;
						end if;
					end if;
					-- When direction is HIGH ALS190 counts DOWN
					if enable = '0' and direction = '1' then
						if temp="0000" then
							temp<="1010";
						else
							temp <= temp - 1;
						end if;
					end if;
		end if;

end process;
   output <= temp;
end Behavioral;
