library IEEE;
use IEEE.std_logic_1164.ALL;

use IEEE.NUMERIC_STD.ALL;

entity cpld_tester is
    Port ( 
       clock : in   std_logic;
       leds  : out  std_logic_vector (31 downto 0)
    );
end cpld_tester;

architecture Behavioral of cpld_tester is

signal selectOut : std_logic;
signal flash     : std_logic;

signal   chaser         : std_logic_vector(7 downto 0);
constant prescaler_max  : integer := 255;
signal   prescaler      : integer range 0 to prescaler_max;
constant count_max      : integer := 31;
signal   count          : integer range 0 to count_max;

begin

   leds <= chaser&chaser&chaser&chaser;
   
   process(clock)
   begin
   if rising_edge(clock) then
      prescaler <= prescaler + 1;
      if (prescaler = prescaler_max) then
         prescaler <= 0;
         count  <= count + 1;
         flash  <= not flash;
         if (selectOut = '0') then
            chaser <= (others => flash);
         else
            chaser <= chaser(0) & chaser(chaser'left downto 1);
         end if;
      end if;
      if (count = count_max) then
         count <= 0;
         selectOut <= not selectOut;
         chaser <= (0=>'0', others => '1');
      end if;
   end if;
   end process;
      
end Behavioral;
