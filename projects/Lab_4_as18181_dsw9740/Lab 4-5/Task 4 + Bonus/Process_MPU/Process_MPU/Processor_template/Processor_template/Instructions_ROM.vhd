library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.common.all;

entity Instructions_ROM is
    port (
        address_in : in STD_LOGIC_VECTOR (6 downto 0);
        data_out   : out STD_LOGIC_VECTOR (15 downto 0)
    );
end Instructions_ROM;

architecture Behavioral of Instructions_ROM is
    type ROM_type is array (0 to 127) of STD_LOGIC_VECTOR (15 downto 0);
    signal rom : ROM_type;
begin
    data_out <= rom(to_integer(unsigned(address_in)));

    rom_process : process(address_in)
    begin
        -- Reset ROM content completely with HLT operations.
        for i in 0 to 127 loop
            rom(i) <= opcode_type_to_std_logic_vector(OP_HALT) & (11 downto 0 => 'X');
        end loop;
        
        -- Initialize R7 with 1.
        rom(0) <= opcode_type_to_std_logic_vector(OP_ADDI) & b"111" & b"000" & b"000_001";
          -- This loads R7 (register "111") with 1.

        -- Load n into R1 (to calculate the nth Fibonacci term).
        rom(1) <= opcode_type_to_std_logic_vector(OP_ADDI) & b"001" & b"000" & b"000_110";
          -- Loads 6 into R1 (so we compute the 7th Fibonacci term).

        --  Initialize R2 (prev1) to 0.
        rom(2) <= opcode_type_to_std_logic_vector(OP_ADDI) & b"010" & b"000" & b"000_000";
          -- R2 = 0.

        -- Initialize R3 (prev2) to 1.
        rom(3) <= opcode_type_to_std_logic_vector(OP_ADDI) & b"011" & b"000" & b"000_001";
          -- R3 = 1.

        -- Branch if R1 < R7 (i.e. if R1 < 1) then skip loop to HALT.
        rom(4) <= opcode_type_to_std_logic_vector(OP_BLT) & b"000" & b"001" & b"111" & b"110";

        -- Calculate next Fibonacci term: R4 = R2 + R3.
        rom(5) <= opcode_type_to_std_logic_vector(OP_ADD) & b"100" & b"010" & b"011" & b"000";
          -- R4 = R2 + R3.

        -- Move R3 into R2.
        rom(6) <= opcode_type_to_std_logic_vector(OP_ADDI) & b"010" & b"011" & b"000_000";
          -- R2 = R3.

        -- Move R4 into R3.
        rom(7) <= opcode_type_to_std_logic_vector(OP_ADDI) & b"011" & b"100" & b"000_000";
          -- R3 = R4.

        -- Decrement n: R1 = R1 - 1.
        rom(8) <= opcode_type_to_std_logic_vector(OP_SUBI) & b"001" & b"001" & b"000001";

        -- Jump back to instruction 4 (start of loop).
        rom(9) <= opcode_type_to_std_logic_vector(OP_JMP) & b"000" & b"000" & b"111011";
          -- The immediate "111011" is assumed to represent -5.

        -- Halt the processor (nth Fibonacci term is in R3).
        rom(10) <= opcode_type_to_std_logic_vector(OP_HALT) & (11 downto 0 => '0');

        -- Dummy instructions (addresses 121 to 127) to infer all registers.
        rom(121) <= opcode_type_to_std_logic_vector(OP_ADD) & b"001_001_000_000";
        rom(122) <= opcode_type_to_std_logic_vector(OP_ADD) & b"010_010_001_000";
        rom(123) <= opcode_type_to_std_logic_vector(OP_ADD) & b"011_011_010_000";
        rom(124) <= opcode_type_to_std_logic_vector(OP_ADD) & b"100_100_011_000";
        rom(125) <= opcode_type_to_std_logic_vector(OP_ADD) & b"101_101_100_000";
        rom(126) <= opcode_type_to_std_logic_vector(OP_ADD) & b"110_110_101_000";
        rom(127) <= opcode_type_to_std_logic_vector(OP_ADD) & b"111_111_110_000";

    end process;
end Behavioral;
