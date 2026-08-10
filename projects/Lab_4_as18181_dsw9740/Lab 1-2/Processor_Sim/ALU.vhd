library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.common.all;

entity ALU is
	port ( 	operand_1 : in STD_LOGIC_VECTOR (13 downto 0);
	 	operand_2 : in STD_LOGIC_VECTOR (13 downto 0);

	 	opcode : in opcode_type;

		result : out STD_LOGIC_VECTOR (13 downto 0);
		overflow : out STD_LOGIC

	     );
end ALU;

architecture Behavioral of ALU is

signal result_internal: STD_LOGIC_VECTOR (13 downto 0);

begin

result <= result_internal;

calculate : process (operand_1, operand_2, opcode)
begin
	-- Signed addition for  OP_ADD, OP_ADDI, OP_BE, OP_BLT, and OP_JMP
	if ((opcode = OP_ADD) or (opcode = OP_ADDI) or (opcode = OP_BE) or (opcode = OP_BLT) or (opcode = OP_JMP)) then

		result_internal <= std_logic_vector( signed(operand_1) + signed(operand_2) );
		
	-- Signed subtraction for both OP_SUB and OP_SUBI
	elsif ((opcode = OP_SUB) or (opcode = OP_SUBI)) then
		result_internal <= std_logic_vector(signed(operand_1) - signed(operand_2));
		 
	-- Bitwise AND for both register and immediate versions
	elsif ((opcode = OP_AND) or (opcode = OP_ANDI)) then
		result_internal <= operand_1 and operand_2;
		
	-- Bitwise OR for both register and immediate versions
	elsif ((opcode = OP_OR) or (opcode = OP_ORI)) then
		result_internal <= operand_1 or operand_2;
		
	-- Bitwise XOR for both register and immediate versions
	elsif ((opcode = OP_XOR) or (opcode = OP_XORI)) then
		result_internal <= operand_1 xor operand_2;
		
	-- Shift Logical Left
	elsif opcode = OP_SLL then
		result_internal <= std_logic_vector( shift_left(unsigned(operand_1), to_integer(unsigned(operand_2))) );
		
	-- Shift Logical Right 
	elsif opcode = OP_SRL then
		result_internal <= std_logic_vector(shift_right(unsigned(operand_1), to_integer(unsigned(operand_2))));
		
	-- For HALT, output undefined value
	elsif opcode = OP_HALT then
		result_internal <= (others => '0'); 
		
	-- Default: assign undefined value
	else
		result_internal <= (others => '0');
end if;
end process;

-- Detection of overflow for all signed arithmetic operations
ofl : process (operand_1, operand_2, result_internal, opcode)
begin
overflow <= '0'; -- Default case: No overflow
  if (opcode = OP_ADD or opcode = OP_ADDI) then
		if ((operand_1(13) = operand_2(13)) and (result_internal(13) /= operand_1(13))) then  -- For addition (OP_ADD, OP_ADDI), overflow occurs if both operands have the same sign
																														  -- but the result has a different sign (indicating overflow).
			 overflow <= '1';
		end if;
		
		
  elsif (opcode = OP_SUB or opcode = OP_SUBI) then
		if ((operand_1(13) /= operand_2(13)) and (result_internal(13) /= operand_1(13))) then -- For subtraction (OP_SUB, OP_SUBI), overflow occurs if the operands have different signs
																														  -- but the result has a different sign than operand_1 (indicating overflow).
			 overflow <= '1';
		end if;
  end if;
end process ofl;		

end Behavioral;
