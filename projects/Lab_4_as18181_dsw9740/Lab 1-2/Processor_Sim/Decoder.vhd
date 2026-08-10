library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.common.all;

entity Decoder is
	port ( 	instruction_in : in STD_LOGIC_VECTOR (15 downto 0);

		opcode_out : out opcode_type;

		Rd_addr_out : out STD_LOGIC_VECTOR (2 downto 0);
		Rs1_addr_out : out STD_LOGIC_VECTOR (2 downto 0);
		Rs2_addr_out : out STD_LOGIC_VECTOR (2 downto 0);

		immediate_out : out STD_LOGIC_VECTOR (13 downto 0)
	     );
end Decoder;

architecture Behavioral of Decoder is

--TODO add signals as needed
signal opcode_internal : opcode_type;
signal Rd, Rs1, Rs2, immediate_val : STD_LOGIC_VECTOR(2 downto 0);

begin
	opcode_internal <= std_logic_vector_to_opcode_type( instruction_in(15 downto 12) );
	Rd <= instruction_in(11 downto 9);
	Rs1 <= instruction_in(8 downto 6);
	Rs2 <= instruction_in(5 downto 3);
	immediate_val <= instruction_in(2 downto 0);

	opcode_out <= opcode_internal;
	Rd_addr_out <= Rd;
	Rs1_addr_out <= Rs1;
	Rs2_addr_out <= Rs2;
	
	process(opcode_internal, Rs2, Rd, immediate_val)
		variable temp1 : STD_LOGIC_VECTOR(5 downto 0);
		variable temp2 : STD_LOGIC_VECTOR(5 downto 0);
	begin
		temp1 := Rs2 & immediate_val;
		temp2 := Rd & immediate_val;

		case opcode_internal is
			when OP_ANDI =>
				immediate_out <= "11111111" & temp1;
			when OP_ORI | OP_XORI=>
				immediate_out <= "00000000" & temp1;
			when OP_ADDI | OP_SUBI =>
				immediate_out <= (7 downto 0 => temp1(5)) & temp1;
			when OP_SLL | OP_SRL =>
				immediate_out <= (10 downto 0 => '0') & immediate_val;
			when OP_BLT | OP_BE | OP_JMP =>
				immediate_out <= (7 downto 0 => Rd(2)) & temp2;
			when others =>
				immediate_out <= (others => '0');
        end case;
    end process;
end Behavioral;
