module instruction_decoder (
				state,
				instruction_register
			);

input [15:0] instruction_register;
output [7:0] state;

reg [7:0] state;

// State Encodings [Hint: same as the one from main file tc140L.v]
parameter	
		fetch		= 8'h01,//*
		reset_pc	= 8'h00,//*
		execute_and	= 8'h02,
		execute_add 	= 8'h03,
		execute_store 	= 8'h04,
		execute_jneg	= 8'h05,
		execute_or	= 8'h06,
		execute_load	= 8'h07,
		execute_jump	= 8'h08,
		execute_xor	= 8'h09,
		execute_out     = 8'h0a,
		execute_addi	= 8'h0b,
		execute_sub	= 8'h0c,
		execute_shl 	= 8'h0d,
		execute_shr	= 8'h0e,
		execute_jpos	= 8'h0f,
		decode		= 8'h10,
		execute_jzero	= 8'h11;
always@(*)
begin
case(instruction_register[15:12])
	// Write your Instruction Decode code here
		fetch:
			state <=8'h01;
		execute_and:
			state <= 8'h02;
		execute_add:
			state <= 8'h03;
		execute_store:
			state <=8'h04;	
		execute_jneg:
			state <= 8'h05;
		execute_or:
			state <=8'h06;
		execute_load:
			state <= 8'h07;
		execute_jump:
			state <= 8'h08;
		execute_xor:
			state <=8'h09;
		execute_out:
			state <=8'h0a;
		execute_addi:
			state <=8'h0b;
		execute_sub:
			state <=8'h0c;
		execute_shl:
			state <=8'h0d;
		execute_shr:
			state <=8'h0e;
		execute_jpos:
			state <=8'h0f;
	endcase
end		
endmodule 
