module instruction_fetch
(
				instruction_register,
				memory_data_register,
				program_counter,
				incremented_program_counter
);

input [15:0] memory_data_register;
input [7:0] program_counter;
output [15:0] instruction_register;
output [7:0] incremented_program_counter;

reg [15:0] instruction_register;
reg [7:0] incremented_program_counter;

always @(*)
begin
// Write your Instrcution Fetch code here
	incremented_program_counter<=program_counter+1;
	instruction_register<=memory_data_register;
end
endmodule 
/*
module mux3ch(muxout,dataa,datab,datac,sel);
parameter wordsize=16;
output[wordsize-1:0] muxout;
input [word_size-1:0] dataa,datab,datac;
input [1:0]sel;
assign mux_out= (sel=0)?dataa:(sel=1)?datab:(sel=2)?datac:'bx;
endmodule 

module address_register(data_out,data_in,load,clk,rst);
parameter word_size=8;
output[wordsize-1:0]data_out;
input [wordsize-1:0]data_in;
input load,clk,rst;
reg data_out;
always@(posedge clk or negedge reset)
begin
	if(rst==0)begin
			data_out<=0;
		end
else if (load)
begin
	data_out <=data_in;
end
end
endmodule


module instruction_register(data_out,data_in,load,clk,rst);
parameter word_size=8;
output[wordsize-1:0]data_out;
input [wordsize-1:0]data_in;
input load,clk,rst;
reg data_out;
always@(posedge clk or negedge reset)
begin
	if(rst==0)begin
			data_out<=0;
		end
else if (load)
begin
	data_out <=data_in;
end
end
endmodule

module alu_RISC();
parameter word_size =16;
parameter op_size =8;

//insert opcodes here

output [word_size-1:0]alu_out;
output alu_zero_flag;
input [word_size-1:0]data_1,data_2;
input [op_code-1:0] sel;
reg alu_out;
assign alu_zero_flag=~|alu_out;
always@(sel or data_1 or data_2)
begin
case(sel)

//insert opcodes here

endcase
end

endmodule


module register_unit(data_out,data_in,load,clk,rst);
parameter word_size=8;
output[wordsize-1:0]data_out;
input [wordsize-1:0]data_in;
input load,clk,rst;
reg data_out;
always@(posedge clk or negedge reset)
begin
	if(rst==0)begin
			data_out<=0;
		end
else if (load)
begin
	data_out <=data_in;
end
end
endmodule



module dflipflop(data_out,data_in,clk,reset,load);
input clk,reset;
output data_out;
input data_in;
input load;
always@(posedge clk or negedge reset)
begin
	if(reset==0)
	begin
		data_out <=0;
	end
	else if(load==1)
	begin
		data_out <= data_in;
	end
end
endmodule 

module Program_counter(count,datain,loadpc,incpc,clk,rst);
begin
	parameter word_size=8;
	output [wordsize-1:0]count;
	input [wordsize-1:0]datain;
input loadpc;
input incpc
inout clk,rst;
reg count;
always(posedge clk or negedge rst)
begin
	if(rst ==0)
	begin
		count <=0;
	end
	else if(loadpc)
	begin
		count<=datain;
	end

	else if(incpc)
	begin
		count<=datain;
	end

	count <=count+1;
end
endmodule

module testbench();
reg rst;
wire clk;
parameter word_size=16;
reg [8:0]k;


endmodule

module tb();
	reg rst;
	wire clk;
	parameter word_size =16;
	reg [8:0] k;
	Clock_Unit M1(clk);
	RISC_SPM M2(clk,rst);

	wire [word_size-1:0] word0,word1,word2,word3,word4,word5,word6,word7,word8,word9,
	word10,word11,word12,word13,word14,word15,word16,word17,word18,word19,
word20,word21,word22,word23,word24,word25,word26,word27,word28,word29,
word30,word31,word32,word33,word34,word35,word36,word37,word38,word39,

word40,word41,word42,word43,word44,word45,word46,word47,word48,word49,
word50,word51,word52,word53,word54,word55,word56,word57,word58,word59,
word60,word61,word62,word63,word64,word65,word66,word67,word68,word69,
word70,word71,word72,word73,word74,word75,word76,word77,word78,word79,
word80,word81,word82,word83,word84,word85,word86,word87,word88,word89,

word90,word91,word92,word93,word94,word95,word96,word97,word98,word99,
word100,word101,word102,word103,word104,word105,word106,word107,word108,word109,
word110,word111,word112,word113,word114,word115,word116,word117,word118,word119,
word120,word121,word122,word123,word124,word125,word126,word127,word128,word129,
word130,word131,word132,word133,word134,word135,word136,word137,word138,word139,

word140,word141,word142,word143,word144,word145,word146,word147,word148,word149,
word150,word151,word152,word153,word154,word155,word156,word157,word158,word159,
word160,word161,word162,word163,word164,word165,word166,word167,word168,word169,
word170,word171,word172,word173,word174,word175,word176,word177,word178,word179,
word180,word181,word182,word183,word184,word185,word186,word187,word188,word189,

word190,word191,word192,word193,word194,word195,word196,word197,word198,word199,


word200,word201,word202,word203,word204,word205,word206,word207,word208,word209,
word210,word211,word212,word213,word214,word215,word216,word217,word218,word219,
word220,word221,word222,word223,word224,word225,word226,word227,word228,word229,
word230,word231,word232,word233,word234,word235,word236,word237,word238,word239,

word240,word241,word242,word243,word244,word245,word246,word247,word248,word249,
word250,word251,word252,word253,word254,word255;
endmodule


module Controlunit
(
loadpc,incpc,load_r0,
load_r1,load_r2,load_r3,
loadpc,incpc,sel_bus_1_mux,
sel_bus_2_mux,load_ir,load_add_r,
load_reg_y,load_reg_z,write,
instruction,zero,clk,rst
);
	parameter word_size =16;
	parameter op_size =8;
	parameter srcs_size =2,dest_size=2,sel1_size=3,sel2_size=2;
	//state code
	//**********
		
	//***********
	//opcode
	//***********
		
	//***********
//source and destination codes
parameter r0 =0,r1=1,r2=2;,r3=3;

output loadpc,incpc;
output load_r0,load_r1,load_r2,load_r3;
output[sel1_size-1:0]sel_bus_1_mux;
output load_ir,load_reg_r;
output load_reg_y,load_reg_z;
output[sel2_size-1:0]sel_bus_2_mux;
output write;
input [word_size-1:0] instruction;
input zero;
input clk,rst;
reg[state_size-1:0] state,next_state;
reg load_ro,load_r1,load_r2,load_r3,loadpc,incpc;
reg load_ir,load_add_r,load_reg_y;
reg sel_alu,sel_bus_1,sel_mem;
reg sel_r0,sel_r1,sel_r2,sel_r3,sel_pc;
reg load_reg_z,write;
reg err_flag;

wire[op_size-1:0]opcode = instruction[word_size-1:word_size-op_size];
wire[src_size-1:0] src =  instruction[src_size+dst_size-1:dest_size];
wire[dest_size-1:0] dest = instruction [dest_size-1:0];
assign sel_bus_1_mux[sel1_size-1:0] = sel_r0?0:sel_r1?1:sel_r3?2:sel_r2?2:
assign sel_bus_2_mux[sel2_size-1:0] = sel_alu?0:
sel_bus_1?1:
sel_mem?2:2'bx;

always@(posedge clk or negedge rst)
begin:state_transitions
	if(rst==0)
	begin
		state <=s_idle;
	end
	else
	begin
		state <=next_state;
	end
end

always@(state or opcode or src or dest or zero)
begin: output_and_next_state
	sel_r0 = 0;sel_r1 = 0; sel_r2=0;sel_r3=0;
sel_pc =0;load_r0 = 0;load_r1=0;load_r2=0;
load_r3 = 0;loadpc=0;

load_ir=0;load_add_r=0;load_reg_y=0;load_reg_z=0;
incpc=0;sel_bus_1=0;sel_alu=0;sel_mem=0;
write =0; err_flag=0;next_state=state;

case(state)
	s_idle: next_state = s_fet1;
	s_fet1:
		begin
			next_state = s_fet2;
			sel_pc =1;
			sel_bus_1 =1;
			load_add_r=1;
		end
	s_fet2:
		begin
			next_state = s_dec;
			sel_mem =1;
			load_ir =1;
			incpc=1;
		end
	s_dec:
		begin
			case(opcode)
			//insert opcode here!!!!
			endcase
		end
	NOT:
		begin
		next_state =s_fet1;
		load_reg_z=1;
		sel_bus_1=1;
		sel_alu =1;
		case(src)
			r0:	sel_r0=1;
			r1:	sel_r1=1;
			r2:	sel_r2=1;
			r3:	sel_r3=1;
		endcase	
		case(dest)
			r0: 	 load_r0=1;
			r1: 	 load_r1=1;
			r2:	 load_r2=1;
			r3:	 load_r3=1;
			default: err_flag=1;
		endcase
		end
	RD:
	begin
		next_state = s_rd1;
		sel_pc = 1; sel_bus_1=1;load_add_r=1;
	end

	wr:
	begin
		next_State = s_wr1;
		sel_pc=1;sel_bus_1=1;load_add_r=1;
	end

	br:
	begin
		next_state =s_br1;
		sel_pc=1;sel_bus_1=1;load_add_r=1;
	end
	brz:
	begin
		if(zero==1)
		begin
			next_state=s_br1;
			sel_pc=1;sel_bus_1=1;load_add_r=1;
		end
		else begin
			next_state =s_fet1;
			incpc=1;
		end
		default:  next_state = s_halt;
	end
	s_ex1:
		begin
			next_state = s_fet1;
			load_reg_z=1;
			sel_alu=1;
			case(dest)
				r0:begin sel_r0 =1; load_r0 =1;end
				r1:begin sel_r1 =1; load_r1 =1;end
				r2:begin sel_r2 =1; load_r2 =1;end
				r3:begin sel_r3 =1; load_r3 =1;end

			endcase
		end

endcase
endcase
endmodule

module mux3ch(muxout,dataa,datab,datac,sel);
parameter wordsize=16;
output[wordsize-1:0] muxout;
input [word_size-1:0] dataa,datab,datac,datad,datae;
input [1:0]sel;
assign mux_out= (sel=0)?dataa:(sel=1)?datab:(sel=2)?datac:(sel=3)?datad:(sel=4):datae:(sel=5):'bx;
endmodule 
*/