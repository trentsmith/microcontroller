//////////////////////////////////////////////////////////
        // ISA:			opcode		   |||
       // ADD addr		00		  // \\
      // ST addr		01		 //   \\
     // LD addr		02	                //-----\\
    // JMP addr		03	               //-------\\ 
   // single 16-bit accumulator		      //	 \\
  // addr is 8-bits			     //		  \\
 // for testing, internal busses are exposed//		   \\
///////////////////////////////////////////////////////////	
module everything();
/*parameter*/

reg[15:0] register_1;
reg [15:0] memory[0:255];
reg clock,reset;
reg [7:0] program_counter;
reg [15:0] memory_data_register;

wire [15:0]  memory_data_register_out;
reg  [15:0]   instruction_register;

reg [15:0] register_A,out;
reg  [7:0] state;
// State Encodings
reg [7:0] memory_address_register;
reg memory_write;


assign memory_data_register_out = memory_data_register;
wire [15:0] instruction_register_fetch;
wire [7:0] state_from_decode_unit;

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
	wire [15:0]word0 ,word1 ,word2 ,word3 ,word4 ,word5 ,word6 ,word7 ,word8 ,word9 ,
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

	assign word0 = memory[0];
	assign word1 = memory[1];
	assign word2 = memory[2];
	assign word3 = memory[3];
	assign word4 = memory[4];
	assign word5 = memory[5];
	assign word6 = memory[6];
	assign word7 = memory[7];
	assign word8 = memory[8];
	assign word9 = memory[9];
	assign word10 = memory[10];
	
	assign word11 = memory[11];
	assign word12 = memory[12];
	assign word13 = memory[13];
	assign word14 = memory[14];
	assign word15 = memory[15];
	assign word16 = memory[16];
	assign word17 = memory[17];
	assign word18 = memory[18];
	assign word19 = memory[19];
	
	assign word20 = memory[20];
	assign word21 = memory[21];
	assign word22 = memory[22];
	assign word23 = memory[23];
	assign word24 = memory[24];
	assign word25 = memory[25];
	assign word26 = memory[26];
	assign word27 = memory[27];
	assign word28 = memory[28];
	assign word29 = memory[29];

	assign word30 = memory[30];
	assign word31 = memory[31];
	assign word32 = memory[32];
	assign word33 = memory[33];
	assign word34 = memory[34];
	assign word35 = memory[35];
	assign word36 = memory[36];
	assign word37 = memory[37];
	assign word38 = memory[38];
	assign word39 = memory[39];
	
	assign word40 = memory[40];
	assign word41 = memory[41];
	assign word42 = memory[42];
	assign word43 = memory[43];
	assign word44 = memory[44];
	assign word45 = memory[45];
	assign word46 = memory[46];
	assign word47 = memory[47];
	assign word48 = memory[48];
	assign word49 = memory[49];
	
	assign word50 = memory[50];
	assign word51 = memory[51];
	assign word52 = memory[52];
	assign word53 = memory[53];
	assign word54 = memory[54];
	assign word55 = memory[55];
	assign word56 = memory[56];
	assign word57 = memory[57];
	assign word58 = memory[58];
	assign word59 = memory[59];
	
	assign word60 = memory[60];
	assign word61 = memory[61];
	assign word62 = memory[62];
	assign word63 = memory[63];
	assign word64 = memory[64];
	assign word65 = memory[65];
	assign word66 = memory[66];
	assign word67 = memory[67];
	assign word68 = memory[68];
	assign word69 = memory[69];
	
	assign word70 = memory[70];
	assign word71 = memory[71];
	assign word72 = memory[72];
	assign word73 = memory[73];
	assign word74 = memory[74];
	assign word75 = memory[75];
	assign word76 = memory[76];
	assign word77 = memory[77];
	assign word78 = memory[78];
	assign word79 = memory[79];
	
	assign word80 = memory[80];
	assign word81 = memory[81];
	assign word82 = memory[82];
	assign word83 = memory[83];
	assign word84 = memory[84];
	assign word85 = memory[85];
	assign word86 = memory[86];
	assign word87 = memory[87];
	assign word88 = memory[88];
	assign word89 = memory[89];
	
	assign word90 = memory[90];
	assign word91 = memory[91];
	assign word92 = memory[92];
	assign word93 = memory[93];
	assign word94 = memory[94];
	assign word95 = memory[95];
	assign word96 = memory[96];
	assign word97 = memory[97];
	assign word98 = memory[98];
	assign word99 = memory[99];
	
	assign word100 = memory[100];
	assign word101 = memory[101];
	assign word102 = memory[102];
	assign word103 = memory[103];
	assign word104 = memory[104];
	assign word105 = memory[105];
	assign word106 = memory[106];
	assign word107 = memory[107];
	assign word108 = memory[108];
	assign word109 = memory[109];
	assign word110 = memory[110];
	
	assign word111 = memory[111];
	assign word112 = memory[112];
	assign word113 = memory[113];
	assign word114 = memory[114];
	assign word115 = memory[115];
	assign word116 = memory[116];
	assign word117 = memory[117];
	assign word118 = memory[118];
	assign word119 = memory[119];
	
	assign word120 = memory[120];
	assign word121 = memory[121];
	assign word122 = memory[122];
	assign word123 = memory[123];
	assign word124 = memory[124];
	assign word125 = memory[125];
	assign word126 = memory[126];
	assign word127 = memory[127];
	assign word128 = memory[128];
	assign word129 = memory[129];
	
	assign word130 = memory[130];
	assign word131 = memory[131];
	assign word132 = memory[132];
	assign word133 = memory[133];
	assign word134 = memory[134];
	assign word135 = memory[135];
	assign word136 = memory[136];
	assign word137 = memory[137];
	assign word138 = memory[138];
	assign word139 = memory[139];

	assign word140 = memory[140];
	assign word141 = memory[141];
	assign word142 = memory[142];
	assign word143 = memory[143];
	assign word144 = memory[144];
	assign word145 = memory[145];
	assign word146 = memory[146];
	assign word147 = memory[147];
	assign word148 = memory[148];
	assign word149 = memory[149];
	
	assign word150 = memory[150];
	assign word151 = memory[151];
	assign word152 = memory[152];
	assign word153 = memory[153];
	assign word154 = memory[154];
	assign word155 = memory[155];
	assign word156 = memory[156];
	assign word157 = memory[157];
	assign word158 = memory[158];
	assign word159 = memory[159];

	assign word160 = memory[160];
	assign word161 = memory[161];
	assign word162 = memory[162];
	assign word163 = memory[163];
	assign word164 = memory[164];
	assign word165 = memory[165];
	assign word166 = memory[166];
	assign word167 = memory[167];
	assign word168 = memory[168];
	assign word169 = memory[169];
	
	assign word170 = memory[170];
	assign word171 = memory[171];
	assign word172 = memory[172];
	assign word173 = memory[173];
	assign word174 = memory[174];
	assign word175 = memory[175];
	assign word176 = memory[176];
	assign word177 = memory[177];
	assign word178 = memory[178];
	assign word179 = memory[179];
	
	assign word180 = memory[180];
	assign word181 = memory[181];
	assign word182 = memory[182];
	assign word183 = memory[183];
	assign word184 = memory[184];
	assign word185 = memory[185];
	assign word186 = memory[186];
	assign word187 = memory[187];
	assign word188 = memory[188];
	assign word189 = memory[189];
	
	assign word190 = memory[190];
	assign word191 = memory[191];
	assign word192 = memory[192];
	assign word193 = memory[193];
	assign word194 = memory[194];
	assign word195 = memory[195];
	assign word196 = memory[196];
	assign word197 = memory[197];
	assign word198 = memory[198];
	assign word199 = memory[199];
	
	assign word200 = memory[200];
	assign word201 = memory[201];
	assign word202 = memory[202];
	assign word203 = memory[203];
	assign word204 = memory[204];
	assign word205 = memory[205];
	assign word206 = memory[206];
	assign word207 = memory[207];
	assign word208 = memory[208];
	assign word209 = memory[209];
	assign word210 = memory[210];
	
	assign word211 = memory[211];
	assign word212 = memory[212];
	assign word213 = memory[213];
	assign word214 = memory[214];
	assign word215 = memory[215];
	assign word216 = memory[216];
	assign word217 = memory[217];
	assign word218 = memory[218];
	assign word219 = memory[219];
	
	assign word220 = memory[220];
	assign word221 = memory[221];
	assign word222 = memory[222];
	assign word223 = memory[223];
	assign word224 = memory[224];
	assign word225 = memory[225];
	assign word226 = memory[226];
	assign word227 = memory[227];
	assign word228 = memory[228];
	assign word229 = memory[229];
	
	assign word230 = memory[230];
	assign word231 = memory[231];
	assign word232 = memory[232];
	assign word233 = memory[233];
	assign word234 = memory[234];
	assign word235 = memory[235];
	assign word236 = memory[236];
	assign word237 = memory[237];
	assign word238 = memory[238];
	assign word239 = memory[239];
	
	assign word240 = memory[240];
	assign word241 = memory[241];
	assign word242 = memory[242];
	assign word243 = memory[243];
	assign word244 = memory[244];
	assign word245 = memory[245];
	assign word246 = memory[246];
	assign word247 = memory[247];
	assign word248 = memory[248];
	assign word249 = memory[249];

	assign word250 = memory[250];
	assign word251 = memory[251];
	assign word252 = memory[252];
	assign word253 = memory[253];
	assign word254 = memory[254];
	assign word255 = memory[255];
	instruction_fetch IF(
				instruction_register_fetch,
				memory[program_counter],
				program_counter,
				incremented_program_conter
			);
	instruction_decoder ID(
				state_from_decoded_unit,
				state
			);
// Implement main memory (256 16-bit words)
//mainm mm(instruction_register[15:0],register_A,program_counter,write,clock);
	always@(posedge clock)
	begin

		instruction_register[15:0] <= memory[program_counter];
		if(memory_write)
		begin
			instruction_register[15:0] <= register_A;
		end
	end

// insert your code here.//255-0 program counter
//15-0 instruction regisers


// Control Block : Control State Machine
// ALU operations can be seen as state actions in EXECUTE control state
always @(posedge clock or posedge reset)
     begin
        if (reset)
            state <= reset_pc;
        else 
		state <=state_from_decode_unit;
		case (state)
		// reset the computer, need to clear some registers
	       		reset_pc :
	       		begin
						program_counter <= 8'b00000000;
						register_A <= 16'b0000000000000000;
						out <= 16'b0000000000000000;
						state <= fetch;	
       		end
		// Fetch instruction from memory and add 1 to program counter
       		fetch :
       		begin
					instruction_register <= instruction_register_fetch;
					program_counter <= program_counter+1;
       		end
		// Decode instruction and send out address of any required data operands
       		decode:
       		begin
					state <= state_from_decode_unit;
       		end
		// Execute the ADD instruction
       		execute_add :
       		begin
					register_A <= register_1 + memory_data_register;
					state <= fetch;

       		end
		// Execute the STORE instruction (needs three clock cycles for memory write)
       		execute_store :
       		begin
		// write register_A to memory
 					state <= execute_store;
       		end
		// This state ensures that the memory address is valid until after memory_write goes low
		// Execute the LOAD instruction
       		execute_load :
       		begin
				register_A <= memory_data_register;
				state <= fetch;
       		end
 		// Execute the JUMP instruction
       		
		execute_jpos :
       		begin
			if(register_A > 0)
				begin
					program_counter <= instruction_register[7:0];
				end	
				state <= fetch;
		end

		execute_jneg :
       		begin
				if(register_A < 0)
				begin
					program_counter <= instruction_register[7:0];
				end	
			state <= fetch;
		end
		execute_jzero :
       		begin
			if(register_A == 0)
				begin
					program_counter <= instruction_register[7:0];
				end
				state <= fetch;
		end
		execute_jump :
       		begin
				program_counter <= instruction_register[7:0];
				state <= fetch;
       		end
	 	// Execute the OUT instruction      		
       		execute_out :
       		begin
				out <= register_A ;
				state <= fetch ;
       		end   		// default is fetch   
		execute_addi:
		begin
				register_A <= register_1 + instruction_register[7:0] ;
				state <= fetch;
		end
		execute_shl :
		begin
				register_A=register_1<<instruction_register[3:0];
				state<=fetch;
		end
		execute_shr: 	
		begin
				register_A=register_1>>instruction_register[3:0];
            			state <= fetch;
		end		    		
		execute_and:
		begin
			register_A <= register_1&memory_data_register;
			state <= fetch;

		end
		execute_sub:
		begin
			register_A <= register_1 - memory_data_register;
			state <= fetch;
		end

		execute_or:
		begin
			register_A <= register_1|memory_data_register;
			state <= fetch;

		end

		execute_xor:
		begin
			register_A <= register_1^memory_data_register;
			state <= fetch;

		end
       		default :
       		begin
            			state <= fetch;
       		end

		endcase
	program_counter<=program_counter +1;	
     end
// Memory Address Register Update Block:     
// Block to enable Memory Write
always@(register_1)
begin
	register_1 <= register_A;
end
always @(state_from_decode_unit or program_counter or instruction_register)
	begin
	case (state_from_decode_unit)
		reset_pc: 		memory_address_register <= 8'h 00;
		fetch:			memory_address_register <= program_counter;
		decode:			memory_address_register <= instruction_register[7:0];
		execute_add: 		memory_address_register <= program_counter;
		execute_store: 		memory_address_register <= instruction_register[7:0];
		//execute_store2: 	memory_address_register <= program_counter;
		execute_load:		memory_address_register <= program_counter;
		execute_jump:		memory_address_register <= instruction_register[7:0];
		execute_out:		memory_address_register <= program_counter;
		execute_addi:   	memory_address_register <= program_counter;
		execute_shl:		memory_address_register <= program_counter;
		execute_shr:		memory_address_register <= program_counter;
		default: 		memory_address_register <= program_counter;
	endcase
	case (state_from_decode_unit)
		execute_store: 	memory_write <= 1'b 1;
		default:	memory_write <= 1'b 0;
	endcase
 //mwrite <= memory_write;
		
	end

initial 
begin 
	memory[0]   = 16'h0208;
	memory[1]   = 16'h0009;
	memory[2]   = 16'h010a;
	memory[3]   = 16'h020a;
	memory[4]   = 16'h0009;
	memory[5]   = 16'h0108;
	memory[6]   = 16'h0000;
	memory[7]   = 16'h0000;
 	memory[8]   = 16'h0010;
	memory[9]   = 16'h0011;
	memory[10]  = 16'h0000;
	memory[11]  = 16'h1108;
	memory[12]  = 16'h0409;
	memory[13]  = 16'h050a;
	memory[14]  = 16'h060a;
	memory[15]  = 16'h0709;
	memory[16]  = 16'h0808;
	memory[17]  = 16'h0900;
	memory[18]  = 16'h0a00;
	memory[19]  = 16'h0b10;
	memory[20]  = 16'h0c08;
	memory[21]  = 16'h0d09;
	memory[22]  = 16'h0e0a;
	memory[23]  = 16'h0f0a;
	memory[24]  = 16'h0009;
	memory[25]  = 16'h0108;
	memory[26]  = 16'h0000;
	memory[27]  = 16'h0000;
	memory[28]  = 16'h0010;
	memory[29]  = 16'h0011;
	memory[30]  = 16'h0000;
	memory[31]  = 16'h0009;
	memory[32]  = 16'h010a;
	memory[33]  = 16'h020a;
	memory[34]  = 16'h0009;
	memory[35]  = 16'h0108;
	memory[36]  = 16'h0000;
	memory[37]  = 16'h0000;
	memory[38]  = 16'h0010;
	memory[39]  = 16'h0011;
	memory[40]  = 16'h0208;
	memory[41]  = 16'h0009;
	memory[42]  = 16'h010a;
	memory[43]  = 16'h020a;
	memory[44]  = 16'h0009;
	memory[45]  = 16'h0108;
	memory[46]  = 16'h0000;
	memory[47]  = 16'h0000;
	memory[48]  = 16'h0010;
	memory[49]  = 16'h0011;
	memory[50]  = 16'h0208;
	memory[51]  = 16'h0009;
	memory[52]  = 16'h010a;
	memory[53]  = 16'h020a;
	memory[54]  = 16'h0009;
	memory[55]  = 16'h0108;
	memory[56]  = 16'h0000;
	memory[57]  = 16'h0000;
	memory[58]  = 16'h0010;
	memory[59]  = 16'h0011;
	memory[60]  = 16'h0208;
	memory[61]  = 16'h0009;
	memory[62]  = 16'h010a;
	memory[63]  = 16'h020a;
	memory[64]  = 16'h0009;
	memory[65]  = 16'h0108;
	memory[66]  = 16'h0000;
	memory[67]  = 16'h0000;
	memory[68]  = 16'h0010;
	memory[69]  = 16'h0011;
	memory[70]  = 16'h0208;
	memory[71]  = 16'h0009;
	memory[72]  = 16'h010a;
	memory[73]  = 16'h020a;
	memory[74]  = 16'h0009;
	memory[75]  = 16'h0108;
	memory[76]  = 16'h0000;
	memory[77]  = 16'h0000;
	memory[78]  = 16'h0010;
	memory[79]  = 16'h0011;
	memory[80]  = 16'h0208;
	memory[81]  = 16'h0009;
	memory[82]  = 16'h010a;
	memory[83]  = 16'h020a;
	memory[84]  = 16'h0009;
	memory[85]  = 16'h0108;
	memory[86]  = 16'h0000;
	memory[87]  = 16'h0000;
	memory[88]  = 16'h0010;
	memory[89]  = 16'h0011;
	memory[90]  = 16'h0208;
	memory[91]  = 16'h0009;
	memory[92]  = 16'h010a;
	memory[93]  = 16'h020a;
	memory[94]  = 16'h0009;
	memory[95]  = 16'h0108;
	memory[96]  = 16'h0000;
	memory[97]  = 16'h0000;
	memory[98]  = 16'h0010;
	memory[99]  = 16'h0011;
	memory[100] =16'h0000;
	memory[101] = 16'h0009;
	memory[102] = 16'h010a;
	memory[103] = 16'h020a;
	memory[104] = 16'h0009;
	memory[105] = 16'h0108;
	memory[106] = 16'h0000;
	memory[107] = 16'h0000;
	memory[108] = 16'h0010;
	memory[109] = 16'h0011;
	memory[110] = 16'h0000;
	memory[111] = 16'h0208;
	memory[112] = 16'h0009;
	memory[113] = 16'h010a;
	memory[114] = 16'h020a;
	memory[115] = 16'h0009;
	memory[116] = 16'h0108;
	memory[117] = 16'h0000;
	memory[118] = 16'h0000;
	memory[119] = 16'h0010;
	memory[120] = 16'h0208;
	memory[121] = 16'h0009;
	memory[122] = 16'h010a;
	memory[123] = 16'h020a;
	memory[124] = 16'h0009;
	memory[125] = 16'h0108;
	memory[126] = 16'h0000;
	memory[127] = 16'h0000;
	memory[128] = 16'h0010;
	memory[129] = 16'h0011;
	memory[130] = 16'h0000;
	memory[131] = 16'h0009;
	memory[132] = 16'h010a;
	memory[133] = 16'h020a;
	memory[134] = 16'h0009;
	memory[135] = 16'h0108;
	memory[136] = 16'h0000;
	memory[137] = 16'h0000;
	memory[138] = 16'h0010;
	memory[139] = 16'h0011;
	memory[140] = 16'h0208;
	memory[141] = 16'h0009;
	memory[142] = 16'h010a;
	memory[143] = 16'h020a;
	memory[144] = 16'h0009;
	memory[145] = 16'h0108;
	memory[146] = 16'h0000;
	memory[147] = 16'h0000;
	memory[148] = 16'h0010;
	memory[149] = 16'h0011;
	memory[150] = 16'h0208;
	memory[151] = 16'h0009;
	memory[152] = 16'h010a;
	memory[153] = 16'h020a;
	memory[154] = 16'h0009;
	memory[155] = 16'h0108;
	memory[156] = 16'h0000;
	memory[157] = 16'h0000;
	memory[158] = 16'h0010;
	memory[159] = 16'h0011;
	memory[160] = 16'h0208;
	memory[161] = 16'h0009;
	memory[162] = 16'h010a;
	memory[163] = 16'h020a;
	memory[164] = 16'h0009;
	memory[165] = 16'h0108;
	memory[166] = 16'h0000;
	memory[167] = 16'h0000;
	memory[168] = 16'h0010;
	memory[169] = 16'h0011;
	memory[170] = 16'h0208;
	memory[171] = 16'h0009;
	memory[172] = 16'h010a;
	memory[173] = 16'h020a;
	memory[174] = 16'h0009;
	memory[175] = 16'h0108;
	memory[176] = 16'h0000;
	memory[177] = 16'h0000;
	memory[178] = 16'h0010;
	memory[179] = 16'h0011;
	memory[180] = 16'h0208;
	memory[181] = 16'h0009;
	memory[182] = 16'h010a;
	memory[183] = 16'h020a;
	memory[184] = 16'h0009;
	memory[185] = 16'h0108;
	memory[186] = 16'h0000;
	memory[187] = 16'h0000;
	memory[188] = 16'h0010;
	memory[189] = 16'h0011;
	memory[190] = 16'h0208;
	memory[191] = 16'h0009;
	memory[192] = 16'h010a;
	memory[193] = 16'h020a;	
	memory[194] = 16'h0009;
	memory[195] = 16'h0108;
	memory[196] = 16'h0000;
	memory[197] = 16'h0000;
	memory[198] = 16'h0010;
	memory[199] = 16'h0011;
	memory[200] = 16'h0208;
	memory[201] = 16'h0009;
	memory[202] = 16'h010a;
	memory[203] = 16'h020a;
	memory[204] = 16'h0009;
	memory[205] = 16'h0108;
	memory[206] = 16'h0000;
	memory[207] = 16'h0000;
	memory[208] = 16'h0010;
	memory[209] = 16'h0011;
	memory[210] = 16'h0000;
	memory[211] = 16'h0208;
	memory[212] = 16'h0009;
	memory[213] = 16'h010a;
	memory[214] = 16'h020a;
	memory[215] = 16'h0009;
	memory[216] = 16'h0108;
	memory[217] = 16'h0000;
	memory[218] = 16'h0000;
	memory[219] = 16'h0010;
	memory[220] = 16'h0208;
	memory[221] = 16'h0009;
	memory[222] = 16'h010a;
	memory[223] = 16'h020a;
	memory[224] = 16'h0009;
	memory[225] = 16'h0108;
	memory[226] = 16'h0000;
	memory[227] = 16'h0000;
	memory[228] = 16'h0010;
	memory[229] = 16'h0011;
	memory[230] = 16'h0000;
	memory[231] = 16'h0009;
	memory[232] = 16'h010a;
	memory[233] = 16'h020a;
	memory[234] = 16'h0009;
	memory[235] = 16'h0108;
	memory[236] = 16'h0000;
	memory[237] = 16'h0000;
	memory[238] = 16'h0010;
	memory[239] = 16'h0011;
	memory[240] = 16'h0208;
	memory[241] = 16'h0009;	
	memory[242] = 16'h010a;
	memory[243] = 16'h020a;
	memory[244] = 16'h0009;
	memory[245] = 16'h0108;
	memory[246] = 16'h0000;
	memory[247] = 16'h0000;
	memory[248] = 16'h0010;
	memory[249] = 16'h0011;
	memory[250] = 16'h0208;
	memory[251] = 16'h0009;
	memory[252] = 16'h010a;
	memory[253] = 16'h020a;
	memory[254] = 16'h0009;
	memory[255] = 16'h0000;
	program_counter = 0;
	clock = 1'b1;
	reset = 0;
forever
		begin
			#3 register_1 = register_A;
			#3 clock = ~clock;
			#1 register_1 = register_A;	
			#1 instruction_register = memory[program_counter];
			#3 program_counter=program_counter+1;		
			#1 state  = instruction_register[15:8];
			#1 memory_data_register = instruction_register[7:0];
		end
end

endmodule 