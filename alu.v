module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);

   input [31:0] data_operandA, data_operandB;
   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
	
	//input[7:0] data_operandA, data_operandB;

   output [31:0] data_result;
	//output[7:0] data_result; 
   output isNotEqual, isLessThan, overflow;
	
	wire [31:0] myB, orOut, andOut, sumOut, raOut, llOut, constant; 
	wire lt0, lt1, lt2,  nA, nB; 
	assign constant = 32'b0; 
	
	and32 myAnd(data_operandA, data_operandB, andOut);
	or32 myOr(data_operandA, data_operandB, orOut);
	
	negMux2to1 bSelect(data_operandB, ctrl_ALUopcode[0], myB); 

   CLAR32 adder0(data_operandA, myB, ctrl_ALUopcode[0], overflow, sumOut); 
	shiftLL myShift(data_operandA, ctrl_shiftamt, llOut);
	shiftRA myShift2(data_operandA, ctrl_shiftamt, raOut);
	
	mux_8(ctrl_ALUopcode[2:0], sumOut, sumOut, andOut, orOut, llOut, raOut, constant, constant, data_result); 
	
	//Logic for LessThan 
	not not0(nA, data_operandA[31]); 
	not not1(nB, data_operandB[31]);
	
	and and1(lt0, data_operandA[31], nB); //remainder neg, divisor pos. 
	and and2(lt1, data_operandA[31], data_operandB[31], sumOut[31]); 
	and and3(lt2, nA, nB, sumOut[31]);
	or or1(isLessThan, lt2, lt1, lt0);
	
	//Basic logic for Equals 
	isZero checkEqual(sumOut, isNotEqual); 

endmodule

module isZero(in, out);
	input [31:0] in; 
	output out; 
	
	or or1(out, in[31], in[30], in[29], in[28], in[27], in[26],
				in[25], in[24], in[23], in[22], in[21], in[20], in[19], in[18],
				in[17], in[16], in[15], in[14], in[13], in[12], in[11], in[10], 
				in[9], in[8], in[7], in[6], in[5], in[4], in[3], in[2], in[1], in[0]);
endmodule 

module or32(in_A, in_B, out);
		input [31:0] in_A, in_B;
		output [31:0] out; 
		genvar i; 
		generate 
			for(i=0; i<32; i=i+1) begin: loop
				or or0(out[i], in_A[i], in_B[i]);
			end 
		endgenerate 
	
endmodule 

module and32(in_A, in_B, out);
		input [31:0] in_A, in_B;
		output [31:0] out; 
		genvar i; 
		generate 
			for(i=0; i<32; i=i+1) begin: loop
				and and0(out[i], in_A[i], in_B[i]);
			end 
		endgenerate 
	
endmodule
	
module shiftLL(in, shamt, out);
	input [31:0] in;
	input [4:0] shamt; 
	output [31:0] out;
	wire [31:0] result1, result2, result3, result4, shift16, shift8;
	wire [31:0] shift4, shift2, shift1; 
	
	assign result1 = shamt[4] ? shift16 : in;
	assign result2 = shamt[3] ? shift8 : result1;
	assign result3 = shamt[2] ? shift4 : result2;
	assign result4 = shamt[1] ? shift2 : result3;
	assign out = shamt[0] ? shift1 : result4;
	
	assign shift16[15:0] = 16'd0; 
	assign shift8[7:0] = 8'd0; 
	assign shift4[3:0] = 4'd0; 
	assign shift2[1:0] = 2'd0; 
	assign shift1[0] = 1'b0; 
	
	genvar i; 
		generate 
			for(i=0; i<16; i=i+1) begin: loop1
				assign shift16[i+16] = in[i];
	
			end 
		endgenerate 
		generate 
			for(i=0; i<24; i=i+1) begin: loop2
				assign shift8[i+8] = result1[i]; 
	
			end 
		endgenerate 
		generate 
			for(i=0; i<28; i=i+1) begin: loop3
				assign shift4[i+4] = result2[i]; 
	
			end 
		endgenerate 
		generate 
			for(i=0; i<30; i=i+1) begin: loop4
				assign shift2[i+2] = result3[i]; 
	
			end 
		endgenerate 
		generate 
			for(i=0; i<31; i=i+1) begin: loop5
				assign shift1[i+1] = result4[i]; 
	
			end 
		endgenerate 
endmodule 

module shiftRA(in, shamt, out);
	input [31:0] in;
	input [4:0] shamt; 
	output [31:0] out;
	wire [31:0] result1, result2, result3, result4, shift16, shift8;
	wire [31:0] shift4, shift2, shift1; 
	
	assign result1 = shamt[4] ? shift16 : in;
	assign result2 = shamt[3] ? shift8 : result1;
	assign result3 = shamt[2] ? shift4 : result2;
	assign result4 = shamt[1] ? shift2 : result3;
	assign out = shamt[0] ? shift1 : result4;
	
	assign shift8[31:24] = in[31] ? 8'b11111111 : 8'd0; 
	assign shift4[31:28] = in[31] ? 4'b1111 : 4'd0; 
	assign shift2[31:30] = in[31] ? 2'b11 : 2'd0; 
	assign shift1[31] = in[31]; 
	
	genvar i; 
		generate 
			for(i=31; i>15; i=i-1) begin: loop1
				assign shift16[i] = 1'b1; 
				assign shift16[i-16] = in[i]; 
	
			end 
		endgenerate 
		generate 
			for(i=31; i>7; i=i-1) begin: loop2
				assign shift8[i-8] = result1[i]; 
	
			end 
		endgenerate 
		generate 
			for(i=31; i>3; i=i-1) begin: loop3
				assign shift4[i-4] = result2[i]; 
	
			end 
		endgenerate 
		generate 
			for(i=31; i>1; i=i-1) begin: loop4
				assign shift2[i-2] = result3[i]; 
	
			end 
		endgenerate 
		generate 
			for(i=31; i>0; i=i-1) begin: loop5
				assign shift1[i-1] = result4[i]; 
	
			end 
		endgenerate 
	
endmodule 

module CLAR32(a, b, carryIn, overflow, out);
		input [31:0] a, b;
		input carryIn;
		output overflow; 
		output [31:0] out;
		wire [3:1] c;
		wire [3:0] g, p; 
		wire myc1;
		wire [1:0] myc2;
		wire [2:0] myc3, over; //over is unused; only care about overflow of last//
		wire [3:0] myc4; 
	
		CLA8Block claOne(a[7:0], b[7:0], carryIn, g[0], p[0], over[0], out[7:0]); 
		CLA8Block claTwo(a[15:8], b[15:8], c[1], g[1], p[1], over[1], out[15:8]); 
		CLA8Block claThree(a[23:16], b[23:16], c[2], g[2], p[2], over[2], out[23:16]); 
		CLA8Block claFour(a[31:24], b[31:24], c[3], g[3], p[3], overflow, out[31:24]); 
	
		and and0(myc1, p[0], carryIn); 
		or or0(c[1], g[0], myc1);
	
		and and11(myc2[0], p[1], g[0]);
		and and12(myc2[1], p[1], p[0], carryIn);
		or or1(c[2], g[1], myc2[1], myc2[0]); 
	
		and and21(myc3[0], p[2], p[1], g[0]);
		and and22(myc3[1], p[2], p[1], p[0], carryIn);
		and and23(myc3[2], p[2], g[1]);
		or or2(c[3], g[2], myc3[2], myc3[1], myc3[0]); 
	
		and and31(myc4[0], p[3], p[2], p[1], g[0]);
		and and32(myc4[1], p[3], p[2], p[1], p[0], carryIn);
		and and33(myc4[2], p[3], p[2], g[1]);
		and and34(myc4[3], p[3], g[2]);
		or or3(carryOut, g[3], myc4[3], myc4[2], myc4[1], myc4[0]); 
	
endmodule 
		
module CLA8Block(data_a, data_b, carryIn, carryOut, pOut, over, out);
		input [7:0] data_a, data_b;
		input carryIn;
		output carryOut, pOut, over; 
		output [7:0] out;
		wire [7:0] g, p; 
		wire [7:1] c;
		wire[7:0] myc8;
		wire[6:0] myc7;
		wire[5:0] myc6;
		wire[4:0] myc5;
		wire[3:0] myc4;
		wire[2:0] myc3;
		wire[1:0] myc2;
		wire myc1; 
	
		and gFunc(g[0], data_a[0], data_b[0]);
		or pFunc(p[0], data_a[0], data_b[0]);
		xor s0(out[0], carryIn, data_a[0], data_b[0]);
		genvar i; 
			generate 
				for(i=1; i<8; i=i+1) begin: loop
					and gFunc(g[i], data_a[i], data_b[i]);
					or pFunc(p[i], data_a[i], data_b[i]);
					xor s0(out[i], c[i], data_a[i], data_b[i]);
				end 
			endgenerate 
	
		and and0(myc1, p[0], carryIn); 
		or or0(c[1], g[0], myc1);
	
		and and11(myc2[0], p[1], g[0]);
		and and12(myc2[1], p[1], p[0], carryIn);
		or or1(c[2], g[1], myc2[1], myc2[0]); 
	
		and and21(myc3[0], p[2], p[1], g[0]);
		and and22(myc3[1], p[2], p[1], p[0], carryIn);
		and and23(myc3[2], p[2], g[1]);
		or or2(c[3], g[2], myc3[2], myc3[1], myc3[0]); 
	
		and and31(myc4[0], p[3], p[2], p[1], g[0]);
		and and32(myc4[1], p[3], p[2], p[1], p[0], carryIn);
		and and33(myc4[2], p[3], p[2], g[1]);
		and and34(myc4[3], p[3], g[2]);
		or or3(c[4], g[3], myc4[3], myc4[2], myc4[1], myc4[0]); 
	
		and and41(myc5[0], p[4], p[3], p[2], p[1], g[0]);
		and and42(myc5[1], p[4], p[3], p[2], p[1], p[0], carryIn);
		and and43(myc5[2], p[4], p[3], p[2], g[1]);
		and and44(myc5[3], p[4], p[3], g[2]);
		and and45(myc5[4], p[4], g[3]);
		or or4(c[5], g[4], myc5[4], myc5[3], myc5[2], myc5[1], myc5[0]); 
	
		and and51(myc6[0], p[5], p[4], p[3], p[2], p[1], g[0]);
		and and52(myc6[1], p[5], p[4], p[3], p[2], p[1], p[0], carryIn);
		and and53(myc6[2], p[5], p[4], p[3], p[2], g[1]);
		and and54(myc6[3], p[5], p[4], p[3], g[2]);
		and and55(myc6[4], p[5], p[4], g[3]);
		and and56(myc6[5], p[5], g[4]);
		or or5(c[6], g[5], myc6[5], myc6[4], myc6[3], myc6[2], myc6[1], myc6[0]); 
	
		and and61(myc7[0], p[6], p[5], p[4], p[3], p[2], p[1], g[0]);
		and and62(myc7[1], p[6], p[5], p[4], p[3], p[2], p[1], p[0], carryIn);
		and and63(myc7[2], p[6], p[5], p[4], p[3], p[2], g[1]);
		and and64(myc7[3], p[6], p[5], p[4], p[3], g[2]);
		and and65(myc7[4], p[6], p[5], p[4], g[3]);
		and and66(myc7[5], p[6], p[5], g[4]);
		and and67(myc7[6], p[6], g[5]);
		or or6(c[7], g[6], myc7[6], myc7[5], myc7[4], myc7[3], myc7[2], myc7[1], myc7[0]); 
	
		and and71(myc8[0], p[7], p[6], p[5], p[4], p[3], p[2], p[1], g[0]);
		and and72(myc8[1], p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0], carryIn);
		and and73(myc8[2], p[7], p[6], p[5], p[4], p[3], p[2], g[1]);
		and and74(myc8[3], p[7], p[6], p[5], p[4], p[3], g[2]);
		and and75(myc8[4], p[7], p[6], p[5], p[4], g[3]);
		and and76(myc8[5], p[7], p[6], p[5], g[4]);
		and and77(myc8[6], p[7], p[6], g[5]);
		and and78(myc8[7], p[7], g[6]); 
		or or7(carryOut, g[7], myc8[7], myc8[6], myc8[5], myc8[4], myc8[3], myc8[2], myc8[1], myc8[0]); 
	
		xor overflow(over, carryOut, c[7]);
		assign pOut = myc8[0]; 
endmodule

module CLAR12(a, b, carryIn, overflow, out);
		input [11:0] a, b;
		input carryIn;
		output overflow; 
		output [11:0] out;
		wire [1:0] c;
		wire [1:0] g, p; 
		wire myc1, over; 

		CLA8Block claOne(a[7:0], b[7:0], carryIn, g[0], p[0], over, out[7:0]); 
		CLA4Block claTwo(a[11:8], b[11:8], c[1], g[1], p[1], overflow, out[11:8]);

		and and0(myc1, p[0], carryIn); 
		or or0(c[1], g[0], myc1);

endmodule 

module CLA4Block(data_a, data_b, carryIn, carryOut, pOut, over, out);
		input [3:0] data_a, data_b;
		input carryIn;
		output carryOut, pOut, over; 
		output [3:0] out;
		wire [3:0] g, p; 
		wire [3:1] c;
		wire[3:0] myc4;
		wire[2:0] myc3;
		wire[1:0] myc2;
		wire myc1; 
	
		and gFunc(g[0], data_a[0], data_b[0]);
		or pFunc(p[0], data_a[0], data_b[0]);
		xor s0(out[0], carryIn, data_a[0], data_b[0]);
		genvar i; 
			generate 
				for(i=1; i<4; i=i+1) begin: loop
					and gFunc(g[i], data_a[i], data_b[i]);
					or pFunc(p[i], data_a[i], data_b[i]);
					xor s0(out[i], c[i], data_a[i], data_b[i]);
				end 
			endgenerate 
	
		and and0(myc1, p[0], carryIn); 
		or or0(c[1], g[0], myc1);
	
		and and11(myc2[0], p[1], g[0]);
		and and12(myc2[1], p[1], p[0], carryIn);
		or or1(c[2], g[1], myc2[1], myc2[0]); 
	
		and and21(myc3[0], p[2], p[1], g[0]);
		and and22(myc3[1], p[2], p[1], p[0], carryIn);
		and and23(myc3[2], p[2], g[1]);
		or or2(c[3], g[2], myc3[2], myc3[1], myc3[0]); 
	
		and and31(myc4[0], p[3], p[2], p[1], g[0]);
		and and32(myc4[1], p[3], p[2], p[1], p[0], carryIn);
		and and33(myc4[2], p[3], p[2], g[1]);
		and and34(myc4[3], p[3], g[2]);
		or or3(carryOut, g[3], myc4[3], myc4[2], myc4[1], myc4[0]); 
	
	
		xor overflow(over, carryOut, c[3]);
		assign pOut = myc4[0]; 
endmodule
