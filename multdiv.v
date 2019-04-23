module multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_resultRDY);
    input [31:0] data_operandA, data_operandB;
    input ctrl_MULT, ctrl_DIV, clock;

    output [63:0] data_result;
    output data_exception, data_resultRDY;
	 
	 wire [31:0] multResult, divResult;
	 wire ctrl, clrn, ctrl_write, divEx, multEx, divRDY, multRDY, prn, notMult; 
	 
	 
	 shiftReg64 multiply(data_operandA, data_operandB, clock, 1'b0, ctrl_MULT, multEx, multRDY, multResult); 
	 dividerCirc divideThis(data_operandA, data_operandB, clock, 1'b0, ctrl_DIV, divEx, divRDY, divResult); 
    
	 //Take mult(0) or div(1)
	 
	 assign clrn = ctrl_MULT; 
	 assign ctrl_write = ctrl_DIV; 
	 dflipflop multOrDiv(1'b1, clock, ctrl_write, clrn, ctrl); 
	 
	 assign data_result = ctrl ? divResult : multResult[31:0]; 
	 assign data_exception = ctrl ? divEx : multEx; 
	 assign data_resultRDY = ctrl ? divRDY : multRDY; 
endmodule

module dividerCirc(dividend, divisor, clk, clr, ctrl_div, overflow, dataReady, quotient);
	input clk, clr, ctrl_div;
	input [31:0] dividend, divisor; 
	
	output overflow, dataReady; 
	output [31:0] quotient;
	
	wire[63:0] pre, q;
	wire[31:0] upd, negDividend, negDivisor, negQ, cDividend, cDivisor, myDivisor, finalDiv, nDivisor; 
	wire prn, ctrl_write, ctrl_preload, sub, sub2, counterEnable, dataNext, negOut, overAdd, zdiv, del, del1, del2, waitbit; 
	wire[4:0] counter, nCounter; 
	
	dflipflop oneDFF(ctrl_div, clk, 1'b1, clr, waitbit); 
	
	//Never preset, always write, and load the values one ctrl_div starts 
	assign prn = 1'b0; 
	not delayCyce(ctrl_write, waitbit); 
	assign ctrl_preload = ctrl_div; 
	
	//If either Divisor or Dividend is negative, take the positive
	negate negDiv1(divisor, negDivisor);
	negate negDiv2(dividend, negDividend); 
	
	
	assign cDividend = dividend[31] ? negDividend : dividend; 
	assign cDivisor = divisor[31] ? negDivisor : divisor; 
	assign nDivisor = divisor[31] ? divisor : negDivisor;
	
	reg32 saveDivisor(clk, 1'b0, ctrl_div, cDivisor, myDivisor);
	
	assign finalDiv = cDivisor;
	
	xor outSign(negOut, dividend[31], divisor[31]); 
	
	subALU_lessEqual subvalues(q[63:32], finalDiv, nDivisor, upd, sub, overAdd);
	
	//dnegflipflop subSave(sub, clk, clrn, prn, ctrl_write, sub2); 
	
	//assign overflow = sub; 
	assign pre[0] = ctrl_preload ? cDividend[0] : sub;
	dflipflop firstDFF(pre[0], clk, ctrl_write, clr, q[0]); 
	
	genvar i; 
	generate 
		for(i=1; i<32; i=i+1) begin: loop1
			assign pre[i] = ctrl_preload ?  cDividend[i]: q[i-1];
			dflipflop singleDFFs1(pre[i], clk, ctrl_write, clr, q[i]); 
		end 
	endgenerate 
	
	assign pre[32] = ctrl_preload ? 1'b0 : q[31]; 
	dflipflop singleDFFs(pre[32], clk, ctrl_write, clr, q[32]); 
	
	generate 
		for(i=33 ; i<64 ; i=i+1) begin: loop2
			assign pre[i] = ctrl_preload ? 1'b0 : upd[i-33]; 
			dflipflop singleDFFs2(pre[i], clk, ctrl_write, clr, q[i]); 
		end
	endgenerate 
	
	//check when data is ready. 
	assign counterEnable = 1'b1; 
	
	not countn0(nCounter[0], counter[0]);
	not countn1(nCounter[1], counter[1]);
	not countn2(nCounter[2], counter[2]);
	not countn3(nCounter[3], counter[3]);
	not countn4(nCounter[4], counter[4]);
	
	myCounter readyCount(clk, ctrl_div, counterEnable, dataNext);
	//two cycle delay for counter 
	dflipflop DFFdelay(dataNext, clk, ctrl_write, clr, del); 
	dflipflop DFFdelay2(del, clk, ctrl_write, clr, del1); 
	dflipflop DFFdelay3(del1, clk, ctrl_write, clr, dataReady); 
	
	
	//pick correct output based on signs of input
	negate negOuput(q[31:0], negQ); 
	assign quotient = negOut ? negQ : q[31:0]; 
	
	//compute the overflow
	notZero z1(divisor, zdiv); 
	not finalOver(overflow, zdiv);
	
endmodule 

module subALU_lessEqual(data_operandA, data_operandB, dataNeg, data_result, subTaken, overflow);
	//A is the Remainder Block, B is the Divisor
   input [31:0] data_operandA, data_operandB, dataNeg;
   output [31:0] data_result;
	output overflow, subTaken; 
	
	wire ctrl_ALUopcode;
	
	wire [31:0] adderOut;
	wire notA, notB, notOut, lt0, lt1, lt2, lt3; 
	
	//Always subtract 
	assign ctrl_ALUopcode = 1'b0;
	

   CLAR32 adder0(data_operandA, dataNeg, ctrl_ALUopcode, overflow, adderOut); 
	
	//Logic for LessThan or Equal 
	not not1(notA, data_operandA[31]);
	not not2(notB, data_operandB[31]);
	not not3(notOut, adderOut[31]); 
	and and1(lt0, data_operandA[31], notB); //remainder neg, divisor pos. 
	and and2(lt1, data_operandA[31], data_operandB[31], adderOut[31]); 
	and and3(lt2, notA, notB, adderOut[31]);
	or or1(lt3, lt2, lt1, lt0);  //Asserts that the remainder is less than div. 
	
	//Only subtract if the remainder is not less than the div.
	assign data_result = lt3 ? data_operandA : adderOut;
	not takeSub(subTaken, lt3); 

endmodule

module notZero(in, out); //outputs a 1 if the number is not Zero
	input [32:0] in; 
	output out; 

   assign out = (in[32] || in[31] || in[30] || in[29] || in[28] || in[27] || in[26] ||
			in[25] || in[24] || in[23] || in[22] || in[21] || in[20] || in[19] || in[18] ||
			in[17] || in[16] || in[15] || in[14] || in[13] || in[12] || in[11] || in[10] || 
			in[9] || in[8] || in[7] || in[6] || in[5] || in[4] || in[3] || in[2] || in[1] || in[0]);
endmodule

module negate(in, out, overflow); 
	input [31:0] in; 
	output [31:0] out; 
	output overflow;
	
	wire throwOver;
	wire [31:0] negIn; 
	
	negMux2to1 negA1(in, 1'b1, negIn); 
	asALU myOut(negIn, 32'd1, 1'b0, 1'b1, out, overflow);

endmodule 

module mux21_32(a, b, sel, out);
	input [31:0] a, b; 
	input sel;
	output [31:0] out;
	
	assign out = sel ? b : a; 
	
endmodule 

module shiftReg64(multiplicand, multiplier, clk, clr, ctrl_mult, overflow, dataReady, quotient);
	input clk, clr, ctrl_mult;
	input [31:0] multiplier, multiplicand; 
	
	output overflow, dataReady; 
	output [31:0] quotient;
	
	wire prn, logicShift, ctrl_preload, ctrl_write, del;
	wire boothBit, Bini, addOrSub, adderEnable, counterEnable, cycle32, nOver, throwOver; 
	wire [31:0] upd, myMultiplicand, myMultiplier, saveMultiplicand, saveMultiplier; 
	wire[64:0] pre, q;
	wire[4:0] counter, nCounter; 
	
	//Latch onto values
	reg32 saveCand(clk, 1'b0, ctrl_mult, multiplicand, saveMultiplicand);
	reg32 savePlier(clk, 1'b0, ctrl_mult, multiplier, saveMultiplier);
	assign myMultiplier = ctrl_mult ? multiplier : saveMultiplier; 
	assign myMultiplicand = ctrl_mult ? multiplicand : saveMultiplicand; 
	
	//Always write to DFF and never preset value
	assign prn = 1'b0; 
	assign ctrl_write = 1'b1; 
	assign ctrl_preload = ctrl_mult; 
	
	//Set up of Multiplier Circuit 
	asALU addvalues(q[63:32], myMultiplicand, addOrSub, adderEnable, upd, throwOver);
	
	assign logicShift = ctrl_preload ? 1'b0 : upd[31];
	dflipflop singleDFF(logicShift, clk, ctrl_write, clr, q[63]); 
	
	genvar i; 
	generate 
		for(i=62; i>31; i=i-1) begin: loop1
			assign pre[i] = ctrl_preload ? 1'b0 : upd[i-31];
			dflipflop singleDFFs(pre[i], clk, ctrl_write, clr, q[i]); 
		end 
	endgenerate 
	
	assign pre[31] = ctrl_preload ? multiplier[31] : upd[0];  
	dflipflop singleDFF2(pre[31], clk, ctrl_write, clr, q[31]);
	
	generate 
		for(i=30 ; i>=0 ; i=i-1) begin: loop2
			assign pre[i] = ctrl_preload ? multiplier[i] : q[i+1]; 
			dflipflop singleDFF2s(pre[i], clk, ctrl_write, clr, q[i]);
		end
	endgenerate 
	
	assign Bini = ctrl_preload ? 1'b0 : q[0]; 
	dflipflop prev(Bini, clk, ctrl_write, clr, boothBit);
	
	//logic for Booth's Algorithm
	assign addOrSub = boothBit ? 1'b0 : 1'b1;
	xor validAdd(adderEnable, boothBit, q[0]);
	
	//check for overflow
	
	overflowCheck over(myMultiplier, myMultiplicand, quotient, overflow); 
	
	//Counter for checking when the data is ready
	assign counterEnable = 1'b1; 
	
	not countn0(nCounter[0], counter[0]);
	not countn1(nCounter[1], counter[1]);
	not countn2(nCounter[2], counter[2]);
	not countn3(nCounter[3], counter[3]);
	not countn4(nCounter[4], counter[4]);
	
	myCounter readyCount(clk, ctrl_mult, counterEnable, del);
	dflipflop DFFdelay2(del, clk, ctrl_write, clr, dataReady); 
	assign quotient = q[31:0]; 
	
endmodule 

module overflowCheck(a, b, multOut, out);
	input [31:0] a, b, multOut; 
	output out; 
	wire [31:0] myB, myA, adderOutA, adderOutB;
	wire correctSign; 

	xor inputs(correctSign, a[31], b[31]); 
	xor final0(out, correctSign, multOut[31]);

endmodule

module lessThan(a, b, out); 
	input [31:0] a, b;
	output out;
	
endmodule 

module myCounter(clock, clr, enable, ready);
	input clock, clr, enable; 
	//output [4:0] out; 
	output ready;
	
	wire [32:0] q; 
	wire [4:0] out; 
	
	
	dflipflop singleDFF(clr, clock, enable, 1'b0, q[0]); 
	genvar i; 
	generate 
		for(i=1; i<32; i=i+1) begin: loop1
			dflipflop singleDFF(q[i-1], clock, enable, clr, q[i]); 
		end 
	endgenerate 
	
	assign ready = q[31]; 

endmodule 

module asALU(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_enable, data_result, overflow);

   input [31:0] data_operandA, data_operandB;
   input ctrl_ALUopcode, ctrl_enable; //opcode 1 for sub, 0 for add 
   output [31:0] data_result;
	output overflow; 
	
	wire [31:0] myB, adderOut;
	//assign constant = 32'd0; 
	
	negMux2to1 bSelect(data_operandB, ctrl_ALUopcode, myB); 

   CLAR32 adder0(data_operandA, myB, ctrl_ALUopcode, overflow, adderOut);

	assign data_result = ctrl_enable ? adderOut : data_operandA; 

endmodule

module mux2to1(a, b, sel, out);
	input [31:0] a, b; 
	input sel; 
	output [31:0] out; 
	
	assign out = sel ? b : a;
endmodule 

