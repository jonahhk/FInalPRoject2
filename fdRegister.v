module fdRegister(clock, reset, enable, instruction, PC, insOut, pcOut); 
		input [31:0] instruction;
		input [11:0] PC;
		input clock, reset, enable; 
		output [31:0] insOut;
		output [11:0] pcOut;
		
		reg12 myPCRegister(clock, reset, enable, PC, pcOut);
		reg32 myInsRegister(clock, reset, enable, instruction, insOut); 

endmodule 

module deRegister(clock, reset, enable, instruction, PC, dataA, dataB, 
						insOut, pcOut, aOut, bOut);
		input [31:0] instruction, dataA, dataB;
		input [11:0] PC;
		input clock, reset, enable; 
		output [31:0] insOut, aOut, bOut;
		output [11:0] pcOut;
		
		reg12 myPCRegister(clock, reset, enable, PC, pcOut);
		reg32 myInsRegister(clock, reset, enable, instruction, insOut);
		reg32 myDataARegister(clock, reset, enable, dataA, aOut);
		reg32 myDataBRegister(clock, reset, enable, dataB, bOut);
		
endmodule 

module emRegister(clock, reset, enable, instruction, result, bReg, lt, eq, overflow, inPC,
						insOut, resultOut, ltOut, eqOut, mybOut, overflowOut, outPC);
		 input [31:0] instruction, result, bReg; 
		 input clock, reset, lt, eq, overflow, enable; 
		 input [11:0] inPC; 
		 output [31:0] insOut, resultOut, mybOut;
		 output [11:0] outPC; 
		 output ltOut, eqOut, overflowOut; 
		 
		 reg32 myInsRegister(clock, reset, enable, instruction, insOut);
		 reg32 myDataARegister(clock, reset, enable, result, resultOut);
		 reg32 myDataBister(clock, reset, enable, bReg, mybOut);
		 dflipflop ltResult(lt, clock, enable, reset, ltOut); 
		 dflipflop eqResult(eq, clock, enable, reset, eqOut); 
		 dflipflop myover(overflow, clock, enable, reset, overflowOut); 
		 reg12 oldPC(clock, reset, enable, inPC, outPC);
		
endmodule 

module mwRegister(clock, reset, enable, instruction, mResult, memOut, overflow, insOut, inPC,
					   wResult, wMemOut, overflowOut, outPC);
		 input [31:0] mResult, memOut, instruction;
		 input clock, reset, overflow, enable;
		 input [11:0] inPC; 
		 output [11:0] outPC;
		 output [31:0] wResult, wMemOut, insOut;
		 output overflowOut; 
		 
		 dflipflop myover(overflow, clock, enable, reset, overflowOut); 
		 reg32 myInsRegister(clock, reset, enable, instruction, insOut);
		 reg32 myResRegister(clock, reset, enable, mResult, wResult);
		 reg32 myMemsRegister(clock, reset, enable, memOut, wMemOut);
		 reg12 oldPC(clock, reset, enable, inPC, outPC);
		 
endmodule 
