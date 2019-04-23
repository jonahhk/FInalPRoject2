
module processor(
    // Control signals
    clock,                          // I: The master clock
    reset,                          // I: A reset signal

    // Imem
    address_imem,                   // O: The address of the data to get from imem
    q_imem,                         // I: The data from imem

    // Dmem
    address_dmem,                   // O: The address of the data to get or put from/to dmem
    data,                           // O: The data to write to dmem
    wren,                           // O: Write enable for dmem
    q_dmem,                         // I: The data from dmem

    // Regfile
    ctrl_writeEnable,               // O: Write enable for regfile
    ctrl_writeReg,                  // O: Register to write to in regfile
    ctrl_readRegA,                  // O: Register to read from port A of regfile
    ctrl_readRegB,                  // O: Register to read from port B of regfile
    data_writeReg,                  // O: Data to write to for regfile
    data_readRegA,                  // I: Data from port A of regfile
    data_readRegB,                   // I: Data from port B of regfile
	 
	 LightSwitch, LEDSwitch, FanSwitch, TimerSwitch, Humidity, Temp,
	 LightOut, LEDOut, FanOut, Timer, BinTimerOut, Ambient, KEY, SW, AUD_BCLK,
	 AUD_ADCLRCK, AUD_DACLRCK, AUD_XCK, AUD_DACDAT, address_sound, q_sound, 
	 Speak
);
    // Control signals
    input clock, reset, Ambient;

    // Imem
    output [11:0] address_imem;
    input [31:0] q_imem;

    // Dmem
    output [11:0] address_dmem;
    output [31:0] data;
    output wren;
    input [31:0] q_dmem;
    // Regfile
    output ctrl_writeEnable;
    output [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
    output [31:0] data_writeReg;
    input [31:0] data_readRegA, data_readRegB;
	 
	 //Input Signals 
	 input LightSwitch, LEDSwitch, FanSwitch, TimerSwitch, Speak; 
	 input [3:0] Timer; 
	 input [4:0] Temp; 
	 input [6:0] Humidity; 
	 
	 //Output Signals (To relays)
	 output LightOut, LEDOut, FanOut; 
	 
	 //Output Signal (Timer)
	 output [7:0] BinTimerOut; 
	 
	 //Audio In/Out
	 // Inputs
		input				SW;
		input 			KEY; 

		// Bidirectionals

		inout				AUD_BCLK;
		inout				AUD_ADCLRCK;
		inout				AUD_DACLRCK;

		// Outputs
		output				AUD_XCK;
		output				AUD_DACDAT;
		
		output [15:0] address_sound;
		output [31:0] q_sound;
		assign address_sound = myAdd;

//************************Stage One: Fetch Stage************************//
	 //First, start with the PC register. Let rest a clock cycle to instatiate 
	 // values in the registers. 
	 //Insert this value into the imem (as a 12-bit address)
	 //PC Register is then increased by one. 
	 //Feed this into a MUX
	 
	 wire [11:0] myPC, updPC, dPCOut, newPC;
	 wire [31:0] myInsn, dInsOut; 
	 wire PCOver, myWait; 
	 
	 
	 //TakeBranch and branchPC computed in Execute 
	 assign newPC = takeB ? bPC : updPC;
	 
	 //Currently: PC updates whenever there is no stalling in the decode stage 
			//Add support for multiplier stalling 
	 
	 reg12 myPCRegister(clock, reset, (!stall && !waitForMD), newPC, myPC); 
	 assign address_imem = myPC; 
	 CLAR12 incrementPC(myPC, 12'd1, 1'b0, PCOver, updPC);
	 
	 //Either choose noop or the instruction from memory; 
	 assign myInsn = takeB ? 32'd0 : q_imem;
	       /* FD Stage Register will take PC+1 and instruction */
	 fdRegister(clock, reset, (!stall), myInsn, updPC, dInsOut, dPCOut);
	 
//***********************Stage Two: Decode Stage************************//
	 
	 //ctrl_writeEnable,               // O: Write enable for regfile
    //ctrl_writeReg,                  // O: Register to write to in regfile
    //ctrl_readRegA,                  // O: Register to read from port A of regfile
    //ctrl_readRegB,                  // O: Register to read from port B of regfile
    //data_writeReg,                  // O: Data to write to for regfile
	 wire [31:0] dataRegA, dataRegB, eInsOut, aOut, bOut, mydInsn; 
	 wire [11:0] ePCOut;
	 wire [4:0] when0; 
	 wire stall; 
	 
	 //"A" register is always equal to $rs 
	 assign ctrl_readRegA = dInsOut[21:17]; 
	 
	 //Logic for address of "b" register
	 assign when0 = (dInsOut[28] || dInsOut[29]) ? dInsOut[26:22] : dInsOut[16:12]; 
	 assign ctrl_readRegB = dInsOut[31] ? 5'b11110 : when0;
	 
	 //Either propogate noOp or the Instruction(make sure to stop PC from moving)
	 //Should disable the adder previously (?)
	 pipelineHazard hazardDetect(dInsOut, eInsOut, stall);
	 assign mydInsn = (stall || takeB) ? 32'd0 : dInsOut;
	 
			/* DE Stage Register will take data from A/B and instruction */
	 deRegister(clock, reset, 1'b1, mydInsn, dPCOut, data_readRegA, data_readRegB, 
					eInsOut, ePCOut, aOut, bOut);
	 
	 wire[31:0] eaOut1; 
	 reg32 myDataa1(clock, reset, 1'b1, data_readRegA, eaOut1);
	 
//**********************Stage Three: Execute Stage**********************//
//assign eIns = eInsOut;
	 wire [31:0] data_operandA, data_operandB, data_result, mInsOut, resultOut, mybOut, myA, myB ; 
	 wire [31:0] nExtend, tExtend, mbOut, data_resultALU, data_resultMD; 
	 wire [31:0] myeInsn; 
	 wire [11:0] branchIMM, bPC, mPCOut; 
	 wire [4:0] ctrl_ALUopcode, ctrl_shiftamt, rORlw, bALU1; 
	 wire isNotEqual, isLessThan, overflow, ltOut, neqOut, takeBLT, takeBNE, takeB;
	 wire rStatusNotZero, takeBex, overflowOut, takeJ, takeJR, takeJAL, ctrl_MULT, ctrl_DIV; 
	 wire data_resultRDY, waitForMD, isNotEqualThrow;
	 wire [1:0] aaluSelect, baluSelect, aregValueSelect, bregValueSelect;
	 
	 assign ctrl_shiftamt = eInsOut[11:7]; 
	 
	 //Sign Extend N Immediate Field 
	 genvar i; 
		generate 
			for(i=31; i>16; i=i-1) begin: loop
				assign nExtend[i] = eInsOut[16];
			end 
		endgenerate 
	 assign nExtend[16:0]= eInsOut [16:0];
	 
	 
	 //Bypass logic for RS1 and RS2:
	 bypassALUA(eInsOut, mInsOut, wInsOut, aregValueSelect); 
	 mux_4 aluAInSelect(aregValueSelect, aOut, resultOut, data_writeReg, resultOut, myA);
	 
	 bypassALUB(eInsOut, mInsOut, wInsOut, bregValueSelect); 
	 mux_4 aluBInSelect(bregValueSelect, bOut, resultOut, data_writeReg, resultOut, myB);
	 
	 
	 //Logic for determining first input of ALU 
	 assign aaluSelect[1] = eInsOut[31] ? 1'b0 : eInsOut[29];
	 assign aaluSelect[0] = eInsOut[31] ? 1'b1 : eInsOut[27];
	 mux_4 aInSelect(aaluSelect, myA, 32'd0, myB, myA, data_operandA);
	 
//assign aValueSelect = aregValueSelect; 
//assign bValueSelect = bregValueSelect; 

	 //Logic for determining second input of ALU 
	 assign baluSelect[1] = (eInsOut[31] || ~(|eInsOut[30:29])) ? 1'b0 : eInsOut[29];
	 assign baluSelect[0] = (eInsOut[31] || ~(|eInsOut[30:29])) ? 1'b1 : eInsOut[27];
	 mux_4 bInSelect(baluSelect, nExtend, myB, myA, nExtend, data_operandB);
	 
	 
	 //Check if the value stored in the $rd register is not zero 
	 assign rStatusNotZero = (|aOut); 
	 
	 
//assign alua = data_operandA;
//assign alub = data_operandB; 
	 /*Logic for determining Opcode 
			-Case where bottom two bits of opcode is 00:
				If eInsOut[30]=1, op is a store and needs addition
				If =0, then use the ALUOPCODE
			-When bottom two is 01 or 11, do addition 
			-When bottom two is 10, do subtraction*/
	 assign rORlw = eInsOut[30] ? 5'd0 : eInsOut[6:2]; //lw or r-type ins. 
	 mux_4 opcodeSelect(eInsOut[28:27], rORlw, 5'd0, 5'b00001, 5'd0, ctrl_ALUopcode);
	 
	 alu mainALU(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt,
					data_resultALU, isNotEqualThrow, isLessThan, overflowALU);
	 
	 assign ctrl_MULT = ~(|eInsOut[31:30]) && (&eInsOut[29:28]) && ~eInsOut[27]; 
	 assign ctrl_DIV = ~(|eInsOut[31:30]) && (&eInsOut[29:27]); 
	 multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_resultMD, overflowMD, data_resultRDY);
	 
	 //Disable all pipeline registers whenever this is high 
/* MULTDIV not implemented */
	 //assign waitForMD = ((ctrl_MULT || ctrl_DIV) && !data_resultRDY); 
	 
	 assign overflow = (ctrl_MULT||ctrl_DIV) ? overflowMD : overflowALU; 
	 assign data_result = data_resultALU; //(ctrl_MULT||ctrl_DIV) ? data_resultMD : data_resultALU; 
		
	 //PC + 1 + Imm 
	 //Upper 5 bits of Immediate field ignored since PC always pos.
	 CLAR12 branchAdder(ePCOut, eInsOut[11:0], 1'b0, PCOver, branchIMM);

	 
	 //Figure out isNotEqual in parallel with ALU 
	 
	 NequalCompare32(data_operandA, data_operandB, isNotEqual); 
	 
	 //Branching and Jumping 
	 assign takeBNE = isNotEqual && ~(|eInsOut[31:29]) && eInsOut[28] && ~eInsOut[27]; 
	 assign takeBLT = isLessThan && ~(|eInsOut[31:30]) && (&eInsOut[29:28]) && ~eInsOut[27];
	 assign bPC = (takeBNE || takeBLT) ? branchIMM : 12'bz; 
	 assign takeBex = rStatusNotZero && eInsOut[31] && (~eInsOut[30]) && (&eInsOut[29:28]) && (~eInsOut[27]); 
	 assign bPC = takeBex ? eInsOut[11:0] : 12'bz;  
	 assign takeB = takeBNE || takeBLT || takeBex || takeJ || takeJR || takeJAL; 
	 
	 assign takeJ = ~(|eInsOut[31:28]) && eInsOut[27]; 
	 assign takeJR = ~(|eInsOut[31:30]) && eInsOut[29] && ~(|eInsOut[28:27]); 
	 assign takeJAL = ~(|eInsOut[31:29]) && (&eInsOut[28:27]); 
	 assign bPC = (takeJ || takeJAL) ? eInsOut[11:0] : 12'bz; 
	 assign bPC = takeJR ? myB[11:0] : 12'bz; 

	 assign myeInsn = takeB ? 32'd0 : eInsOut;
	 
	 emRegister(clock, reset, 1'b1, eInsOut, data_result, bOut, isLessThan, isNotEqual, overflow, ePCOut,
					mInsOut, resultOut, ltOut, neqOut, mbOut, overflowOut, mPCOut);
	 wire[31:0] maOut1; 
	 reg32 myDataa2(clock, reset, 1'b1, eaOut1, maOut1);		
//***********************Stage Four: Memory Stage***********************//
//assign mIns = mInsOut;
	 //Only write to memory when the operation is either a load or a store 
	 //address_dmem,                   // O: The address of the data to get or put from/to dmem
    //data,                           // O: The data to write to dmem
    //wren,                           // O: Write enable for dmem
    //q_dmem,                         // I: The data from dmem
	 wire[31:0] wInsOut, wResult, wMemOut, finalB; 
	 wire[11:0] wPCOut; 
	 wire isStore, isLoad, finalOver; 
	 
	 assign isStore = ~(|mInsOut[31:30]) && (&mInsOut[29:27]);
	 
	 assign address_dmem = resultOut[11:0]; 
	 assign data = mbOut; 
	 assign wren = isStore; 
	 
	 mwRegister(clock, reset, 1'b1, mInsOut, resultOut, q_dmem, overflowOut, wInsOut, mPCOut,
					wResult, wMemOut, finalOver, wPCOut);
	wire [31:0] finalA;
	 reg32 myDataB(clock, reset, 1'b1, maOut1, finalA);
//********************* Stage Five: Writeback Stage*********************//
//assign wIns = wInsOut;
	 wire [31:0] tSign; 
	 wire isjal, isRtype, isLw, isSetx, isAddi; 
	 wire [1:0] dataSelect, RDselect; 
	 
	 assign isjal   = ~(|wInsOut[31:29]) && (&wInsOut[28:27]); 
	 assign isRtype = ~(|wInsOut[31:27]); 
	 assign isLw    = ~(wInsOut[31]) && (wInsOut[30]) && ~(|wInsOut[29:27]);
	 assign isAddi = ~(|wInsOut[31:30]) && wInsOut[29] && ~wInsOut[28] && wInsOut[27];
	 assign isSetx  = ((wInsOut[31]) && ~(wInsOut[30]) && (wInsOut[29]) && 
						  ~(wInsOut[28]) && (wInsOut[27])); 
	
	 //Only write when the opCode is one of these 
	 assign ctrl_writeEnable = isjal || writeRtype || isLw || isSetx || isAddi;

	 //Logic for selecting destination address 
	 assign RDselect[0] = wInsOut[28]; //If asserted, $r31
	 assign RDselect[1] = wInsOut[31]; //If asserted, $r30
	 mux_4 FindRD(RDselect, wInsOut[26:22], 5'b11111, 5'b11110, wInsOut[26:22], 
			  ctrl_writeReg); 

	 //T Sign Extend 
	 assign tSign[31:27] = 5'd0; 
	 assign tSign[26:0] = wInsOut[26:0]; 
	 
	 //General logic for data to write 
	 wire normalRtype; 
	 assign normalRtype = !wInsOut[6] && !wInsOut[5];
	 
	 assign data_writeReg = isjal ?  wPCOut : 32'bz; 
	 assign data_writeReg = ((isRtype) && !finalOver && normalRtype) ? wResult : 32'bz; 
	 assign data_writeReg = (isAddi && !finalOver) ? wResult : 32'bz; 
	 assign data_writeReg = isSetx ? tSign : 32'bz; 
	 assign data_writeReg = isLw ? wMemOut : 32'bz;
	 
	 //Special Instructions: 
	 wire [31:0] myLight, myLED, myFan, myHumidity, myTemp, myTimer, myAmbient, mySpeak, digit; 

	 assign myLight[31:1] = 30'd0; 
	 assign myLight[0] = LightSwitch;
	 assign myLED[31:1] = 30'd0; 
	 assign myLED[0] = LEDSwitch; 
	 assign myFan[31:1] = 30'd0; 
	 assign myFan[0] = FanSwitch; 
	 assign myTimer[31:1] = 30'd0; 
	 assign myTimer[0] = TimerSwitch; 
	 assign myTemp[31:5] = 27'd0;
	 assign myTemp[4:0] = Temp; 
	 assign myHumidity[31:7] = 25'd0;
	 assign myHumidity[6:0] = Humidity; 
	 assign myAmbient[31:1] = 30'd0; 
	 assign myAmbient[0] = Ambient; 
	 assign mySpeak[31:1] = 30'd0; 
	 assign mySpeak[0] = Speak; 
	 assign digit[31:4] = 28'd0;
	 assign digit[3:0] = wInsOut[0] ? tens : ones;
	 wire lLight, lLED, lFan, lHum, lTemp, lTimer, lAmb, lSpeak, writeRtype, rDigit; 
	 assign lLight = (!wInsOut[6] && wInsOut[5] && !wInsOut[4] && !wInsOut[3] && !wInsOut[2]); 
	 assign lLED = (!wInsOut[6] && wInsOut[5] && !wInsOut[4] && !wInsOut[3] && wInsOut[2]); 
	 assign lFan = (!wInsOut[6] && wInsOut[5] && !wInsOut[4] && wInsOut[3] && !wInsOut[2]); 
	 assign lHum = (!wInsOut[6] && wInsOut[5] && !wInsOut[4] && wInsOut[3] && wInsOut[2]); 
	 assign lTemp = (!wInsOut[6] && wInsOut[5] && wInsOut[4] && !wInsOut[3] && !wInsOut[2]); 
	 assign lTimer = (!wInsOut[6] && wInsOut[5] && wInsOut[4] && !wInsOut[3] && wInsOut[2]); 
	 assign lAmb = (wInsOut[6] && !wInsOut[5] && wInsOut[4] && !wInsOut[3] && !wInsOut[2]);
	 assign lSpeak = (wInsOut[6] && !wInsOut[5] && wInsOut[4] && !wInsOut[3] && wInsOut[2]);
	 assign rDigit = (wInsOut[6] && wInsOut[5] && !wInsOut[4] && !wInsOut[3] && !wInsOut[2]);
	 
	 assign data_writeReg = (lLight && isRtype) ? myLight : 32'bz; 
	 assign data_writeReg = (lLED && isRtype) ? myLED : 32'bz; 
	 assign data_writeReg = (lFan && isRtype) ? myFan : 32'bz; 
	 assign data_writeReg = (lTimer && isRtype) ? myTimer : 32'bz; 
	 assign data_writeReg = (lTemp && isRtype) ? myTemp : 32'bz; 
	 assign data_writeReg = (lHum && isRtype) ? myHumidity : 32'bz; 
	 assign data_writeReg = (lAmb && isRtype) ? myAmbient : 32'bz;
	 assign data_writeReg = (lSpeak && isRtype) ? mySpeak : 32'bz;
	 assign data_writeReg = (rDigit && isRtype) ? digit : 32'bz;
	 
	 
	 assign writeRtype = (lLight || lLED || lFan || lHum || lTemp || lTimer || lAmb || lSpeak ||normalRtype); 
	 
	 //Overflow handling 
	 assign data_writeReg = (finalOver && isRtype && ~(|wInsOut[6:2])) ? 32'd1 : 32'bz; 
	 assign data_writeReg = (isAddi && finalOver) ? 32'd2 : 32'bz; 
	 assign data_writeReg = (finalOver && isRtype && ~(|wInsOut[6:3]) && wInsOut[2]) ? 32'd3 : 32'bz; 
	 assign data_writeReg = (finalOver && isRtype && 
								  ~(|wInsOut[6:5]) && (&wInsOut[4:3]) && ~wInsOut[2]) ? 32'd4 : 32'bz; 
	 assign data_writeReg = (finalOver && isRtype && ~(|wInsOut[6:5]) && (&wInsOut[4:2])) ? 32'd5 : 32'bz; 

	 //Special Commands: Setting registers
	 reg LightOut, FanOut, LEDOut, timerStart; 
	 initial LightOut = 1'b0; 
	 initial FanOut = 1'b0; 
	 initial LEDOut = 1'b0; 
	 initial timerStart = 1'b0; 
	 always @(posedge clock) begin 
		if(!(|wInsOut[31:27]) && wInsOut[6] &&!(|wInsOut[5:2])) begin
			LightOut = wInsOut[0]; 
		end 
		if(!(|wInsOut[31:27]) && wInsOut[6] &&!(|wInsOut[5:4]) && wInsOut[3] && !wInsOut[2]) begin
			FanOut = wInsOut[0]; 
		end 
		if(!(|wInsOut[31:27]) && wInsOut[6] &&!(|wInsOut[5:3]) && wInsOut[2]) begin
			LEDOut = wInsOut[0]; 
		end 
		if(!(|wInsOut[31:27]) && wInsOut[6] &&!(|wInsOut[5:4])&& wInsOut[3] && wInsOut[2]) begin 
			timerStart = wInsOut[0]; 
		end
	 end
	 
	 //Initialize and set the ROMS
	 wire setROMin, setROM;
	 assign setROMin = !(|wInsOut[31:27]) && wInsOut[6] && !(wInsOut[5]) && (&wInsOut[4:2]);
	 assign setROM = !(|wInsOut[31:27]) && wInsOut[6] && !(wInsOut[5]) && (&wInsOut[4:3]) && !wInsOut[2];
	 reg romChoose; 
	 reg [15:0] myAdd;
	 always @(posedge clock) begin
		if(setROM) begin
			romChoose = wInsOut[0];
		end
		if(setROMin) begin
			myAdd = finalA;
		end
	 end
	 
	 assign q_sound = romChoose ? q_sound2 : q_sound3;
	 wire [31:0] q_sound2, q_sound3;//, q_sound2;//, q_sound2; 
    OneToFiveAudio AllOnes(

        .address    (myAdd),            
        .clock      (clock),                 
        .q          (q_sound2)                  
    );
    SpeechATens SpeechandTens(

        .address    (myAdd),            
        .clock      (clock),                 
        .q          (q_sound3)                  
    );
    /*TensAudio my_Sound2(

        .address    (myAdd),            
        .clock      (CLOCK_8k),                 
        .q          (q_sound1)                  
    );*/

    /*SpeechAudio my_Sound3(

        .address    (myAdd),            
        .clock      (CLOCK_8k),                 
        .q          (q_sound0)                  
    );*/
		
	 wire CLOCK_8k; 
	 clock_divider fiftyTOeightyk(clock, CLOCK_8k);
	 
	 //Special Command: Start the Timer 
	 countDown(clock, Timer, timerStart, BinTimerOut);
	 
	 wire [3:0] hun, tens, ones;
	 wire[7:0] in; 
	 assign in = Temp; 
	 bit8toDec convert1(in, hun, tens, ones);
	 
	 
	 //assign timerStart = (!(|wInsOut[31:27]) && wInsOut[6] &&!(|wInsOut[5:4])&& wInsOut[3] && wInsOut[2]) ? wInsOut[0] : timerStart;
	 DE2_Audio_Example audioModule(
	 // Inputs
	 clock,
	 KEY, SW,

	 // Bidirectionals
	 AUD_BCLK,
	 AUD_ADCLRCK,
	 AUD_DACLRCK,

	 // Outputs
	 AUD_XCK,
	 AUD_DACDAT,
	 //address_sound,
	 q_sound
	 );
endmodule

//Counter Module: 
module countDown(clk, startTime, Nreset, timer); 
	input clk, Nreset; 
	input [3:0] startTime; //Max Timer; 80 seconds (16*5)
	output [7:0] timer; 

	wire [6:0] BinTimer; 
	wire reset;
	assign BinTimer = startTime*5; 
	assign reset = !Nreset; 

	reg [8:0] timer; 
	reg [25:0] accum = 0;
	wire pps = (accum == 0);
	always @(posedge clk) begin
    		accum <= (pps ? 50_000_000 : accum) - 1; //Should be 50_000_000
 		if (reset) begin
        	 timer = BinTimer; 
		end
		else if ((timer != 0) && pps) begin
			timer <= timer - 1; 
		end
	end
endmodule
