module skeleton(CLOCK_50, reset, PC, idata, writeReg, ctrl_writeEnable, ctrl_writeReg,
					LightSwitch, LEDSwitch, FanSwitch, TimerSwitch, Humidity, Temp, Ambient,
					LightOut, LEDOut, FanOut, TempSeg1, TempSeg0, HumSeg1, HumSeg0,
					TimSeg2, TimSeg1, TimSeg0, Timer, BinTimerOut, TimSegLow, KEY, SW, AUD_BCLK,
	            AUD_ADCLRCK, AUD_DACLRCK, AUD_XCK, AUD_DACDAT, address_sound, q_sound, Speak,
					heaterin, heaterout);
    input CLOCK_50, reset;
	 output [31:0] writeReg; 
	 output [11:0] PC; 
	 output [31:0] idata;
	 output [4:0] ctrl_writeReg;
	 output ctrl_writeEnable;
	 
	 input LightSwitch, LEDSwitch, FanSwitch, TimerSwitch, Speak, heaterin; 
	 input Ambient; 
	 input [6:0] Humidity;
	 input [4:0] Temp; 
	 output LightOut, LEDOut, FanOut, heaterout; 
	 output [6:0] TempSeg1, TempSeg0, HumSeg1, HumSeg0, TimSeg2, TimSeg1, TimSeg0, TimSegLow; 
	 
	 input[3:0] Timer; 
	 output[7:0] BinTimerOut;
	 
	 assign PC = address_imem;
	 assign idata = q_imem; 
	 assign writeReg = data_writeReg;
	 
	 assign heaterout = heaterin; 
	 
	 //assign LightOut = Ambient; 
	 wire clock; 
	 assign clock = CLOCK_50;
	 
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
	 
	 wire [6:0] TimSeg2a, TimSeg1a, TimSeg0a; 
	 
	 assign TimSegLow = 7'b1111111; 
	 
	 /*Temperature and Humidity Display*/
	 temp2Segment(Temp, TempSeg1, TempSeg0); 
	 hum2Segment(Humidity, HumSeg1, HumSeg0); 
	 timer2Segment(BinTimerOut, TimSeg2a, TimSeg1a, TimSeg0a);
	 
	 reg [31:0] counter; 
	 reg flip =1'b1; 
	 always @(posedge CLOCK_50) begin
			if (BinTimerOut == 8'd0) begin
				counter = counter + 1; 
			end
			if(counter >= 32'd50000000) begin  //32'd5000000
				flip = ~flip; 
				counter = 0;
			end
			if(BinTimerOut > 8'd0) begin
				flip = 1'b1;
			end
	 end
	 assign TimSeg2 = flip ? TimSeg2a : 7'b1111111;
	 assign TimSeg1 = flip ? TimSeg1a : 7'b1111111;
	 assign TimSeg0 = flip ? TimSeg0a : 7'b1111111;
	 
	 
	 /*Timer Display*/
	 
	 
    /** IMEM **/
    // Figure out how to generate a Quartus syncram component and commit the generated verilog file.
    // Make sure you configure it correctly!
    wire [11:0] address_imem;
    wire [31:0] q_imem;
    imem my_imem(
        .address    (address_imem),            // address of data
        .clock      (~clock),                  // you may need to invert the clock
        .q          (q_imem)                   // the raw instruction
    );

    /** DMEM **/
    // Figure out how to generate a Quartus syncram component and commit the generated verilog file.
    // Make sure you configure it correctly!
    wire [11:0] address_dmem;
    wire [31:0] data;
    wire wren;
    wire [31:0] q_dmem;
    dmem my_dmem(
        .address    (address_dmem),      // address of data
        .clock      (~clock),            // may need to invert the clock
        .data	    (data),               // data you want to write
        .wren	    (wren),               // write enable
        .q         (q_dmem)              // data from dmem
    );

    /** REGFILE **/
    // Instantiate your regfile
    wire ctrl_writeEnable;
    wire [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
    wire [31:0] data_writeReg;
    wire [31:0] data_readRegA, data_readRegB;
    regfile my_regfile(
        clock,
        ctrl_writeEnable,
        reset,
        ctrl_writeReg,
        ctrl_readRegA,
        ctrl_readRegB,
        data_writeReg,
        data_readRegA,
        data_readRegB
    );

    /** PROCESSOR **/
    processor my_processor(
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
		  LightOut, LEDOut1, FanOut, Timer, BinTimerOut, Ambient, KEY, SW, AUD_BCLK,
		  AUD_ADCLRCK, AUD_DACLRCK, AUD_XCK, AUD_DACDAT, address_sound, q_sound, 
		  Speak
    );

endmodule


module temp2Segment(temp, seg1, seg0); 
	input [4:0] temp; 
	output [6:0] seg0, seg1; 
	
	wire[7:0] in; 
	wire[3:0] tens, ones; 
	assign in[7:5] = 3'd0; 
	assign in[4:0] = temp; 
	wire [3:0] hun; 
	
	bit8toDec convert1(in, hun, tens, ones);
	Hexadecimal_To_Seven_Segment(tens, seg1);
	Hexadecimal_To_Seven_Segment(ones, seg0);
	
endmodule 

module hum2Segment(humidity, seg1, seg0); 
	input [7:0] humidity; 
	output [6:0] seg0, seg1; 
	
	wire[7:0] in; 
	wire[3:0] tens, ones; 
	assign in= humidity; 
	//assign in[0] = 1'd0; 
	wire [3:0] hun; 
	bit8toDec convert1(in, hun, tens, ones);
	Hexadecimal_To_Seven_Segment(tens, seg1);
	Hexadecimal_To_Seven_Segment(ones, seg0);

endmodule 

module timer2Segment(BinTimerOut, seg2, seg1, seg0); 
	input [7:0] BinTimerOut; 
	output [6:0] seg0, seg1, seg2; 
	
	wire[7:0] in; 
	wire[3:0] tens, ones; 
	assign in= BinTimerOut; 
	//assign in[0] = 1'd0; 
	wire [3:0] hun;
	bit8toDec convert1(in, hun, tens, ones);
	Hexadecimal_To_Seven_Segment(tens, seg1);
	Hexadecimal_To_Seven_Segment(ones, seg0);
	Hexadecimal_To_Seven_Segment(hun, seg2);

endmodule 

module bit8toDec(in, hundreds, tens, ones);
	input [7:0] in; 
	output reg [3:0] tens, ones; 
	output reg [3:0] hundreds; 
	
	integer i;
	always @(in) begin
		hundreds = 4'd0;
		tens = 4'd0; 
		ones = 4'd0; 
		
		for(i=7; i>=0; i=i-1) begin
			if (hundreds >= 5)
				hundreds = hundreds + 3; 
			if (tens >= 5)
				tens = tens +3; 
			if (ones >= 5)
				ones = ones +3; 
				
			hundreds = hundreds << 1; 
			hundreds[0] = tens[3];
			tens = tens << 1; 
			tens[0] = ones[3];
			ones = ones << 1; 
			ones[0] = in[i];
		end
	end
	
endmodule
