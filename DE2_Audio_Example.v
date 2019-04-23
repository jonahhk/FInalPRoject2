
module DE2_Audio_Example (
	// Inputs
	CLOCK_50,
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


/*****************************************************************************

 *                           Parameter Declarations                          *

 *****************************************************************************/

/*****************************************************************************

 *                             Port Declarations                             *

 *****************************************************************************/
// Inputs

input				CLOCK_50;
input				SW;
input 			KEY; 
assign AUD_ADCDAT = 1'b0;

// Bidirectionals

inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

/*****************************************************************************

 *                 Internal Wires and Registers Declarations                 *

 *****************************************************************************/
// Internal Wires

wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

// Internal Registers

// State Machine Registers

/*****************************************************************************

 *                         Finite State Machine(s)                           *

 *****************************************************************************/

 

 /*****************************************************************************

 *                         SOUNDS GO HERE                               *

 *****************************************************************************/
 // [15:0] address_sound;
 input [31:0] q_sound;
 wire CLOCK_8k; 
	
 /*wire q_sound1, q_sound2, q_sound3; 
 OneToFiveAudio my_Sound(

        .address    (myAdd),            
        .clock      (CLOCK_8k),                 
        .q          (q_sound1)                  
    );
SixtoNine my_Sound1(

        .address    (myAdd),            
        .clock      (CLOCK_8k),                 
        .q          (q_sound)                  
    );
TensAudio my_Sound2(

        .address    (myAdd),            
        .clock      (CLOCK_8k),                 
        .q          (q_sound2)                  
    );

SpeechAudio my_Sound3(

        .address    (myAdd),            
        .clock      (CLOCK_8k),                 
        .q          (q_sound3)                  
    );

*/
	 

 clock_divider fiftyTOeightyk(CLOCK_50, CLOCK_8k);
 /*reg [16:0] myAdd;
 always @(posedge CLOCK_8k) begin
		if(SW) begin
			myAdd = myAdd + 1;
		end
		//if(myAdd > 14'h9b18 || SW[3]==0) begin 
			//myAdd = 14'd0; 
		//end
 end
 //assign address_sound = myAdd; 
 */
 assign right_out = q_sound;
 assign left_out = right_out; 

 /*****************************************************************************

 *                         SOUNDS END HERE                              *

 *****************************************************************************/
assign read_audio_in			= audio_in_available & audio_out_allowed;

wire [31:0] left_in, right_in, left_out, right_out;
assign left_in = left_channel_audio_in;
assign right_in = right_channel_audio_in;

assign left_channel_audio_out	= left_out;
assign right_channel_audio_out	= right_out;
assign write_audio_out			= audio_in_available & audio_out_allowed;

/*****************************************************************************

 *                              Internal Modules                             *

 *****************************************************************************/
Audio_Controller Audio_Controller (

	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						   (~KEY),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),

	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),
	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals

	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),

	// Outputs

	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),
	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);




endmodule



module clock_divider(clock_in, clock_out);
	input clock_in; // input clock on FPGA
	output clock_out; // output clock after dividing the input clock by divisor
	reg[27:0] counter=28'd0;
	parameter DIVISOR = 28'd6250;//d10000;//d6250;

	//parameter DIVISOR = 28'd100; 

	// The frequency of the output clk_out

	//  = The frequency of the input clk_in divided by DIVISOR

	// For example: Fclk_in = 50Mhz, if you want to get 1Hz signal to blink LEDs

	// You will modify the DIVISOR parameter value to 28'd50.000.000

	// Then the frequency of the output clk_out = 50Mhz/50.000.000 = 1Hz
	always @(posedge clock_in)
	begin
		counter <= counter + 28'd1;
		if(counter>=(DIVISOR-1)) begin
			counter <= 28'd0;
		end
	end
	assign clock_out = (counter<DIVISOR/2)? 1'b0 : 1'b1;
endmodule

module Audio_Controller(
	// Inputs
	CLOCK_50,
	reset,

	clear_audio_in_memory,	
	read_audio_in,

	clear_audio_out_memory,
	left_channel_audio_out,
	right_channel_audio_out,
	write_audio_out,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	// Outputs
	left_channel_audio_in,
	right_channel_audio_in,
	audio_in_available,

	audio_out_allowed,

	AUD_XCK,
	AUD_DACDAT
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

localparam AUDIO_DATA_WIDTH	= 32;
localparam BIT_COUNTER_INIT	= 5'd31;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input				reset;

input				clear_audio_in_memory;
input				read_audio_in;

input				clear_audio_out_memory;
input		[AUDIO_DATA_WIDTH:1]	left_channel_audio_out;
input		[AUDIO_DATA_WIDTH:1]	right_channel_audio_out;
input				write_audio_out;

input				AUD_ADCDAT;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

// Outputs
output	reg			audio_in_available;
output		[AUDIO_DATA_WIDTH:1]	left_channel_audio_in;
output		[AUDIO_DATA_WIDTH:1]	right_channel_audio_in;

output	reg			audio_out_allowed;

output				AUD_XCK;
output				AUD_DACDAT;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				bclk_rising_edge;
wire				bclk_falling_edge;

wire				adc_lrclk_rising_edge;
wire				adc_lrclk_falling_edge;

wire				dac_lrclk_rising_edge;
wire				dac_lrclk_falling_edge;

wire		[7:0]	left_channel_read_available;
wire		[7:0]	right_channel_read_available;

wire		[7:0]	left_channel_write_space;
wire		[7:0]	right_channel_write_space;

// Internal Registers
reg					done_adc_channel_sync;
reg					done_dac_channel_sync;

// State Machine Registers


/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// Output Registers
always @ (posedge CLOCK_50)
begin
	if (reset == 1'b1)
		audio_in_available <= 1'b0;
	else if ((left_channel_read_available[7] | left_channel_read_available[6])
			& (right_channel_read_available[7] | right_channel_read_available[6]))
		audio_in_available <= 1'b1;
	else
		audio_in_available <= 1'b0;
end

always @ (posedge CLOCK_50)
begin
	if (reset == 1'b1)
		audio_out_allowed <= 1'b0;
	else if ((left_channel_write_space[7] | left_channel_write_space[6])
			& (right_channel_write_space[7] | right_channel_write_space[6]))
		audio_out_allowed <= 1'b1;
	else
		audio_out_allowed <= 1'b0;
end

// Internal Registers
always @ (posedge CLOCK_50)
begin
	if (reset == 1'b1)
		done_adc_channel_sync <= 1'b0;
	else if (adc_lrclk_rising_edge == 1'b1)
		done_adc_channel_sync <= 1'b1;
end

always @ (posedge CLOCK_50)
begin
	if (reset == 1'b1)
		done_dac_channel_sync <= 1'b0;
	else if (dac_lrclk_falling_edge == 1'b1)
		done_dac_channel_sync <= 1'b1;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign AUD_BCLK		= 1'bZ;
assign AUD_ADCLRCK	= 1'bZ;
assign AUD_DACLRCK	= 1'bZ;


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_Clock_Edge Bit_Clock_Edges (
	// Inputs
	.clk			(CLOCK_50),
	.reset			(reset),
	
	.test_clk		(AUD_BCLK),
	
	// Bidirectionals

	// Outputs
	.rising_edge	(bclk_rising_edge),
	.falling_edge	(bclk_falling_edge)
);

Altera_UP_Clock_Edge ADC_Left_Right_Clock_Edges (
	// Inputs
	.clk			(CLOCK_50),
	.reset			(reset),
	
	.test_clk		(AUD_ADCLRCK),
	
	// Bidirectionals

	// Outputs
	.rising_edge	(adc_lrclk_rising_edge),
	.falling_edge	(adc_lrclk_falling_edge)
);

Altera_UP_Clock_Edge DAC_Left_Right_Clock_Edges (
	// Inputs
	.clk			(CLOCK_50),
	.reset			(reset),
	
	.test_clk		(AUD_DACLRCK),
	
	// Bidirectionals

	// Outputs
	.rising_edge	(dac_lrclk_rising_edge),
	.falling_edge	(dac_lrclk_falling_edge)
);

Altera_UP_Audio_In_Deserializer Audio_In_Deserializer (
	// Inputs
	.clk							(CLOCK_50),
	.reset							(reset | clear_audio_in_memory),
	
	.bit_clk_rising_edge			(bclk_rising_edge),
	.bit_clk_falling_edge			(bclk_falling_edge),
	.left_right_clk_rising_edge		(adc_lrclk_rising_edge),
	.left_right_clk_falling_edge	(adc_lrclk_falling_edge),

	.done_channel_sync				(done_adc_channel_sync),

	.serial_audio_in_data			(AUD_ADCDAT),

	.read_left_audio_data_en		(read_audio_in & audio_in_available),
	.read_right_audio_data_en		(read_audio_in & audio_in_available),

	// Bidirectionals

	// Outputs
	.left_audio_fifo_read_space		(left_channel_read_available),
	.right_audio_fifo_read_space	(right_channel_read_available),

	.left_channel_data				(left_channel_audio_in),
	.right_channel_data				(right_channel_audio_in)
);
defparam
	Audio_In_Deserializer.AUDIO_DATA_WIDTH = AUDIO_DATA_WIDTH,
	Audio_In_Deserializer.BIT_COUNTER_INIT = BIT_COUNTER_INIT;

Altera_UP_Audio_Out_Serializer Audio_Out_Serializer (
	// Inputs
	.clk							(CLOCK_50),
	.reset							(reset | clear_audio_out_memory),
	
	.bit_clk_rising_edge			(bclk_rising_edge),
	.bit_clk_falling_edge			(bclk_falling_edge),
	.left_right_clk_rising_edge		(done_dac_channel_sync & dac_lrclk_rising_edge),
	.left_right_clk_falling_edge	(done_dac_channel_sync & dac_lrclk_falling_edge),
	
	.left_channel_data				(left_channel_audio_out),
	.left_channel_data_en			(write_audio_out & audio_out_allowed),

	.right_channel_data				(right_channel_audio_out),
	.right_channel_data_en			(write_audio_out & audio_out_allowed),
	
	// Bidirectionals

	// Outputs
	.left_channel_fifo_write_space	(left_channel_write_space),
	.right_channel_fifo_write_space	(right_channel_write_space),

	.serial_audio_out_data			(AUD_DACDAT)
);
defparam
	Audio_Out_Serializer.AUDIO_DATA_WIDTH = AUDIO_DATA_WIDTH;

Audio_Clock Audio_Clock (
	// Inputs
	.inclk0			(CLOCK_50),
	.areset			(),

	// Outputs
	.c0				(AUD_XCK),
	.locked			()
);

endmodule

// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module Audio_Clock (
	areset,
	inclk0,
	c0,
	locked);

	input	  areset;
	input	  inclk0;
	output	  c0;
	output	  locked;

	wire [5:0] sub_wire0;
	wire  sub_wire2;
	wire [0:0] sub_wire5 = 1'h0;
	wire [0:0] sub_wire1 = sub_wire0[0:0];
	wire  c0 = sub_wire1;
	wire  locked = sub_wire2;
	wire  sub_wire3 = inclk0;
	wire [1:0] sub_wire4 = {sub_wire5, sub_wire3};

	altpll	altpll_component (
				.inclk (sub_wire4),
				.areset (areset),
				.clk (sub_wire0),
				.locked (sub_wire2),
				.activeclock (),
				.clkbad (),
				.clkena ({6{1'b1}}),
				.clkloss (),
				.clkswitch (1'b0),
				.configupdate (1'b0),
				.enable0 (),
				.enable1 (),
				.extclk (),
				.extclkena ({4{1'b1}}),
				.fbin (1'b1),
				.fbmimicbidir (),
				.fbout (),
				.pfdena (1'b1),
				.phasecounterselect ({4{1'b1}}),
				.phasedone (),
				.phasestep (1'b1),
				.phaseupdown (1'b1),
				.pllena (1'b1),
				.scanaclr (1'b0),
				.scanclk (1'b0),
				.scanclkena (1'b1),
				.scandata (1'b0),
				.scandataout (),
				.scandone (),
				.scanread (1'b0),
				.scanwrite (1'b0),
				.sclkout0 (),
				.sclkout1 (),
				.vcooverrange (),
				.vcounderrange ());
	defparam
		altpll_component.clk0_divide_by = 4,
		altpll_component.clk0_duty_cycle = 50,
		altpll_component.clk0_multiply_by = 1,
		altpll_component.clk0_phase_shift = "0",
		altpll_component.compensate_clock = "CLK0",
		altpll_component.gate_lock_signal = "NO",
		altpll_component.inclk0_input_frequency = 20000,
		altpll_component.intended_device_family = "Cyclone II",
		altpll_component.invalid_lock_multiplier = 5,
		altpll_component.lpm_hint = "CBX_MODULE_PREFIX=Audio_Clock",
		altpll_component.lpm_type = "altpll",
		altpll_component.operation_mode = "NORMAL",
		altpll_component.port_activeclock = "PORT_UNUSED",
		altpll_component.port_areset = "PORT_USED",
		altpll_component.port_clkbad0 = "PORT_UNUSED",
		altpll_component.port_clkbad1 = "PORT_UNUSED",
		altpll_component.port_clkloss = "PORT_UNUSED",
		altpll_component.port_clkswitch = "PORT_UNUSED",
		altpll_component.port_configupdate = "PORT_UNUSED",
		altpll_component.port_fbin = "PORT_UNUSED",
		altpll_component.port_inclk0 = "PORT_USED",
		altpll_component.port_inclk1 = "PORT_UNUSED",
		altpll_component.port_locked = "PORT_USED",
		altpll_component.port_pfdena = "PORT_UNUSED",
		altpll_component.port_phasecounterselect = "PORT_UNUSED",
		altpll_component.port_phasedone = "PORT_UNUSED",
		altpll_component.port_phasestep = "PORT_UNUSED",
		altpll_component.port_phaseupdown = "PORT_UNUSED",
		altpll_component.port_pllena = "PORT_UNUSED",
		altpll_component.port_scanaclr = "PORT_UNUSED",
		altpll_component.port_scanclk = "PORT_UNUSED",
		altpll_component.port_scanclkena = "PORT_UNUSED",
		altpll_component.port_scandata = "PORT_UNUSED",
		altpll_component.port_scandataout = "PORT_UNUSED",
		altpll_component.port_scandone = "PORT_UNUSED",
		altpll_component.port_scanread = "PORT_UNUSED",
		altpll_component.port_scanwrite = "PORT_UNUSED",
		altpll_component.port_clk0 = "PORT_USED",
		altpll_component.port_clk1 = "PORT_UNUSED",
		altpll_component.port_clk2 = "PORT_UNUSED",
		altpll_component.port_clk3 = "PORT_UNUSED",
		altpll_component.port_clk4 = "PORT_UNUSED",
		altpll_component.port_clk5 = "PORT_UNUSED",
		altpll_component.port_clkena0 = "PORT_UNUSED",
		altpll_component.port_clkena1 = "PORT_UNUSED",
		altpll_component.port_clkena2 = "PORT_UNUSED",
		altpll_component.port_clkena3 = "PORT_UNUSED",
		altpll_component.port_clkena4 = "PORT_UNUSED",
		altpll_component.port_clkena5 = "PORT_UNUSED",
		altpll_component.port_extclk0 = "PORT_UNUSED",
		altpll_component.port_extclk1 = "PORT_UNUSED",
		altpll_component.port_extclk2 = "PORT_UNUSED",
		altpll_component.port_extclk3 = "PORT_UNUSED",
		altpll_component.valid_lock_multiplier = 1;


endmodule

module Altera_UP_SYNC_FIFO (
	// Inputs
	clk,
	reset,

	write_en,
	write_data,

	read_en,
	
	// Bidirectionals

	// Outputs
	fifo_is_empty,
	fifo_is_full,
	words_used,

	read_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter	DATA_WIDTH	= 32;
parameter	DATA_DEPTH	= 128;
parameter	ADDR_WIDTH	= 7;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				clk;
input				reset;

input				write_en;
input		[DATA_WIDTH:1]	write_data;

input				read_en;

// Bidirectionals

// Outputs
output				fifo_is_empty;
output				fifo_is_full;
output		[ADDR_WIDTH:1]	words_used;

output		[DATA_WIDTH:1]	read_data;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/


/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


scfifo	Sync_FIFO (
	// Inputs
	.clock			(clk),
	.sclr			(reset),

	.data			(write_data),
	.wrreq			(write_en),

	.rdreq			(read_en),

	// Bidirectionals

	// Outputs
	.empty			(fifo_is_empty),
	.full			(fifo_is_full),
	.usedw			(words_used),
	
	.q				(read_data)

	// Unused
	// synopsys translate_off
	,
	.aclr			(),
	.almost_empty	(),
	.almost_full	()
	// synopsys translate_on
);
defparam
	Sync_FIFO.add_ram_output_register	= "OFF",
	Sync_FIFO.intended_device_family	= "Cyclone IV E",
	Sync_FIFO.lpm_numwords				= DATA_DEPTH,
	Sync_FIFO.lpm_showahead				= "ON",
	Sync_FIFO.lpm_type					= "scfifo",
	Sync_FIFO.lpm_width					= DATA_WIDTH,
	Sync_FIFO.lpm_widthu				= ADDR_WIDTH,
	Sync_FIFO.overflow_checking			= "OFF",
	Sync_FIFO.underflow_checking		= "OFF",
	Sync_FIFO.use_eab					= "ON";

endmodule

module Altera_UP_Clock_Edge (
	// Inputs
	clk,
	reset,
	
	test_clk,
	
	// Bidirectionals

	// Outputs
	rising_edge,
	falling_edge
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				clk;
input				reset;
	
input				test_clk;

// Bidirectionals

// Outputs
output				rising_edge;
output				falling_edge;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				found_edge;

// Internal Registers
reg					cur_test_clk;
reg					last_test_clk;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
	cur_test_clk	<= test_clk;

always @(posedge clk)
	last_test_clk	<= cur_test_clk;

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

// Output Assignments
assign rising_edge	= found_edge & cur_test_clk;
assign falling_edge	= found_edge & last_test_clk;

// Internal Assignments
assign found_edge	= last_test_clk ^ cur_test_clk;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

endmodule

module Altera_UP_Audio_Out_Serializer (
	// Inputs
	clk,
	reset,
	
	bit_clk_rising_edge,
	bit_clk_falling_edge,
	left_right_clk_rising_edge,
	left_right_clk_falling_edge,
	
	left_channel_data,
	left_channel_data_en,

	right_channel_data,
	right_channel_data_en,
	
	// Bidirectionals

	// Outputs
	left_channel_fifo_write_space,
	right_channel_fifo_write_space,

	serial_audio_out_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter AUDIO_DATA_WIDTH	= 32;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				bit_clk_rising_edge;
input				bit_clk_falling_edge;
input				left_right_clk_rising_edge;
input				left_right_clk_falling_edge;

input		[AUDIO_DATA_WIDTH:1]		left_channel_data;
input				left_channel_data_en;

input		[AUDIO_DATA_WIDTH:1]		right_channel_data;
input				right_channel_data_en;

// Bidirectionals

// Outputs
output	reg	[7:0]	left_channel_fifo_write_space;
output	reg	[7:0]	right_channel_fifo_write_space;

output	reg			serial_audio_out_data;


/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				read_left_channel;
wire				read_right_channel;

wire				left_channel_fifo_is_empty;
wire				right_channel_fifo_is_empty;

wire				left_channel_fifo_is_full;
wire				right_channel_fifo_is_full;

wire		[6:0]	left_channel_fifo_used;
wire		[6:0]	right_channel_fifo_used;

wire		[AUDIO_DATA_WIDTH:1]		left_channel_from_fifo;
wire		[AUDIO_DATA_WIDTH:1]		right_channel_from_fifo;

// Internal Registers
reg					left_channel_was_read;
reg			[AUDIO_DATA_WIDTH:1]	data_out_shift_reg;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		left_channel_fifo_write_space <= 8'h00;
	else
		left_channel_fifo_write_space <= 8'h80 - {left_channel_fifo_is_full,left_channel_fifo_used};
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		right_channel_fifo_write_space <= 8'h00;
	else
		right_channel_fifo_write_space <= 8'h80 - {right_channel_fifo_is_full,right_channel_fifo_used};
end


always @(posedge clk)
begin
	if (reset == 1'b1)
		serial_audio_out_data <= 1'b0;
	else
		serial_audio_out_data <= data_out_shift_reg[AUDIO_DATA_WIDTH];
end


always @(posedge clk)
begin
	if (reset == 1'b1)
		left_channel_was_read <= 1'b0;
	else if (read_left_channel)
		left_channel_was_read <=1'b1;
	else if (read_right_channel)
		left_channel_was_read <=1'b0;
end


always @(posedge clk)
begin
	if (reset == 1'b1)
		data_out_shift_reg	<= {AUDIO_DATA_WIDTH{1'b0}};
	else if (read_left_channel)
		data_out_shift_reg	<= left_channel_from_fifo;
	else if (read_right_channel)
		data_out_shift_reg	<= right_channel_from_fifo;
	else if (left_right_clk_rising_edge | left_right_clk_falling_edge)
		data_out_shift_reg	<= {AUDIO_DATA_WIDTH{1'b0}};
	else if (bit_clk_falling_edge)
		data_out_shift_reg	<= 
			{data_out_shift_reg[(AUDIO_DATA_WIDTH - 1):1], 1'b0};
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign read_left_channel	= left_right_clk_rising_edge &
								 ~left_channel_fifo_is_empty & 
								 ~right_channel_fifo_is_empty;
assign read_right_channel	= left_right_clk_falling_edge &
								left_channel_was_read;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_SYNC_FIFO Audio_Out_Left_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(left_channel_data_en & ~left_channel_fifo_is_full),
	.write_data		(left_channel_data),

	.read_en		(read_left_channel),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(left_channel_fifo_is_empty),
	.fifo_is_full	(left_channel_fifo_is_full),
	.words_used		(left_channel_fifo_used),

	.read_data		(left_channel_from_fifo)
);
defparam 
	Audio_Out_Left_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_Out_Left_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_Out_Left_Channel_FIFO.ADDR_WIDTH	= 7;

Altera_UP_SYNC_FIFO Audio_Out_Right_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(right_channel_data_en & ~right_channel_fifo_is_full),
	.write_data		(right_channel_data),

	.read_en		(read_right_channel),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(right_channel_fifo_is_empty),
	.fifo_is_full	(right_channel_fifo_is_full),
	.words_used		(right_channel_fifo_used),

	.read_data		(right_channel_from_fifo)
);
defparam 
	Audio_Out_Right_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_Out_Right_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_Out_Right_Channel_FIFO.ADDR_WIDTH	= 7;

endmodule

module Altera_UP_Audio_In_Deserializer (
	// Inputs
	clk,
	reset,
	
	bit_clk_rising_edge,
	bit_clk_falling_edge,
	left_right_clk_rising_edge,
	left_right_clk_falling_edge,

	done_channel_sync,

	serial_audio_in_data,

	read_left_audio_data_en,
	read_right_audio_data_en,

	// Bidirectionals

	// Outputs
	left_audio_fifo_read_space,
	right_audio_fifo_read_space,

	left_channel_data,
	right_channel_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter AUDIO_DATA_WIDTH	= 32;
parameter BIT_COUNTER_INIT	= 5'd31;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				bit_clk_rising_edge;
input				bit_clk_falling_edge;
input				left_right_clk_rising_edge;
input				left_right_clk_falling_edge;

input				done_channel_sync;

input				serial_audio_in_data;

input				read_left_audio_data_en;
input				read_right_audio_data_en;

// Bidirectionals

// Outputs
output	reg	[7:0]	left_audio_fifo_read_space;
output	reg	[7:0]	right_audio_fifo_read_space;

output		[AUDIO_DATA_WIDTH:1]	left_channel_data;
output		[AUDIO_DATA_WIDTH:1]	right_channel_data;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				valid_audio_input;

wire				left_channel_fifo_is_empty;
wire				right_channel_fifo_is_empty;

wire				left_channel_fifo_is_full;
wire				right_channel_fifo_is_full;

wire		[6:0]	left_channel_fifo_used;
wire		[6:0]	right_channel_fifo_used;

// Internal Registers
reg			[AUDIO_DATA_WIDTH:1]	data_in_shift_reg;

// State Machine Registers


/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		left_audio_fifo_read_space			<= 8'h00;
	else
	begin
		left_audio_fifo_read_space[7]		<= left_channel_fifo_is_full;
		left_audio_fifo_read_space[6:0]		<= left_channel_fifo_used;
	end
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		right_audio_fifo_read_space			<= 8'h00;
	else
	begin
		right_audio_fifo_read_space[7]		<= right_channel_fifo_is_full;
		right_audio_fifo_read_space[6:0]	<= right_channel_fifo_used;
	end
end




always @(posedge clk)
begin
	if (reset == 1'b1)
		data_in_shift_reg	<= {AUDIO_DATA_WIDTH{1'b0}};
	else if (bit_clk_rising_edge & valid_audio_input)
		data_in_shift_reg	<= 
			{data_in_shift_reg[(AUDIO_DATA_WIDTH - 1):1], 
			 serial_audio_in_data};
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_Audio_Bit_Counter Audio_Out_Bit_Counter (
	// Inputs
	.clk							(clk),
	.reset							(reset),
	
	.bit_clk_rising_edge			(bit_clk_rising_edge),
	.bit_clk_falling_edge			(bit_clk_falling_edge),
	.left_right_clk_rising_edge		(left_right_clk_rising_edge),
	.left_right_clk_falling_edge	(left_right_clk_falling_edge),

	// Bidirectionals

	// Outputs
	.counting						(valid_audio_input)
);
defparam 
	Audio_Out_Bit_Counter.BIT_COUNTER_INIT	= BIT_COUNTER_INIT;

Altera_UP_SYNC_FIFO Audio_In_Left_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(left_right_clk_falling_edge & ~left_channel_fifo_is_full & done_channel_sync),
	.write_data		(data_in_shift_reg),

	.read_en		(read_left_audio_data_en & ~left_channel_fifo_is_empty),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(left_channel_fifo_is_empty),
	.fifo_is_full	(left_channel_fifo_is_full),
	.words_used		(left_channel_fifo_used),

	.read_data		(left_channel_data)
);
defparam 
	Audio_In_Left_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_In_Left_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_In_Left_Channel_FIFO.ADDR_WIDTH	= 7;

Altera_UP_SYNC_FIFO Audio_In_Right_Channel_FIFO(
	// Inputs
	.clk			(clk),
	.reset			(reset),

	.write_en		(left_right_clk_rising_edge & ~right_channel_fifo_is_full & done_channel_sync),
	.write_data		(data_in_shift_reg),

	.read_en		(read_right_audio_data_en & ~right_channel_fifo_is_empty),
	
	// Bidirectionals

	// Outputs
	.fifo_is_empty	(right_channel_fifo_is_empty),
	.fifo_is_full	(right_channel_fifo_is_full),
	.words_used		(right_channel_fifo_used),

	.read_data		(right_channel_data)
);
defparam 
	Audio_In_Right_Channel_FIFO.DATA_WIDTH	= AUDIO_DATA_WIDTH,
	Audio_In_Right_Channel_FIFO.DATA_DEPTH	= 128,
	Audio_In_Right_Channel_FIFO.ADDR_WIDTH	= 7;

endmodule

module Altera_UP_Audio_Bit_Counter (
	// Inputs
	clk,
	reset,
	
	bit_clk_rising_edge,
	bit_clk_falling_edge,
	left_right_clk_rising_edge,
	left_right_clk_falling_edge,
	
	// Bidirectionals

	// Outputs
	counting
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter BIT_COUNTER_INIT	= 5'd31;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input				clk;
input				reset;
	
input				bit_clk_rising_edge;
input				bit_clk_falling_edge;
input				left_right_clk_rising_edge;
input				left_right_clk_falling_edge;

// Bidirectionals

// Outputs
output	reg			counting;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/


/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/

// Internal Wires
wire				reset_bit_counter;

// Internal Registers
reg			[4:0]	bit_counter;

// State Machine Registers


/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		bit_counter <= 5'h00;
	else if (reset_bit_counter == 1'b1)
		bit_counter <= BIT_COUNTER_INIT;
	else if ((bit_clk_falling_edge == 1'b1) && (bit_counter != 5'h00))
		bit_counter <= bit_counter - 5'h01;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		counting <= 1'b0;
	else if (reset_bit_counter == 1'b1)
		counting <= 1'b1;
	else if ((bit_clk_falling_edge == 1'b1) && (bit_counter == 5'h00))
		counting <= 1'b0;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign reset_bit_counter = left_right_clk_rising_edge | 
							left_right_clk_falling_edge;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

endmodule
