module regfile (
    clock,
    ctrl_writeEnable,
    ctrl_reset, ctrl_writeReg,
    ctrl_readRegA, ctrl_readRegB, data_writeReg,
    data_readRegA, data_readRegB
);

   input clock, ctrl_writeEnable, ctrl_reset;
   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
   input [31:0] data_writeReg;
   output [31:0] data_readRegA, data_readRegB;

   wire [31:0] regSelect, regEnable, readRegA, readRegB, registerData[31:0];
	
	dec5to32 writeSelect(ctrl_writeReg, regSelect); 
	dec5to32 readSelectA(ctrl_readRegA, readRegA);
	dec5to32 readSelectB(ctrl_readRegB, readRegB);
	
	my_32tri single32TriA(32'd0, readRegA[0], data_readRegA);
	my_32tri single32TriB(32'd0, readRegB[0], data_readRegB);
			
	genvar i; 
	generate 
		for(i=1; i<32; i=i+1) begin: loop
			and andEnable(regEnable[i], regSelect[i], ctrl_writeEnable);
			reg32 myReg(~clock, ctrl_reset, regEnable[i], data_writeReg , registerData[i]);
			my_32tri loop32TriA(registerData[i], readRegA[i], data_readRegA);
			my_32tri loop32TriB(registerData[i], readRegB[i], data_readRegB);
		end 
	endgenerate 
	

endmodule

module reg32(clk, clr, ctrl_write, d, q);
	input clk, clr, ctrl_write;
	input [31:0] d; 
	output [31:0] q;

	genvar i; 
	generate 
		for(i=0; i<32; i=i+1) begin: loop1
			dflipflop singleDFF(d[i], clk, ctrl_write, clr, q[i]); 
		end 
	endgenerate 
	
endmodule 

module reg12(clk, clr, ctrl_write, d, q);
	input clk, clr, ctrl_write;
	input [11:0] d; 
	output [11:0] q;

	genvar i; 
	generate 
		for(i=0; i<12; i=i+1) begin: loop1
			dflipflop singleDFF(d[i], clk, ctrl_write, clr, q[i]); 
		end 
	endgenerate 
	
endmodule 

module my_32tri(in, oe, out);
	input [31:0] in;
	input oe;
	output [31:0] out; 
	
	genvar i; 
	generate 
		for(i=0; i<32; i=i+1) begin: loop1
			assign out[i] = oe ? in[i] : 1'bz;
		end 
	endgenerate 
	
	
endmodule 

module dflipflop(d, clk, ena, clr, q);
	input d, clk, ena, clr;
	output q;
	reg q; 
	
	initial 
	begin 
		q = 1'b0; 
	end 
	
	always @(posedge clk or posedge clr) begin 
		if (q == 1'bx) begin
			q <= 1'b0;
		end else if(clr) begin 
			q <= 1'b0;
		end else if (ena) begin 
			q <= d; 
			end 
	end
endmodule 