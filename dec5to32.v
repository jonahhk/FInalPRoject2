//Contains all decoders and multiplexers used in the circuit//

module dec5to32(in, out);
	input [4:0] in;  
	output [31:0] out; 
	
	wire[3:0] ena; 
	
	dec2to4 decEna(in[4:3], 1'b1, ena); 
	
	dec3to8 dec0(in[2:0], ena[0], out[7:0]);
	dec3to8 dec1(in[2:0], ena[1], out[15:8]);
	dec3to8 dec2(in[2:0], ena[2], out[23:16]);
	dec3to8 dec3(in[2:0], ena[3], out[31:24]); 
	
endmodule 

module dec3to8(in, enable, out);
	input [2:0] in; 
	input enable; 
	output [7:0] out;
	
	wire [7:0] myOut;
	wire [2:0] nIn; 
	
	not n0(nIn[0], in[0]);
	not n1(nIn[1], in[1]);
	not n2(nIn[2], in[2]); 
	
	and and0(myOut[0], nIn[2], nIn[1], nIn[0]);
	assign out[0] = enable ? myOut[0] : 1'b0;
	and and1(myOut[1], nIn[2], nIn[1], in[0]);
	assign out[1] = enable ? myOut[1] : 1'b0; 
	and and2(myOut[2], nIn[2], in[1], nIn[0]);
	assign out[2] = enable ? myOut[2] : 1'b0; 
	and and3(myOut[3], nIn[2], in[1], in[0]);
	assign out[3] = enable ? myOut[3] : 1'b0; 
	and and4(myOut[4], in[2], nIn[1], nIn[0]);
	assign out[4] = enable ? myOut[4] : 1'b0; 
	and and5(myOut[5], in[2], nIn[1], in[0]);
	assign out[5] = enable ? myOut[5] : 1'b0; 
	and and6(myOut[6], in[2], in[1], nIn[0]);
	assign out[6] = enable ? myOut[6] : 1'b0; 
	and and7(myOut[7], in[2], in[1], in[0]);
	assign out[7] = enable ? myOut[7] : 1'b0; 
	
endmodule

module dec2to4(in, ena, out);
	input [1:0] in;
	input ena; 
	output [3:0] out;
	
	wire[1:0] nIn;
	
	not n0(nIn[0], in[0]);
	not n1(nIn[1], in[1]);
	
	and and0(out[0], nIn[1], nIn[0], ena);
	and and1(out[1], nIn[1], in[0], ena);
	and and2(out[2], in[1], nIn[0], ena);
	and and3(out[3], in[1], in[0], ena);
	
endmodule

module mux_8(select, in0, in1, in2, in3, in4, in5, in6, in7, out);
		input [2:0] select; 
		input [31:0] in0, in1, in2, in3, in4, in5, in6, in7;
		output [31:0] out; 
		wire [31:0] w1, w2; 
		mux_4 first_top(select[1:0], in0, in1, in2, in3, w1);
		mux_4 first_bottom(select[1:0], in4, in5, in6, in7, w2); 
		mux_2 second(select[2], w1, w2, out);
	endmodule

module mux_4(select, in0, in1, in2, in3, out);
		input [1:0] select; 
		input [31:0] in0, in1, in2, in3; 
		output [31:0] out; 
		wire [31:0] w1, w2; 
		wire [1:0] ns0, ns1; 
		
		mux_2 first_top(select[0], in0, in1, w1);
		mux_2 first_bottom(select[0], in2, in3, w2); 
		mux_2 second(select[1], w1, w2, out);
	endmodule

module mux_2(sel, a, b, out);
		input [31:0] a, b; 
		input sel; 
		output [31:0] out; 
	
		assign out = sel ? b : a;
endmodule

module negMux2to1(b, sel, out);
		input [31:0] b;
		input sel; 
		output [31:0] out; 
	
		genvar i; 
			generate 
				for(i=0; i<32; i=i+1) begin: loop
					xor s0(out[i], b[i], sel);
				end 
			endgenerate 
endmodule