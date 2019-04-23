//These modules contain noop logic and bypass logic 

//make sure these are referring to the correct pipes
module pipelineHazard(fInsn, dInsn, stall);
	input [31:0] fInsn, dInsn;
	output stall;
	
	wire isLoad, isNotStore, haz1, haz2, haz3, haz4;
	wire [1:0] RDselect; 
	wire [4:0] RS1, RS2, dReadAdd, topBitSelect, when0 ;
	
	assign isLoad = ~(dInsn[31]) && dInsn[30] && ~(|dInsn[29:27]);
	assign isNotStore = ~(~(|fInsn[31:30]) && (&fInsn[29:27]));
	
	//This is the logic for RS1
			//assign  topBitSelect = fInsn[31] ? 5'b11110 : fInsn[26:22];  
			//mux_4 regASelect(fInsn[28:27], fInsn[21:17], fInsn[21:17],  
									//topBitSelect, fInsn[21:17], RS1);  
	assign RS1 = fInsn[21:17]; 						
	//This is the logic for RS2
			//assign RS2 = fInsn[28] ? fInsn[21:17] : fInsn[16:12];
	assign when0 = (fInsn[28] || fInsn[29]) ? fInsn[26:22] : fInsn[16:12]; 
	assign RS2 = fInsn[31] ? 5'b11110 : when0;			
	
	//This is the logic for destination register
	assign RDselect[0] = dInsn[28]; //If asserted, $r31
	assign RDselect[1] = dInsn[31]; //If asserted, $r30
	mux_4 findRD(RDselect, dInsn[26:22], 5'b11111, 5'b11110, dInsn[26:22], dReadAdd); 
	
	equalCompare5 RS2andRD(RS2, dReadAdd, haz1); 
	equalCompare5 RS1andRD(RS1, dReadAdd, haz2); 
	
	assign stall = (isLoad && ((haz1&& |RS2) || ((haz2 && |RS1) && isNotStore)));
			  

endmodule

module bypassALUA(eInsn, mInsn, wInsn, select); 
	//If select = ...0 take RS1in, 1 take from m, 2 take from w, 3 just take  m
	input [31:0] eInsn, mInsn, wInsn; 
	output [1:0] select; 
	
	wire takeM, takeW, isLW1, isAddi1, isRtype1, bypass1, isSetX1;
	wire isLW2, isAddi2, isRtype2, bypass2, isSetX2; 
	wire [4:0] RS1, mRA, wRA; 
	
	assign RS1 = eInsn[21:17];  //00001
	//This is the logic for destination register
	assign mRA = mInsn[26:22];  //00000
	//This is the logic for destination register
	assign wRA = wInsn[26:22];  //00010

	//Only take $rd when the type is Addi, Rtype, setx, and Lw 
	assign isRtype1 = ~(|mInsn[31:27]); 
	assign isLW1   = ~(mInsn[31]) && (mInsn[30]) && ~(|mInsn[29:27]);
	assign isAddi1 = ~(|mInsn[31:30]) && mInsn[29] && ~mInsn[28] && mInsn[27];
	assign isSetX1 = mInsn[31] && ~mInsn[30] && mInsn[29] && ~mInsn[28] && mInsn[27]; 
	
	assign bypass1 = (((isRtype1 || isLW1 || isAddi1) && (|mInsn[26:22])) || (isSetX1));
	
	//Only take $rd when the type is Addi, Rtype, and Lw 
	assign isRtype2 = ~(|wInsn[31:27]); 
	assign isLW2    = ~(wInsn[31]) && (wInsn[30]) && ~(|wInsn[29:27]);
	assign isAddi2 = ~(|wInsn[31:30]) && wInsn[29] && ~wInsn[28] && wInsn[27];
	assign isSetX2 = wInsn[31] && ~wInsn[30] && wInsn[29] && ~wInsn[28] && wInsn[27]; 
	assign bypass2 = (((isRtype2 || isLW2 || isAddi2) && (|wInsn[26:22])) || (isSetX2));
	

	equalCompare5 preALUout(RS1, mRA, takeM); 
	equalCompare5 writeBackData(RS1, wRA, takeW);
	
	assign select[1] = (takeW && bypass2);  //0
	assign select[0] = (takeM && bypass1);  //1
endmodule 

module bypassALUB(eInsn, mInsn, wInsn, select); 
	//If select = ...0 take A, 1 take from m, 2 take from w, 3 take from m
			//Three takes from m since it is more up to date 
	input [31:0] eInsn, mInsn, wInsn; 
	output [1:0] select; 
	
	wire takeM, takeW, isRtype1, isLw1, isAddi1, isRtype2, isLw2, isAddi2,  bypass1, bypass2; 
	wire [1:0] selectBit, topBitSelect;
	wire [4:0] mRA, wRA, when0, RS2; 
	
	//This is the logic for RS2
				//assign RS2 = eInsn[28] ? eInsn[21:17] : eInsn[16:12];
	assign when0 = (eInsn[28] || eInsn[29]) ? eInsn[26:22] : eInsn[16:12]; 
	assign RS2 = eInsn[31] ? 5'b11110 : when0;			
	//This is the logic for destination register
	assign mRA = mInsn[26:22];
	//This is the logic for destination register
	assign wRA = wInsn[26:22];
	
	//Only take $rd when the type is Addi, Rtype, and Lw 
	assign isRtype1 = ~(|mInsn[31:27]); 
	assign isLw1   = ~(mInsn[31]) && (mInsn[30]) && ~(|mInsn[29:27]);
	assign isAddi1 = ~(|mInsn[31:30]) && mInsn[29] && ~mInsn[28] && mInsn[27];
	
	assign bypass1 = isRtype1 || isLw1 || isAddi1;
	
	//Only take $rd when the type is Addi, Rtype, and Lw 
	assign isRtype2 = ~(|wInsn[31:27]); 
	assign isLw2    = ~(wInsn[31]) && (wInsn[30]) && ~(|wInsn[29:27]);
	assign isAddi2 = ~(|wInsn[31:30]) && wInsn[29] && ~wInsn[28] && wInsn[27];
	
	assign bypass2 =isRtype2 || isLw2 || isAddi2;
	

	equalCompare5 preALUout(RS2, mRA, takeM); 
	equalCompare5 writeBackData(RS2, wRA, takeW);
	
	assign select[0] = takeM && (|mInsn[26:22]) && bypass1; 
	assign select[1] = takeW && (|wInsn[26:22]) && bypass2; 
endmodule 

module equalCompare(addrOne, addrTwo, equal); 
	input [11:0] addrOne, addrTwo; 
	output equal; 
	
	wire eq1, eq2, eq3; 
	
	equalCompare4 lowBits(addrOne[3:0], addrTwo[3:0], eq1); 
	equalCompare4 midBits(addrOne[7:4], addrTwo[7:4], eq2); 
	equalCompare4 highBits(addrOne[11:8], addrTwo[11:8], eq3);

	and final12(equal, eq1, eq2, eq3); 
	
endmodule

module equalCompare4(a, b, equal); 
	input [3:0] a, b; 
	output equal; 
	
	wire bit0, bit1, bit2, bit3;
	
	xnor first(bit0, a[0], b[0]); 
	xnor second(bit1, a[1], b[1]); 
	xnor third(bit2, a[2], b[2]); 
	xnor fourth(bit3, a[3], b[3]); 
	
	and myfinal(equal, bit0, bit1, bit2, bit3);
	
endmodule

module equalCompare5(a, b, equal); 
	input [4:0] a, b; 
	output equal; 
	
	wire bit0, bit1, bit2, bit3, bit4;
	
	xnor first(bit0, a[0], b[0]); 
	xnor second(bit1, a[1], b[1]); 
	xnor third(bit2, a[2], b[2]); 
	xnor fourth(bit3, a[3], b[3]); 
	xnor fifth(bit4, a[4], b[4]); 
	
	assign equal = bit0 && bit1 && bit2 && bit3 && bit4; 
	
endmodule

module NequalCompare32(a, b, Nequal); 
	input[31:0] a, b;
	output Nequal; 
	
	wire [7:0] bits; 
	equalCompare4(a[31:28], b[31:28], bits[7]); 
	equalCompare4(a[27:24], b[27:24], bits[6]); 
	equalCompare4(a[23:20], b[23:20], bits[5]); 
	equalCompare4(a[19:16], b[19:16], bits[4]); 
	equalCompare4(a[15:12], b[15:12], bits[3]); 
	equalCompare4(a[11:8], b[11:8], bits[2]); 
	equalCompare4(a[7:4], b[7:4], bits[1]); 
	equalCompare4(a[3:0], b[3:0], bits[0]); 
	
	assign Nequal = !(bits[7] && bits[6] && bits[5] && bits[4] &&
						  bits[3] && bits[2] && bits[1] && bits[0]); 
endmodule 

