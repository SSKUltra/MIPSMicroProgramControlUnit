`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   17:08:39 03/07/2019
// Design Name:   TopModule
// Module Name:   G:/Xilinx/projects/MicroprogrammedControlUnit2/TopModuleTest.v
// Project Name:  MicroprogrammedControlUnit2
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: TopModule
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TopModuleTest;

	// Inputs
	reg clk;
	reg [5:0] opcode;
	reg [5:0] funcfield;
	reg overflow;

	// Instantiate the Unit Under Test (UUT)
	TopModule uut (
		.clk(clk), 
		.opcode(opcode), 
		.funcfield(funcfield),
		.overflow(overflow)
	); 
	always
	begin
		#10 clk=~clk;
	end
	initial begin
		// Initialize Inputs
		clk = 0;		//mfhi
		opcode = 6'd0;
		funcfield = 6'b010000; 
		overflow = 0;
		
		#60		//mflo
		opcode = 6'd0;
		funcfield = 6'b010010;
		
		#60		//mthi
		opcode = 6'd0;
		funcfield = 6'b010001;
		
		#60		//mtlo
		opcode = 6'd0;
		funcfield = 6'b010011;
		
		#60		//lui
		opcode = 6'b001111;
		funcfield = 6'bxxxxxx;
		
		#60		//beq
		opcode = 6'b000100;
		funcfield = 6'bxxxxxx;
		
		#60		//jump
		opcode = 6'b000010;
		funcfield = 6'bxxxxxx;
		
		#60		//jal
		opcode = 6'b000011;
		funcfield = 6'bxxxxxx;
		
		#60		//jr
		opcode = 6'd0;
		funcfield = 6'b001000;
		
		#60		//jalr
		opcode = 6'd0;
		funcfield = 6'b001001;
		
		#60		//add
		opcode = 6'd0;
		funcfield = 6'b100000;
		
		#80		//add overflow
		overflow = 1;
		opcode = 6'd0;
		funcfield = 6'b100000;
		
		#140		//sll
		overflow = 0;
		opcode = 6'd0;
		funcfield = 6'b000000;

		#80			//sllv
		opcode = 6'd0;
		funcfield = 6'b000100;

		#80			//div
		opcode = 6'd0;
		funcfield = 6'b011010;

		#80			//mult
		opcode = 6'd0;
		funcfield = 6'b011000;

		#80			//madd
		opcode = 6'b011100;
		funcfield = 6'b000000;
		
		#80			//msub
		opcode = 6'b011100;
		funcfield = 6'b000100;

		#80			//addi
		opcode = 6'b001000;
		funcfield = 6'bxxxxxx;

		#80			//lw
		opcode = 6'b100011;
		funcfield = 6'bxxxxxx;

		#100		//sw
		opcode = 6'b101011;
		funcfield = 6'bxxxxxx;

		#80		//ori
		opcode = 6'b001101;
		funcfield = 6'bxxxxxx;

		#80		//Undefined Instruction
		opcode = 6'b111111;
		funcfield = 6'bxxxxxx;

	end    
endmodule

