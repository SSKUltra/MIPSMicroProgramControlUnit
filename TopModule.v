`timescale 1ns / 1ps

module Adder(
input [31:0] inputA,
input [31:0] inputB,
output reg [31:0] ALUOut);

always@(inputA or inputB)
	begin
		ALUOut<=inputA+inputB;
	end

endmodule 

module DT1(
	input [5:0] opcode,
	input [5:0] funcField,
	output reg [31:0] addressOut
    );
	reg [43:0] memory [0:255];
	reg [43:0] ii = 44'd0;
	initial begin
		$readmemb("dispatchtable1.tv", memory);
	end
	always @(opcode or funcField) 
	begin
		addressOut = 32'b00000000000000000000000000011011; //default value as exception  routine 
		ii = 44'd0;
		while (ii != memory[0])
		begin
			ii =  ii + 1;
			if(opcode == 6'b000000 || opcode == 6'b011100)
			begin
				if((opcode == memory[ii][43:38]) & (funcField == memory[ii][37:32]))
				begin
					addressOut = memory[ii][31:0];
				end
			end
			else if(opcode == memory[ii][43:38]) 
			begin
				addressOut = memory[ii][31:0];
			end	
		end
	end
endmodule 

module DT2(
	input [5:0] opcode,
	input [5:0] funcField,
	output reg [31:0] addressOut
    );
	reg [43:0] memory [0:255];
	reg [43:0] ii = 44'd0;
	initial begin
		$readmemb("dispatchtable2.tv", memory);
	end
	always @(opcode or funcField) 
	begin
		ii = 44'd0;
		while (ii != memory[0])
		begin
			ii =  ii + 1;
			if(opcode == 6'b000000 || opcode == 6'b011100)
			begin
				if((opcode == memory[ii][43:38]) & (funcField == memory[ii][37:32]))
				begin
					addressOut = memory[ii][31:0];
				end
			end
			else if(opcode == memory[ii][43:38])
			begin
				addressOut = memory[ii][31:0];
			end	
		end
	end
endmodule 

module DT3(
	input [5:0] opcode,
	input [5:0] funcField,
	output reg [31:0] addressOut
    );
	reg [43:0] memory [0:255];
	reg [43:0] ii = 44'd0;
	initial begin
		$readmemb("dispatchtable3.tv", memory);
	end
	always @(opcode or funcField) 
	begin
		ii = 44'd0;
		while (ii != memory[0])
		begin
			ii =  ii + 1;
			if(opcode == 6'b000000 || opcode == 6'b011100)
			begin
				if((opcode == memory[ii][43:38]) & (funcField == memory[ii][37:32]))
				begin
					addressOut = memory[ii][31:0];
				end
			end
			else if(opcode == memory[ii][43:38])
			begin
				addressOut = memory[ii][31:0];
			end	
		end
	end
endmodule 

module DT4(
	input [5:0] opcode,
	input [5:0] funcField,
    input overflow,
	output reg [31:0] addressOut
    );
	reg [43:0] memory [0:255];
	reg [43:0] ii = 44'd0;
	initial begin
		$readmemb("dispatchtable4.tv", memory);
	end
	always @(opcode or funcField or overflow) 
	begin
		addressOut = 32'd0;	//fetch default value
		ii = 44'd0;
		while (ii != memory[0])
		begin
			ii =  ii + 1;
			if(opcode == 6'b000000 || opcode == 6'b011100)
			begin
				if((opcode == memory[ii][43:38]) & (funcField == memory[ii][37:32]) & (overflow == 1))
				begin
					addressOut = memory[ii][31:0];
				end
			end
			else if((opcode == memory[ii][43:38]) & (overflow == 1)) 
			begin
                addressOut = memory[ii][31:0];
			end	
		end
	end
endmodule 

module MicroInstructionMem(
    input [31:0] address,
    output reg [35:0] instruction
);
reg [35:0] memory [0:255];
initial begin 
    $readmemb("microinstructions.tv", memory);
end
always @(address) 
begin
    instruction<=memory[address[31:0]];
end
endmodule

module Mux8to1(
input [31:0] outR0,input [31:0] outR1, input [31:0] outR2,input [31:0] outR3,input [31:0] outR4,input [31:0] outR5,
input [31:0] outR6,input [31:0] outR7,input [2:0]Sel,output reg [31:0] outBus);

initial begin
    outBus = 32'd0;
end
always @ (Sel,outR0, outR1,outR2,outR3,outR4,outR5,outR6,outR7)
begin
case(Sel)
3'd0:outBus=outR0;
3'd1:outBus=outR1;
3'd2:outBus=outR2;
3'd3:outBus=outR3;
3'd4:outBus=outR4;
3'd5:outBus=outR5;
3'd6:outBus=outR6;
3'd7:outBus=outR7;

endcase
end

endmodule

module Register32bit(
	 input clk,
	 input regWrite,
    input [31:0] writedata,
    output reg [31:0] outbus
    );
    always @(negedge clk) 
    begin
        if(regWrite == 1)
        begin
            outbus = writedata;
        end
    end

endmodule

module TopModule(
    input clk,
    input [5:0] opcode,
    input [5:0] funcfield,
    input overflow
    );

wire [31:0] seq,DT1out,DT2out,DT3out,DT4out;
reg [31:0] fetch = 32'd0;
reg [31:0] WBAdd = 32'd13;
//wire [2:0] addrCtrl;
wire [31:0] next_addr;
reg regWrite = 1;
reg [31:0] EPCUpdateAddr = 32'b00000000000000000000000000011100;
reg [31:0] adder_inputA = 32'd1;
wire [31:0] curr_addr;
wire [35:0] microInstruction;


Mux8to1 addressMUX(seq,fetch,WBAdd,DT1out,DT2out,DT3out,DT4out,EPCUpdateAddr,microInstruction[2:0],next_addr);  //addressMUX
Register32bit addressReg(clk,regWrite,next_addr,curr_addr);    //address register
Adder addressAdder(adder_inputA,curr_addr,seq);     //address Adder
MicroInstructionMem ControlMem(curr_addr,microInstruction);     //Micro Instruction Memory
DT1 DTable1(opcode,funcfield,DT1out);   //Dispatch Table 1
DT2 DTable2(opcode,funcfield,DT2out);   //Dispatch Table 2
DT3 DTable3(opcode,funcfield,DT3out);   //Dispatch Table 3
DT4 DTable4(opcode,funcfield,overflow,DT4out);   //Dispatch Table 4

endmodule
