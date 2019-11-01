`define S0 4'b0000 //Reset
`define S1 4'b0001 //IF1
`define S2 4'b0010 //IF2
`define S3 4'b0011 //UpdatePC
`define S4 4'b0100 //Decode
`define S5 4'b0101 //Writelmm and GetA
`define S6 4'b0110 //GetB and adding A and sximm5
`define S7 4'b0111 //Operation and load address
`define S8 4'b1000 //WriteReg
`define S9 4'b1001 //Load into memeory
`define S10 4'b1111 //HaltState

`define MREAD 2'b01 //MREAD
`define MWRITE 2'b10 //MWRITE

module cpu(clk, reset, in, out, N, V, Z, mem_addr, mem_cmd);
   input clk, reset;
   input [15:0] in;
   output [15:0] out;
   output 	 N, V, Z;
   output [8:0]  mem_addr;
   output [1:0]  mem_cmd;
   wire [15:0] 	 instruction, read_data;
   wire [2:0] 	 opcode, register;
   wire [1:0] 	 op, ALUop, shift;
   wire 	 loadb, loada, asel, bsel, write, loads, loadc, only_shift;
   wire [1:0] 	 vsel;
   wire [15:0] 	 sximm5, sximm8;
   wire [2:0] 	 nsel;
   wire [8:0] 	 PC, next_pc;
   wire 	 reset_pc, load_pc, addr_sel, load_addr, load_ir;
   wire [8:0] 	 data_address;

   assign read_data = in;
   
   //Creating datapath
   datapath DP (.clk(clk), .readnum(register), .vsel(vsel), .loada(loada), .loadb(loadb), .shift(shift), .asel(asel), .bsel(bsel), .ALUop(ALUop), .loadc(loadc), .loads(loads), .writenum(register), .write(write), .N(N), .V(V), .Z(Z), .datapath_out(out), .sximm5(sximm5), .sximm8(sximm8), .mdata(read_data), .PC(PC[7:0]), .only_shift(only_shift));

   //Creating register that stores instructions
   register InstructionRegister (.data_in(in), .load(load_ir), .clk(clk), .data_out(instruction));

   //Creating program Counter register
   register #(9) ProgramCounter(.data_in(next_pc), .load(load_pc), .clk(clk), .data_out(PC));

   //Creating Data Address register
   register #(9) DataAddress(.data_in(out[8:0]), .load(load_addr), .clk(clk), .data_out(data_address));
   
   //Creates decoder for instructions to datapath and statemachine
   instructionDecoder InstructionDecoder (.instruction(instruction), .nsel(nsel), .op(op), .opcode(opcode), .register(register), .shift(shift), .sximm5(sximm5), .sximm8(sximm8), .ALUop(ALUop));

   //Statemachine for the cpu
   CPUstateMachine StateMachine(.clk(clk), .reset(reset), .opcode(opcode), .op(op), .vsel(vsel), .loadb(loadb), .loada(loada), .asel(asel), .write(write), .loadc(loadc), .loads(loads), .nsel(nsel), .only_shift(only_shift), .load_pc(load_pc), .reset_pc(reset_pc), .load_addr(load_addr), .addr_sel(addr_sel), .mem_cmd(mem_cmd), .load_ir(load_ir), .bsel(bsel));

   //Multiplexers that determine next_pc and mem_addr
   assign next_pc = reset_pc ? {9{1'b0}} : PC + 1;
   assign mem_addr = addr_sel ? PC : data_address;
   
endmodule // cpu

module instructionDecoder (instruction, nsel, op, opcode, register, shift, sximm5, sximm8, ALUop);
   input [15:0] instruction;
   input [2:0] nsel;
   output [2:0] opcode, register;
   output [1:0] op, shift, ALUop;
   output [15:0] sximm5, sximm8;
   wire [2:0] 	Rd, Rm, Rn;

   //Assigning the values of instructions to the corresponding wires
   assign opcode = instruction[15:13];
   assign op = instruction [12:11];
   assign ALUop = instruction[12:11];
   assign Rn = instruction [10:8];
   assign Rd = instruction [7:5];
   assign shift = instruction[4:3];
   assign Rm = instruction[2:0];
   assign sximm8 = {{8{instruction[7]}}, instruction[7:0]};
   assign sximm5 = {{11{instruction[4]}}, instruction[4:0]};

   //Assigning the register that is being interacted with
   assign register = nsel[2] ? Rm : (nsel[1] ? Rd : (nsel[0] ? Rn : {3{1'bx}}));
   
endmodule // instructionDecoder

module CPUstateMachine(reset, opcode, op, vsel, loadb, loada, asel, write, clk, loadc, loads, nsel, only_shift, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir, bsel);
   input reset, clk;
   input [2:0] opcode;
   input [1:0] op;
   output reg [1:0] vsel;
   output reg 	    loadb, loada, asel, bsel, write, loadc, loads, only_shift;
   output reg [2:0] nsel;
   output reg 	    load_pc, reset_pc, load_addr, addr_sel, load_ir;
   output reg [1:0] mem_cmd;
   wire [3:0] 	state;
   reg [3:0] 	nextstate;

   //Always block that controls the statemachine
   always @(posedge clk) begin
      if (reset) {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S0, 18'b000000000001101000};
      else begin
        casex ({state, opcode, op})
	  
	  //Goes to reset state if reset is pressed
	  {`S0, 5'bxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 16'b0000000000000001,`MREAD, 1'b0};
	  //Goes to IF1 state
	  {`S1, 5'bxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S2, 16'b0000000000000001, `MREAD, 1'b1};
	  //Goes to IF2 state
	  {`S2, 5'bxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S3, 19'b0000000000001001000};
	  //Goes to UpdatePC state
	  {`S3, 5'bxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S4, 19'b0000000000000001000};
	  //Goes to decoded state

	  //Goes to halt state
	  {{4{1'bx}}, 5'b111xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S10, 19'b0000000000000000000};
	  //Stays at halt state
	  {`S10, 5'bxxxxx} : nextstate <= `S10;
	  
	  //Loading register
	  //Decoded state
	  {`S4, 5'b1101x} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b100000010001};
	  //Load to regfile
	  {`S5, 5'b1101x} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 16'b0000000000000001,`MREAD, 1'b0};
	  
	  //Loading register with shifted value of other register
	  //Decoded state
	  {`S4, 5'b1100x} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S6, 12'b001000001100};
	  //Load B
	  {`S6, 5'b1100x} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S8, 12'b100000001010};
	  //Load regfile
	  {`S8, 5'b1100x} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 16'b0000000000000001,`MREAD, 1'b0};
	  
	  //Operation performed on loaded values
	  //Decoding state
	  {`S4, 5'b101xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b010000000001};
	  //Get A
	  {`S5, 5'b101xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S6, 12'b001000000100};
	  //Get B
	  {`S6, 5'b10101} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S8, 12'b000010000000};
	  //Operation if CMP
	  {`S6, 5'b101xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S7, 13'b0001000000000};
	  //Operation
	  {`S7, 5'b101xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S8, 13'b1000000000100};
	  //Loading result to regfile
	  {`S8, 5'b101xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 16'b0000000000000001,`MREAD, 1'b0};

	  //Load from memeory
	  //Decoded state
	  {`S4, 5'b011xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b010000000001};
	  //Get A
	  {`S5, 5'b011xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S6, 13'b0001000000001};
	  //Add A and sximm5
	  {`S6, 5'b011xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S7, 19'b0000000000000010000};
	  //Load address
	  {`S7, 5'b011xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S8, 16'b1000001100100000, `MREAD, 1'b0};
	  //Load from memory
	  {`S8, 5'b011xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S9, 16'b1000001100100000, `MREAD, 1'b0};
	  {`S9, 5'b011xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 16'b0000000000000001, `MREAD, 1'b0};

	  //Store in memory
	  //Decoded state
	  {`S4, 5'b100xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b010000000001};
	  //Get A
	  {`S5, 5'b100xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S6, 13'b0001000000001};
	  //Add A and sximm5
	  {`S6, 5'b100xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S7, 19'b0000000000000010000};
	  //Load address
	  {`S7, 5'b100xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S8, 19'b0010000010100000000};
	  //Get B
	  {`S8, 5'b100xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S9, 16'b0000000010000000, `MWRITE, 1'b0};
	  //Load into memory
	  {`S9, 5'b100xx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 16'b0000000000000001,`MREAD, 1'b0};
	  
	  default {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {23{1'bx}};
	  endcase
      end // else: !if(reset)
   end // always @ (posedge clk)
   assign state = nextstate;
endmodule // CPUstateMachine
