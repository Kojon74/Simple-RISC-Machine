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

module cpu(clk, reset, in, out, mem_addr, mem_cmd);
   input clk, reset;
   input [15:0] in;
   output [15:0] out;
   output [8:0] mem_addr;
   output [1:0] mem_cmd;
   wire [15:0] instruction, read_data, sximm5, sximm8;
   wire [8:0] PC, next_pc, data_address;
   wire [2:0] opcode, register, nsel, con;
   wire [1:0] op, ALUop, shift, vsel, reset_pc;
   wire loadb, loada, asel, bsel, write, loads, loadc, only_shift, N, V, Z, load_pc, addr_sel, load_addr, load_ir;

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
   instructionDecoder InstructionDecoder (.instruction(instruction), .nsel(nsel), .op(op), .opcode(opcode), .register(register), .shift(shift), .sximm5(sximm5), .sximm8(sximm8), .ALUop(ALUop), .con(con));

   //Statemachine for the cpu
   CPUstateMachine StateMachine(.clk(clk), .reset(reset), .opcode(opcode), .op(op), .vsel(vsel), .loadb(loadb), .loada(loada), .asel(asel), .write(write), .loadc(loadc), .loads(loads), .nsel(nsel), .only_shift(only_shift), .load_pc(load_pc), .reset_pc(reset_pc), .load_addr(load_addr), .addr_sel(addr_sel), .mem_cmd(mem_cmd), .load_ir(load_ir), .bsel(bsel), .N(N), .V(V), .Z(Z), .con(con));

   //Multiplexers that determine next_pc and mem_addr
   assign next_pc = (reset_pc == 2'b11) ? {9{1'b0}} : {(reset_pc == 2'b00) ? {PC + 1} : {PC + 1 + sximm8}};
   assign mem_addr = addr_sel ? PC : data_address;
   
endmodule // cpu

module instructionDecoder (instruction, nsel, op, opcode, register, shift, sximm5, sximm8, ALUop, con);
   input [15:0] instruction;
   input [2:0] nsel;
   output [2:0] opcode, register, con;
   output [1:0] op, shift, ALUop;
   output [15:0] sximm5, sximm8;
   wire [2:0] Rd, Rm, Rn;

   //Assigning the values of instructions to the corresponding wires
   assign opcode = instruction[15:13];
   assign op = instruction [12:11];
   assign ALUop = instruction[12:11];
   assign Rn = instruction [10:8];
   assign con = instruction [10:8];
   assign Rd = instruction [7:5];
   assign shift = instruction[4:3];
   assign Rm = instruction[2:0];
   assign sximm8 = {{8{instruction[7]}}, instruction[7:0]};
   assign sximm5 = {{11{instruction[4]}}, instruction[4:0]};

   //Assigning the register that is being interacted with
   assign register = nsel[2] ? Rm : (nsel[1] ? Rd : (nsel[0] ? Rn : {3{1'bx}}));
   
endmodule // instructionDecoder

module CPUstateMachine(reset, opcode, op, vsel, loadb, loada, asel, write, clk, loadc, loads, nsel, only_shift, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir, bsel, N, V, Z, con);
   input [2:0] opcode, con;
   input [1:0] op;
   input reset, clk, N, V, Z;
   output reg [2:0] nsel;
   output reg [1:0] vsel, reset_pc, mem_cmd;
   output reg loadb, loada, asel, bsel, write, loadc, loads, only_shift, load_pc, load_addr, addr_sel, load_ir;
   wire [3:0] state;
   reg [3:0] nextstate;

   //Always block that controls the statemachine
   always @(posedge clk) begin
      if (reset) {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S0, 19'b0000_0000_0001_11010_00};
      else begin
        casex ({state, opcode, op, con})
	  //BLT
	  {{4{1'bx}}, 8'b00100011} : if (N != V) begin reset_pc <= 2'b10; end
				   else begin reset_pc <= 2'b11; end
	  //B
	  {{4{1'bx}}, 8'b00100000} : reset_pc <= 2'b10;

	  //BEQ
	  {{4{1'bx}}, 8'b00100001} : if (Z == 1) begin reset_pc <= 2'b10; end
				   else begin reset_pc <= 2'b11; end
	  //BNE
	  {{4{1'bx}}, 8'b00100010} : if (Z == 0) begin reset_pc <= 2'b10; end
				   else begin reset_pc <= 2'b11; end
	  //BLE
	  {{4{1'bx}}, 8'b00100100} : if (N != V | Z == 1) begin reset_pc <= 2'b10; end
				   else begin reset_pc <= 2'b11; end

	  //Goes to reset state if reset is pressed
	  {`S0, 8'bxxxxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 17'b0000000000000_0001,`MREAD, 1'b0};
	  //Goes to IF1 state
	  {`S1, 8'bxxxxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S2, 17'b0000000000000_0001, `MREAD, 1'b1};
	  //Goes to IF2 state
	  {`S2, 8'bxxxxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S3, 20'b0000000000001_0001000};
	  //Goes to UpdatePC state
	  {`S3, 8'bxxxxxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S4, 20'b0000000000000_0001000};
	  //Goes to decoded state

	  //Goes to halt state
	  {{4{1'bx}}, 8'b111xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S10, 20'b0000000000000_0000000};
	  //Stays at halt state
	  {`S10, 8'bxxxxxxxx} : nextstate <= `S10;
	  
	  //Loading register
	  //Decoded state
	  {`S4, 8'b1101xxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b100000_010_001};
	  //Load to regfile
	  {`S5, 8'b1101xxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 17'b0000000000000_0001,`MREAD, 1'b0};
	  
	  //Loading register with shifted value of other register
	  //Decoded state
	  {`S4, 8'b1100xxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S6, 12'b001000001100};
	  //Load B
	  {`S6, 8'b1100xxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S8, 12'b100000001010};
	  //Load regfile
	  {`S8, 8'b1100xxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 17'b0000000000000_0001,`MREAD, 1'b0};
	  
	  //Operation performed on loaded values
	  //Decoding state
	  {`S4, 8'b101xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b010000000001};
	  //Get A
	  {`S5, 8'b101xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S6, 12'b001000000100};
	  //Get B
	  {`S6, 8'b10101xxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S8, 12'b000010000000};
	  //Operation if CMP
	  {`S6, 8'b101xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S7, 13'b0001000000000};
	  //Operation
	  {`S7, 8'b101xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S8, 13'b1000000000100};
	  //Loading result to regfile
	  {`S8, 8'b101xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 17'b0000000000000_0001,`MREAD, 1'b0};

	  //Load from memeory
	  //Decoded state
	  {`S4, 8'b011xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b010000000001};
	  //Get A
	  {`S5, 8'b011xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S6, 13'b0001000000001};
	  //Add A and sximm5
	  {`S6, 8'b011xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S7, 20'b0000000000000_0010000};
	  //Load address
	  {`S7, 8'b011xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S8, 17'b1000001100100_0000, `MREAD, 1'b0};
	  //Load from memory
	  {`S8, 8'b011xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S9, 17'b1000001100100_0000, `MREAD, 1'b0};
	  {`S9, 8'b011xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 17'b0000000000000_0001, `MREAD, 1'b0};

	  //Store in memory
	  //Decoded state
	  {`S4, 8'b100xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel} <= {`S5, 12'b010000000001};
	  //Get A
	  {`S5, 8'b100xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, bsel} <= {`S6, 13'b0001000000001};
	  //Add A and sximm5
	  {`S6, 8'b100xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S7, 20'b0000000000000_0010000};
	  //Load address
	  {`S7, 8'b100xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S8, 20'b0010000010100_0000000};
	  //Get B
	  {`S8, 8'b100xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S9, 17'b0000000010000_0000, `MWRITE, 1'b0};
	  //Load into memory
	  {`S9, 8'b100xxxxx} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {`S1, 17'b0000000000000_0001,`MREAD, 1'b0};
	  

	  //Direct call
	  //Decode state
	  {`S4, 5'b01011} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc} <= {`S1, 13'b000000100010}; //Not sure if load_pc and reset_pc should be set in this state or next
	  //{`S7, 5'b01011} : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <=

	  //Return
	  default : {nextstate, write, loada, loadb, loadc, loads, asel, vsel, only_shift, nsel, load_pc, reset_pc, load_addr, addr_sel, mem_cmd, load_ir} <= {24{1'bx}};
	  endcase
      end // else: !if(reset)
   end // always @ (posedge clk)
   assign state = nextstate;
endmodule // CPUstateMachine
