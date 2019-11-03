module datapath (clk, readnum, vsel, loada, loadb, shift, asel, bsel, ALUop, loadc, loads, writenum, write, N, V, Z, datapath_out, sximm5, sximm8, mdata, PC, only_shift);
   wire [15:0] 	data_in, data_out, in, sout, Ain, Bin, out, aout, out_in, cout;
   wire [2:0] Z_in;
   input [2:0] 	writenum, readnum;
   input [1:0] shift, ALUop;
   input       write, clk, loada, loadb, loadc, loads, asel, bsel, only_shift;
   input [1:0] vsel;
   input [15:0] sximm5, sximm8;
   output [15:0] datapath_out;
   output N, V, Z;
   input [15:0] 	 mdata;
   input [7:0] 	 PC;
   wire [2:0] Z_out;

   assign V = Z_out[2];
   assign N = Z_out[1];
   assign Z = Z_out[0];
   
   //Instantiating the register file
   regfile REGFILE (data_in, writenum, write, readnum, clk, data_out);
   //Instantiating a shifter
   shifter U1(in, shift, sout);
   //Instantiating an Arithmetic Logic Unit
   ALU U2 (Ain, Bin, ALUop, out, Z_in);

   //Instantiating the load enabled registers
   register A (data_out, loada, clk, aout);
   register B (data_out, loadb, clk, in);
   register C (out, loadc, clk, cout);
   register #(3) status (Z_in, loads, clk, Z_out);

   //Instantiating the multiplexers
   assign data_in = vsel[0] ? (vsel[1] ? mdata : sximm8) : (vsel[1] ? {8'b0, PC} : out_in);
   assign Ain = asel ? 16'b0 : aout;
   assign Bin = bsel ? sximm5 : sout;
   assign out_in = only_shift ? sout : datapath_out;
   assign datapath_out = only_shift ? sout : cout;
   
endmodule // datapath
