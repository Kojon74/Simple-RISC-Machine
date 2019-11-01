module regfile(data_in, writenum, write, readnum, clk, data_out);
   input [15:0] data_in;
   input [2:0] 	writenum, readnum;
   input 	write, clk;
   output reg [15:0] data_out;
   wire [7:0] 	 decoded_writenum, writeto, decoded_readnum;
   wire [15:0] 	 R0, R1, R2, R3, R4, R5, R6, R7;

   //Decodes writenum into one hot
   decoder writenumdecoder (writenum, decoded_writenum);
   //Makes it 
   assign writeto = decoded_writenum & {8{write}};

   //Creating 8 load enabled registers, that update every positive edge of clk
   register Register0 (data_in, writeto[0], clk, R0);
   register Register1 (data_in, writeto[1], clk, R1);
   register Register2 (data_in, writeto[2], clk, R2);
   register Register3 (data_in, writeto[3], clk, R3);
   register Register4 (data_in, writeto[4], clk, R4);
   register Register5 (data_in, writeto[5], clk, R5);
   register Register6 (data_in, writeto[6], clk, R6);
   register Register7 (data_in, writeto[7], clk, R7);

   //Decodes readnum into one hot
   decoder readnumdecoder (readnum, decoded_readnum);
   //Assigns output to be equivilent to the register represented by readnum
   always @(*) begin
      case(decoded_readnum)
	8'b00000001: data_out = R0;
	8'b00000010: data_out = R1;
	8'b00000100: data_out = R2;
	8'b00001000: data_out = R3;
	8'b00010000: data_out = R4;
	8'b00100000: data_out = R5;
	8'b01000000: data_out = R6;
	8'b10000000: data_out = R7;
	default: data_out = {16{1'bx}};
      endcase // case (decoded_readnum)
   end
   
endmodule // regfile
module decoder (binary, one_hot);
   input [2:0] binary;
   output reg [7:0] one_hot;

   //Returns the one_hot value of the binary input
   always @(binary)begin
      case (binary)
	3'b000: one_hot = 8'b00000001;
	3'b001: one_hot = 8'b00000010;
	3'b010: one_hot = 8'b00000100;
	3'b011: one_hot = 8'b00001000;
	3'b100: one_hot = 8'b00010000;
	3'b101: one_hot = 8'b00100000;
	3'b110: one_hot = 8'b01000000;
	3'b111: one_hot = 8'b10000000;
	default: one_hot = {8{1'bx}};
      endcase // case (binary)
   end
endmodule // decoder
