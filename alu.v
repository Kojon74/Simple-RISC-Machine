module ALU(Ain, Bin, ALUop, out, Z);
   input [15:0] Ain, Bin;
   input [1:0] 	ALUop;
   output reg [15:0] out;
   output [2:0] Z;
   wire [15:0] 	negBin;

   assign negBin = ~(Bin - 1'b1);

   //If out is zero Z is 1 otherwise Z is 0
   assign Z[0] = ~| out;
   assign Z[1] = out[15];
   assign Z[2] = (out[15]^Ain[15]) & ~(Ain[15]^negBin[15]); 
 
   //Assigns out to be a logical operation of Ain and Bin based off ALUop
   always @(*) begin
      case (ALUop)
	2'b00 : out = Ain + Bin;
	2'b01 : out = Ain - Bin;
	2'b10 : out = Ain & Bin;
	2'b11 : out = ~Bin;
	default : out = {16{1'bx}};
      endcase // case (ALUop)
   end
endmodule // ALU
