module register(data_in, load, clk, data_out);
   parameter N = 16;

   input [N-1:0] data_in;
   input 	 load, clk;
   output reg [N-1:0] data_out;

   //Returns the input if load is enabled when there is a positive edge of clock
   //Else it returns the previous output
   always @(posedge clk) begin
      if (load)
	data_out <= data_in;
      else
	data_out <= data_out;
   end
endmodule // register
