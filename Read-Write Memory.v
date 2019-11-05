//RAM code from the end of slide set 7
module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];  //dpesnt work with quartus

  initial $readmemb(filename, mem);

  always @ (negedge clk) begin   
    if (write)
      mem[write_address] = din; 
    dout = mem[read_address]; //Set dout using non-blocking assignment so it wont be updated until next clk cycle
  end 
endmodule
