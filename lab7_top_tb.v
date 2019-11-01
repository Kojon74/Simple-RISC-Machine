module lab7_top_tb;
  reg [3:0] KEY;
  reg [9:0] SW;
  wire [9:0] LEDR; 
  wire [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
  reg err;

  lab7_top #("test.txt") DUT(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);

  initial forever begin
    KEY[0] = 1; #5;
    KEY[0] = 0; #5;
  end

  initial begin
    err = 0;
    KEY[1] = 1'b0; // reset asserted

    #10; // wait until next falling edge of clock
    KEY[1] = 1'b1; // reset de-asserted, PC still undefined if as in Figure 4

    #10; // waiting for RST state to cause reset of PC

    // NOTE: your program counter register output should be called PC and be inside a module with instance name CPU
    if (DUT.CPU.PC !== 9'b0) begin err = 1; $display("FAILED: PC is not reset to zero."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 1 *before* executing MOV R0, X

    if (DUT.CPU.PC !== 9'h1) begin err = 1; $display("FAILED: PC should be 1."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 2 *after* executing MOV R0, X

    if (DUT.CPU.PC !== 9'h2) begin err = 1; $display("FAILED: PC should be 2."); $stop; end
    if (DUT.CPU.DP.REGFILE.R0 !== 16'h9) begin err = 1; $display("FAILED: R0 should be 9."); $stop; end  // because MOV R0, X should have occurred

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 3 *after* executing MOV R1, X

     if (DUT.CPU.PC !== 9'h3) begin err = 1; $display("FAILED: PC should be 3."); $stop; end

    if (DUT.CPU.DP.REGFILE.R1 !== 16'h5) begin err = 1; $display("FAILED: R1 should be 5. Looks like your MOV isn't working."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 4 *after* executing MOV R2, Y

     if (DUT.CPU.PC !== 9'h4) begin err = 1; $display("FAILED: PC should be 4."); $stop; end

    if (DUT.CPU.DP.REGFILE.R2 !== 16'hA) begin err = 1; $display("FAILED: R2 should be 10."); $stop; end

    @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 5 *after* executing ADD R3, R1, R0, LSL#1

     if (DUT.CPU.PC !== 9'h5) begin err = 1; $display("FAILED: PC should be 5."); $stop; end

    if (DUT.CPU.DP.REGFILE.R3 !== 16'h17) begin err = 1; $display("FAILED: R3 should be 23."); $stop; end

     @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 6 *after* executing CMP R2, R3

     if (DUT.CPU.PC !== 9'h6) begin err = 1; $display("FAILED: PC should be 6."); $stop; end

     if (DUT.CPU.DP.Z !== 1'b0) begin err = 1; $display("FAILED: Z should be 0."); $stop; end
     if (DUT.CPU.DP.V !== 1'b0) begin err = 1; $display("FAILED: V should be 0."); $stop; end
     if (DUT.CPU.DP.N !== 1'b1) begin err = 1; $display("FAILED: N should be 1."); $stop; end

     @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 7 *after* executing AND R4, R0, R1

     if (DUT.CPU.PC !== 9'h7) begin err = 1; $display("FAILED: PC should be 7."); $stop; end

    if (DUT.CPU.DP.REGFILE.R4 !== 16'h1) begin err = 1; $display("FAILED: R4 should be 1."); $stop; end

     @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 8 *after* executing MVN R0, R1

    if (DUT.CPU.DP.REGFILE.R0 !== 16'hFFFA) begin err = 1; $display("FAILED: R0 should be -6."); $stop; end

     @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 9 *after* executing LDR R1, [R1]

     if (DUT.CPU.DP.REGFILE.R1 !== 16'hB081) begin err = 1; $display("FAILED: R1 should be 45185."); $stop; end

     @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 10 *after* executing MOV R0, #2
     @(posedge DUT.CPU.PC or negedge DUT.CPU.PC);  // wait here until PC changes; autograder expects PC set to 11 *after* executing STR R2, [R0]
     
    if (DUT.CPU.PC !== 9'hB) begin err = 1; $display("FAILED: PC should be 11."); $stop; end
    if (DUT.MEM.mem[2] !== 16'hA) begin err = 1; $display("FAILED: mem[2] wrong; looks like your STR isn't working"); $stop; end

    // NOTE: if HALT is working, PC won't change again...

    if (~err) $display("IT WORKED!");
    $stop;
  end
endmodule
