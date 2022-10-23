module top #(
  parameter WIDTH = 8,
  parameter DIGITS = 3
)(
  // interface signals
  input  wire             clk,      // clock 
  input  wire             rst,      // reset 
  input  wire             en,       // enable
  input  wire [WIDTH-1:0] v,        // value to preload - what is this? I presume this is for the case if we want an immediate conversion to funnel into our BCD converter?  
  output wire [11:0]      bcd       // count output
);

  wire  [WIDTH-1:0]       count;    // interconnect wire

// NEED TO USE V input somehow. Also digits. multiplexing of input v signal and count to give the BCD module input but this has count in parantheses below so don't know if its correct. 

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (en),
  .count (count)
);

bin2bcd myDecoder (
  .x (count),
  .BCD (bcd)
);

endmodule
