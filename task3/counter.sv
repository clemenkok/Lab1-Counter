module counter #(
    parameter WIDTH = 8
)(
    // interface signals
    input logic                    clk, //clock
    input logic                    rst, //reset
    input logic                    en,  //counter enable
    input logic [WIDTH-1:0]        value,
    output logic [WIDTH-1:0]       count //count output
);

always_ff @ (posedge clk)
    if (rst) count <= value;
    else     count <= count + {{WIDTH-1{1'b0}}, en};

endmodule
