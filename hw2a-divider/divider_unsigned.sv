/* Mark Xia 46332974 */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

// TODO: your code here
// Set the SIZE to be 32 bits so that we process 32 bits data
parameter SIZE = 32;

// Intermediary value in shape 33 * 32 bits to store values for every single 1iter inputs
logic [31:0] dividend [0:32];
logic [31:0] remainder [0:32];
logic [31:0] quotient [0:32];

// First 1iter dividend input is wrapper input
assign dividend[0] = i_dividend;
// The final result is stored in the last remainder element
assign o_remainder = remainder[32];
// Same goes for quotient
assign o_quotient = quotient[32];

// Initialize (zero) the remainder and quotient for the first 1iter module
assign remainder[0] = 0;
assign quotient[0] = 0;

// Generate variables for the for loop
genvar i;
generate
    for(i=0; i<SIZE; i++) begin: iter
        divu_1iter divu_inst(
            .i_dividend(dividend[i]),
            .i_divisor(i_divisor),
            .i_remainder(remainder[i]),
            .i_quotient(quotient[i]),
            .o_remainder(remainder[i+1]),
            .o_quotient(quotient[i+1]),
            .o_dividend(dividend[i+1])    
        );
    end
endgenerate

endmodule


module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

  /*
    for (int i = 0; i < 32; i++) {
        remainder = (remainder << 1) | ((dividend >> 31) & 0x1);
        if (remainder < divisor) {
            quotient = (quotient << 1);
        } else {
            quotient = (quotient << 1) | 0x1;
            remainder = remainder - divisor;
        }
        dividend = dividend << 1;
    }
    */

// TODO: your code here
// Intermediary variables to store shifted values
logic [31:0] dividend_shift, remainder_shift;
// Calculate this iteration of shift
assign remainder_shift = {i_remainder[30:0],1'b0} | {{31{1'b0}},{(i_dividend[31] & 1'b1)}};
// Assign output values based on the algorithm
assign o_remainder = (remainder_shift < i_divisor) ? remainder_shift : remainder_shift - i_divisor;
assign o_quotient = (remainder_shift < i_divisor) ? {i_quotient[30:0],1'b0} : ({i_quotient[30:0],1'b0} | 32'h00000001);  
assign o_dividend = {i_dividend[30:0], 1'b0};


endmodule
