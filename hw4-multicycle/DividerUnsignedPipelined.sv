/* Mark Xia 46332974 */
// /* verilator lint_off UNOPTFLAT */
`timescale 1ns / 1ns

// quotient = dividend / divisor
// Handles both signed and unsigned division
module divider(
    input wire clk, rst, stall,
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire sign,  // custom signal - 0 means unsigned, 1 means signed;
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
    parameter STAGES = 8;
    parameter SIZE = 32;
    typedef struct packed {
        logic sign;
        logic [SIZE-1:0] dividend_original;
        logic [SIZE-1:0] divisor_original;
    } sign_p;
    
    sign_p sign_pipeline[STAGES-1:0];

    // Stores the sign signal
    assign sign_pipeline[0].sign = sign;
    assign sign_pipeline[0].dividend_original = i_dividend;
    assign sign_pipeline[0].divisor_original = i_divisor;

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 1; i < STAGES; i++) begin
                sign_pipeline[i] <= '0; // Reset all except sign_p[0]
            end
        end
        else begin
            for (int k = 1; k < STAGES; k++) begin
                sign_pipeline[k] <= sign_pipeline[k-1];
            end
        end
    end
    // Intermediary storage
    logic [SIZE-1:0] abs_dividend, abs_divisor, o_quotient_u, o_remainder_u, o_divisor;
    // logic [31:0] o_divisor_original, o_dividend_original;
    // Assign values
    assign abs_dividend = sign ? (i_dividend[31] ? (~i_dividend + 1) : i_dividend) : i_dividend;
    assign abs_divisor = sign ? (i_divisor[31] ? (~i_divisor + 1) : i_divisor) : i_divisor;

    DividerUnsignedPipelined div_up(.clk(clk), .rst(rst), .stall(stall),
                            .i_dividend(abs_dividend), .i_divisor(abs_divisor),
                            .o_remainder(o_remainder_u), .o_quotient(o_quotient_u));
                            // .o_divisor_original(o_divisor_original), .o_dividend_original(o_dividend_original));

    assign o_quotient = ~(|sign_pipeline[STAGES-1].divisor_original) ? 32'hffffffff : sign_pipeline[STAGES-1].sign
                        ? ((sign_pipeline[STAGES-1].dividend_original[31] ^ sign_pipeline[STAGES-1].divisor_original[31])
                        ? (~o_quotient_u + 1) : o_quotient_u) : o_quotient_u;

    assign o_remainder = sign_pipeline[STAGES-1].sign ? ((sign_pipeline[STAGES-1].dividend_original[31])
                        ? (~o_remainder_u + 1) : o_remainder_u) : o_remainder_u;

endmodule

module DividerUnsignedPipelined (
    input wire clk, rst, stall,
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
    // output logic [31:0] o_divisor_original,
    // output logic [31:0] o_dividend_original
);

    // TODO: your code here
    parameter SIZE = 32;
    parameter STAGES = 8;
    // parameter ITER_PER_STAGE = SIZE / STAGES;
    // Assign outputs
    // These are ready after 7 cycles
    assign o_remainder = tmp[STAGES].remainder;
    assign o_quotient = tmp[STAGES].quotient;
    // assign o_divisor_original = tmp[STAGES].divisor;
    // assign o_dividend_original = pipeline[STAGES-1].dividend_p_original;

    // Define and instantiate packed pipeline registers
    typedef struct packed {
        logic [31:0] dividend_p;
        logic [31:0] remainder_p;
        logic [31:0] quotient_p;
        logic [31:0] divisor_p;
        // logic [31:0] dividend_p_original;
    } p_stages;

    p_stages pipeline[0:STAGES - 1];

    // assign pipeline[0].dividend_p_original = i_dividend;

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int k = 0; k < STAGES; k++) begin
                pipeline[k].dividend_p  <= 32'd0;
                pipeline[k].remainder_p <= 32'd0;
                pipeline[k].quotient_p  <= 32'd0;
                pipeline[k].divisor_p   <= 32'd0;
                // Do NOT reset pipeline[k].dividend_p_original
            end
        end
        else if (~stall) begin
            for (int k = 1; k < STAGES; k++) begin
                pipeline[k].dividend_p <= tmp[k].dividend;
                pipeline[k].remainder_p <= tmp[k].remainder;
                pipeline[k].quotient_p <= tmp[k].quotient;
                pipeline[k].divisor_p <= tmp[k].divisor;
                // pipeline[k].dividend_p_original <= pipeline[k-1].dividend_p_original;
            end
        end
    end

    // Define generate variables and instantiate 8 times of 4 iteration modules
    genvar i;
    
    typedef struct packed {
        logic [SIZE-1:0] dividend;
        logic [SIZE-1:0] remainder;
        logic [SIZE-1:0] quotient;     
        logic [SIZE-1:0] divisor;   
    } div_tmp;
    
    div_tmp tmp [0:STAGES];

    generate
        for(i = 0; i < STAGES; i++) begin
            divu_4iter inst_4(
                .i_dividend(i==0 ? i_dividend : pipeline[i].dividend_p),
                .i_divisor(i==0 ? i_divisor : pipeline[i].divisor_p),
                .i_remainder(i==0 ? 0 : pipeline[i].remainder_p),
                .i_quotient(i==0 ? 0 : pipeline[i].quotient_p),
                .o_dividend(tmp[i+1].dividend),
                .o_remainder(tmp[i+1].remainder),
                .o_quotient(tmp[i+1].quotient),
                .o_divisor(tmp[i+1].divisor)
            );
        end
    endgenerate

endmodule


module divu_4iter (
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    input  wire  [31:0] i_remainder,
    input  wire  [31:0] i_quotient,
    output logic [31:0] o_dividend,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient,
    output logic [31:0] o_divisor
);

    logic [31:0] dividend [0:4];
    logic [31:0] remainder [0:4];
    logic [31:0] quotient [0:4];
    assign dividend[0] = i_dividend;
    // Forgetting to assign these values will result in UNOPTFLAT Circular Combinational Logic in Verilator
    assign quotient[0] = i_quotient;
    assign remainder[0] = i_remainder;
    // --------------------------------
    assign o_dividend = dividend[4];
    assign o_quotient = quotient[4];
    assign o_remainder = remainder[4];
    assign o_divisor = i_divisor;
    genvar i;
    generate
        for(i = 0; i < 4; i++)begin
            divu_1iter inst(
                .i_dividend(dividend[i]),
                .i_divisor(i_divisor),
                .i_remainder(remainder[i]),
                .i_quotient(quotient[i]),
                .o_dividend(dividend[i+1]),
                .o_remainder(remainder[i+1]),
                .o_quotient(quotient[i+1])
            );
        end
    endgenerate

endmodule

module divu_1iter (
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    input  wire  [31:0] i_remainder,
    input  wire  [31:0] i_quotient,
    output logic [31:0] o_dividend,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);

    // TODO: copy your code from HW2A here
    // Intermediary variables to store shifted values
    logic [31:0] remainder_shift;
    // Calculate this iteration of shift
    assign remainder_shift = {i_remainder[30:0],1'b0} | {{31{1'b0}},{(i_dividend[31] & 1'b1)}};
    // Assign output values based on the algorithm
    assign o_remainder = (remainder_shift < i_divisor) ?  remainder_shift : remainder_shift - i_divisor;
    assign o_quotient = (remainder_shift < i_divisor) ? {i_quotient[30:0],1'b0} : ({i_quotient[30:0],1'b0} | 32'h00000001);  
    assign o_dividend = {i_dividend[30:0], 1'b0};

endmodule
