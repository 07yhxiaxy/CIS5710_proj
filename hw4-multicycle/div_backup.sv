module DividerUnsignedPipelined (
    input wire clk, rst, stall,
    input  wire sign,  // custom signal - 0 means unsigned, 1 means signed;
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);

    // TODO: your code here
    parameter SIZE = 32;
    parameter STAGES = 8;
    // parameter ITER_PER_STAGE = SIZE / STAGES;
    logic [31:0] o_dividend_original, o_divisor;
    // Assign outputs
    assign o_remainder = tmp[STAGES].remainder;
    assign o_quotient = tmp[STAGES].quotient;
    assign o_divisor = pipeline[STAGES-1].divisor_p;
    assign o_dividend_original = pipeline[STAGES-1].dividend_p_original;


    // Define and instantiate packed pipeline registers
    typedef struct packed {
        logic [31:0] dividend_p;
        logic [31:0] remainder_p;
        logic [31:0] quotient_p;
        logic [31:0] divisor_p;
        logic [31:0] dividend_p_original;
    } p_stages;

    p_stages pipeline[0:STAGES - 1];

    always_ff @(posedge clk) begin
        if (rst) begin
            pipeline <= '{default: 0};
        end
        else if (~stall) begin
            for (int k = 1; k < STAGES; k++) begin
                pipeline[k].dividend_p <= tmp[k].dividend;
                pipeline[k].remainder_p <= tmp[k].remainder;
                pipeline[k].quotient_p <= tmp[k].quotient;
                pipeline[k].divisor_p <= tmp[k].divisor;
                pipeline[k].dividend_p_original <= pipeline[k-1].dividend_p_original;
            end
        end
    end

    assign pipeline[0].dividend_p_original = i_dividend;

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
