module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	clk_mem,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire clk_mem;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "20" *) (* FREQUENCY_PIN_CLKOS = "20" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
		.PLLRST_ENA("DISABLED"),
		.INTFB_WAKE("DISABLED"),
		.STDBY_ENABLE("DISABLED"),
		.DPHASE_SOURCE("DISABLED"),
		.OUTDIVIDER_MUXA("DIVA"),
		.OUTDIVIDER_MUXB("DIVB"),
		.OUTDIVIDER_MUXC("DIVC"),
		.OUTDIVIDER_MUXD("DIVD"),
		.CLKI_DIV(5),
		.CLKOP_ENABLE("ENABLED"),
		.CLKOP_DIV(30),
		.CLKOP_CPHASE(15),
		.CLKOP_FPHASE(0),
		.CLKOS_ENABLE("ENABLED"),
		.CLKOS_DIV(30),
		.CLKOS_CPHASE(22),
		.CLKOS_FPHASE(4),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(4)
	) pll_i(
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(input_clk_25MHz),
		.CLKOP(clk_proc),
		.CLKOS(clk_mem),
		.CLKFB(clkfb),
		.CLKINTFB(clkfb),
		.PHASESEL0(1'b0),
		.PHASESEL1(1'b0),
		.PHASEDIR(1'b1),
		.PHASESTEP(1'b1),
		.PHASELOADREG(1'b1),
		.PLLWAKESYNC(1'b0),
		.ENCLKOP(1'b0),
		.LOCK(locked)
	);
endmodule
module gp1 (
	a,
	b,
	g,
	p
);
	input wire a;
	input wire b;
	output wire g;
	output wire p;
	assign g = a & b;
	assign p = a | b;
endmodule
module gp4 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [3:0] gin;
	input wire [3:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [2:0] cout;
	assign cout[0] = (pin[0] & cin) | gin[0];
	assign cout[1] = (((pin[0] & pin[1]) & cin) | (gin[0] & pin[1])) | gin[1];
	assign cout[2] = (((((pin[0] & pin[1]) & pin[2]) & cin) | ((gin[0] & pin[1]) & pin[2])) | (pin[2] & gin[1])) | gin[2];
	assign gout = ((gin[3] | (pin[3] & gin[2])) | ((pin[3] & pin[2]) & gin[1])) | (((pin[3] & pin[2]) & pin[1]) & gin[0]);
	assign pout = ((pin[3] & pin[2]) & pin[1]) & pin[0];
endmodule
module gp8 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [7:0] gin;
	input wire [7:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [6:0] cout;
	wire [2:0] cout_tmp_low;
	wire [2:0] cout_tmp_high;
	wire gout_tmp;
	wire pout_tmp;
	wire gout_tmp1;
	wire pout_tmp1;
	wire carry;
	assign carry = gout_tmp | (pout_tmp & cin);
	gp4 gp4_low(
		.gin(gin[3:0]),
		.pin(pin[3:0]),
		.cin(cin),
		.gout(gout_tmp),
		.pout(pout_tmp),
		.cout(cout_tmp_low[2:0])
	);
	gp4 gp4_high(
		.gin(gin[7:4]),
		.pin(pin[7:4]),
		.cin(carry),
		.gout(gout_tmp1),
		.pout(pout_tmp1),
		.cout(cout_tmp_high[2:0])
	);
	assign cout = {cout_tmp_high, carry, cout_tmp_low};
	assign gout = gout_tmp1 | (gout_tmp & pout_tmp1);
	assign pout = pout_tmp & pout_tmp1;
endmodule
module cla (
	a,
	b,
	cin,
	sum
);
	input wire [31:0] a;
	input wire [31:0] b;
	input wire cin;
	output wire [31:0] sum;
	wire [31:0] gtemp;
	wire [31:0] ptemp;
	wire [6:0] cout_temp [3:0];
	wire carry1;
	wire carry2;
	wire carry3;
	wire carry4;
	wire [3:0] gout_temp;
	wire [3:0] pout_temp;
	genvar _gv_i_1;
	generate
		for (_gv_i_1 = 0; _gv_i_1 < 32; _gv_i_1 = _gv_i_1 + 1) begin : gp1_block
			localparam i = _gv_i_1;
			gp1 a0(
				.a(a[i]),
				.b(b[i]),
				.g(gtemp[i]),
				.p(ptemp[i])
			);
		end
	endgenerate
	gp8 a1(
		.gin(gtemp[7:0]),
		.pin(ptemp[7:0]),
		.cin(cin),
		.gout(gout_temp[0]),
		.pout(pout_temp[0]),
		.cout(cout_temp[0])
	);
	gp8 a2(
		.gin(gtemp[15:8]),
		.pin(ptemp[15:8]),
		.cin(carry1),
		.gout(gout_temp[1]),
		.pout(pout_temp[1]),
		.cout(cout_temp[1])
	);
	gp8 a3(
		.gin(gtemp[23:16]),
		.pin(ptemp[23:16]),
		.cin(carry2),
		.gout(gout_temp[2]),
		.pout(pout_temp[2]),
		.cout(cout_temp[2])
	);
	gp8 a4(
		.gin(gtemp[31:24]),
		.pin(ptemp[31:24]),
		.cin(carry3),
		.gout(gout_temp[3]),
		.pout(pout_temp[3]),
		.cout(cout_temp[3])
	);
	assign carry1 = gout_temp[0] | (pout_temp[0] & cin);
	assign carry2 = gout_temp[1] | (pout_temp[1] & carry1);
	assign carry3 = gout_temp[2] | (pout_temp[2] & carry2);
	assign carry4 = gout_temp[3] | (pout_temp[3] & carry3);
	assign sum[0] = (cin ^ a[0]) ^ b[0];
	genvar _gv_j_1;
	generate
		for (_gv_j_1 = 1; _gv_j_1 < 8; _gv_j_1 = _gv_j_1 + 1) begin : sum1_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[0][j - 1] ^ a[j]) ^ b[j];
		end
	endgenerate
	assign sum[8] = (carry1 ^ a[8]) ^ b[8];
	generate
		for (_gv_j_1 = 9; _gv_j_1 < 16; _gv_j_1 = _gv_j_1 + 1) begin : sum2_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[1][j - 9] ^ a[j]) ^ b[j];
		end
	endgenerate
	assign sum[16] = (carry2 ^ a[16]) ^ b[16];
	generate
		for (_gv_j_1 = 17; _gv_j_1 < 24; _gv_j_1 = _gv_j_1 + 1) begin : sum3_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[2][j - 17] ^ a[j]) ^ b[j];
		end
	endgenerate
	assign sum[24] = (carry3 ^ a[24]) ^ b[24];
	generate
		for (_gv_j_1 = 25; _gv_j_1 < 32; _gv_j_1 = _gv_j_1 + 1) begin : sum4_block
			localparam j = _gv_j_1;
			assign sum[j] = (cout_temp[3][j - 25] ^ a[j]) ^ b[j];
		end
	endgenerate
endmodule
module divider (
	clk,
	rst,
	stall,
	i_dividend,
	i_divisor,
	sign,
	o_remainder,
	o_quotient
);
	input wire clk;
	input wire rst;
	input wire stall;
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	input wire sign;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	parameter STAGES = 8;
	parameter SIZE = 32;
	reg [((1 + SIZE) + SIZE) - 1:0] sign_pipeline [STAGES - 1:0];
	wire [1:1] sv2v_tmp_E3878;
	assign sv2v_tmp_E3878 = sign;
	always @(*) sign_pipeline[0][1 + (SIZE + (SIZE - 1))] = sv2v_tmp_E3878;
	wire [((SIZE + (SIZE - 1)) >= (SIZE + 0) ? ((SIZE + (SIZE - 1)) - (SIZE + 0)) + 1 : ((SIZE + 0) - (SIZE + (SIZE - 1))) + 1) * 1:1] sv2v_tmp_C7590;
	assign sv2v_tmp_C7590 = i_dividend;
	always @(*) sign_pipeline[0][SIZE + (SIZE - 1)-:((SIZE + (SIZE - 1)) >= (SIZE + 0) ? ((SIZE + (SIZE - 1)) - (SIZE + 0)) + 1 : ((SIZE + 0) - (SIZE + (SIZE - 1))) + 1)] = sv2v_tmp_C7590;
	wire [SIZE * 1:1] sv2v_tmp_98CDB;
	assign sv2v_tmp_98CDB = i_divisor;
	always @(*) sign_pipeline[0][SIZE - 1-:SIZE] = sv2v_tmp_98CDB;
	always @(posedge clk)
		if (rst) begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 1; i < STAGES; i = i + 1)
				sign_pipeline[i] <= 1'sb0;
		end
		else begin : sv2v_autoblock_2
			reg signed [31:0] k;
			for (k = 1; k < STAGES; k = k + 1)
				sign_pipeline[k] <= sign_pipeline[k - 1];
		end
	wire [SIZE - 1:0] abs_dividend;
	wire [SIZE - 1:0] abs_divisor;
	wire [SIZE - 1:0] o_quotient_u;
	wire [SIZE - 1:0] o_remainder_u;
	wire [SIZE - 1:0] o_divisor;
	assign abs_dividend = (sign ? (i_dividend[31] ? ~i_dividend + 1 : i_dividend) : i_dividend);
	assign abs_divisor = (sign ? (i_divisor[31] ? ~i_divisor + 1 : i_divisor) : i_divisor);
	DividerUnsignedPipelined div_up(
		.clk(clk),
		.rst(rst),
		.stall(stall),
		.i_dividend(abs_dividend),
		.i_divisor(abs_divisor),
		.o_remainder(o_remainder_u),
		.o_quotient(o_quotient_u)
	);
	assign o_quotient = (~(|sign_pipeline[STAGES - 1][SIZE - 1-:SIZE]) ? 32'hffffffff : (sign_pipeline[STAGES - 1][1 + (SIZE + (SIZE - 1))] ? (sign_pipeline[STAGES - 1][(SIZE + (SIZE - 1)) - (SIZE - 32)] ^ sign_pipeline[STAGES - 1][(SIZE - 1) - (SIZE - 32)] ? ~o_quotient_u + 1 : o_quotient_u) : o_quotient_u));
	assign o_remainder = (sign_pipeline[STAGES - 1][1 + (SIZE + (SIZE - 1))] ? (sign_pipeline[STAGES - 1][(SIZE + (SIZE - 1)) - (SIZE - 32)] ? ~o_remainder_u + 1 : o_remainder_u) : o_remainder_u);
endmodule
module DividerUnsignedPipelined (
	clk,
	rst,
	stall,
	i_dividend,
	i_divisor,
	o_remainder,
	o_quotient
);
	input wire clk;
	input wire rst;
	input wire stall;
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	parameter SIZE = 32;
	parameter STAGES = 8;
	wire [(((SIZE + SIZE) + SIZE) + SIZE) - 1:0] tmp [0:STAGES];
	assign o_remainder = tmp[STAGES][SIZE + (SIZE + (SIZE - 1))-:((SIZE + (SIZE + (SIZE - 1))) >= (SIZE + (SIZE + 0)) ? ((SIZE + (SIZE + (SIZE - 1))) - (SIZE + (SIZE + 0))) + 1 : ((SIZE + (SIZE + 0)) - (SIZE + (SIZE + (SIZE - 1)))) + 1)];
	assign o_quotient = tmp[STAGES][SIZE + (SIZE - 1)-:((SIZE + (SIZE - 1)) >= (SIZE + 0) ? ((SIZE + (SIZE - 1)) - (SIZE + 0)) + 1 : ((SIZE + 0) - (SIZE + (SIZE - 1))) + 1)];
	reg [127:0] pipeline [0:STAGES - 1];
	always @(posedge clk)
		if (rst) begin : sv2v_autoblock_1
			reg signed [31:0] k;
			for (k = 0; k < STAGES; k = k + 1)
				begin
					pipeline[k][127-:32] <= 32'd0;
					pipeline[k][95-:32] <= 32'd0;
					pipeline[k][63-:32] <= 32'd0;
					pipeline[k][31-:32] <= 32'd0;
				end
		end
		else if (~stall) begin : sv2v_autoblock_2
			reg signed [31:0] k;
			for (k = 1; k < STAGES; k = k + 1)
				begin
					pipeline[k][127-:32] <= tmp[k][SIZE + (SIZE + (SIZE + (SIZE - 1)))-:((SIZE + (SIZE + (SIZE + (SIZE - 1)))) >= (SIZE + (SIZE + (SIZE + 0))) ? ((SIZE + (SIZE + (SIZE + (SIZE - 1)))) - (SIZE + (SIZE + (SIZE + 0)))) + 1 : ((SIZE + (SIZE + (SIZE + 0))) - (SIZE + (SIZE + (SIZE + (SIZE - 1))))) + 1)];
					pipeline[k][95-:32] <= tmp[k][SIZE + (SIZE + (SIZE - 1))-:((SIZE + (SIZE + (SIZE - 1))) >= (SIZE + (SIZE + 0)) ? ((SIZE + (SIZE + (SIZE - 1))) - (SIZE + (SIZE + 0))) + 1 : ((SIZE + (SIZE + 0)) - (SIZE + (SIZE + (SIZE - 1)))) + 1)];
					pipeline[k][63-:32] <= tmp[k][SIZE + (SIZE - 1)-:((SIZE + (SIZE - 1)) >= (SIZE + 0) ? ((SIZE + (SIZE - 1)) - (SIZE + 0)) + 1 : ((SIZE + 0) - (SIZE + (SIZE - 1))) + 1)];
					pipeline[k][31-:32] <= tmp[k][SIZE - 1-:SIZE];
				end
		end
	genvar _gv_i_2;
	generate
		for (_gv_i_2 = 0; _gv_i_2 < STAGES; _gv_i_2 = _gv_i_2 + 1) begin : genblk1
			localparam i = _gv_i_2;
			divu_4iter inst_4(
				.i_dividend((i == 0 ? i_dividend : pipeline[i][127-:32])),
				.i_divisor((i == 0 ? i_divisor : pipeline[i][31-:32])),
				.i_remainder((i == 0 ? 0 : pipeline[i][95-:32])),
				.i_quotient((i == 0 ? 0 : pipeline[i][63-:32])),
				.o_dividend(tmp[i + 1][SIZE + (SIZE + (SIZE + (SIZE - 1)))-:((SIZE + (SIZE + (SIZE + (SIZE - 1)))) >= (SIZE + (SIZE + (SIZE + 0))) ? ((SIZE + (SIZE + (SIZE + (SIZE - 1)))) - (SIZE + (SIZE + (SIZE + 0)))) + 1 : ((SIZE + (SIZE + (SIZE + 0))) - (SIZE + (SIZE + (SIZE + (SIZE - 1))))) + 1)]),
				.o_remainder(tmp[i + 1][SIZE + (SIZE + (SIZE - 1))-:((SIZE + (SIZE + (SIZE - 1))) >= (SIZE + (SIZE + 0)) ? ((SIZE + (SIZE + (SIZE - 1))) - (SIZE + (SIZE + 0))) + 1 : ((SIZE + (SIZE + 0)) - (SIZE + (SIZE + (SIZE - 1)))) + 1)]),
				.o_quotient(tmp[i + 1][SIZE + (SIZE - 1)-:((SIZE + (SIZE - 1)) >= (SIZE + 0) ? ((SIZE + (SIZE - 1)) - (SIZE + 0)) + 1 : ((SIZE + 0) - (SIZE + (SIZE - 1))) + 1)]),
				.o_divisor(tmp[i + 1][SIZE - 1-:SIZE])
			);
		end
	endgenerate
endmodule
module divu_4iter (
	i_dividend,
	i_divisor,
	i_remainder,
	i_quotient,
	o_dividend,
	o_remainder,
	o_quotient,
	o_divisor
);
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	input wire [31:0] i_remainder;
	input wire [31:0] i_quotient;
	output wire [31:0] o_dividend;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	output wire [31:0] o_divisor;
	wire [31:0] dividend [0:4];
	wire [31:0] remainder [0:4];
	wire [31:0] quotient [0:4];
	assign dividend[0] = i_dividend;
	assign quotient[0] = i_quotient;
	assign remainder[0] = i_remainder;
	assign o_dividend = dividend[4];
	assign o_quotient = quotient[4];
	assign o_remainder = remainder[4];
	assign o_divisor = i_divisor;
	genvar _gv_i_3;
	generate
		for (_gv_i_3 = 0; _gv_i_3 < 4; _gv_i_3 = _gv_i_3 + 1) begin : genblk1
			localparam i = _gv_i_3;
			divu_1iter inst(
				.i_dividend(dividend[i]),
				.i_divisor(i_divisor),
				.i_remainder(remainder[i]),
				.i_quotient(quotient[i]),
				.o_dividend(dividend[i + 1]),
				.o_remainder(remainder[i + 1]),
				.o_quotient(quotient[i + 1])
			);
		end
	endgenerate
endmodule
module divu_1iter (
	i_dividend,
	i_divisor,
	i_remainder,
	i_quotient,
	o_dividend,
	o_remainder,
	o_quotient
);
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	input wire [31:0] i_remainder;
	input wire [31:0] i_quotient;
	output wire [31:0] o_dividend;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	wire [31:0] remainder_shift;
	assign remainder_shift = {i_remainder[30:0], 1'b0} | {{31 {1'b0}}, i_dividend[31] & 1'b1};
	assign o_remainder = (remainder_shift < i_divisor ? remainder_shift : remainder_shift - i_divisor);
	assign o_quotient = (remainder_shift < i_divisor ? {i_quotient[30:0], 1'b0} : {i_quotient[30:0], 1'b0} | 32'h00000001);
	assign o_dividend = {i_dividend[30:0], 1'b0};
endmodule
module RegFile (
	rd,
	rd_data,
	rs1,
	rs1_data,
	rs2,
	rs2_data,
	clk,
	we,
	rst
);
	input wire [4:0] rd;
	input wire [31:0] rd_data;
	input wire [4:0] rs1;
	output wire [31:0] rs1_data;
	input wire [4:0] rs2;
	output wire [31:0] rs2_data;
	input wire clk;
	input wire we;
	input wire rst;
	localparam signed [31:0] NumRegs = 32;
	reg [31:0] regs [0:31];
	always @(posedge clk)
		if (rst) begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < NumRegs; i = i + 1)
				regs[i] <= 1'sb0;
		end
		else if (we && (rd != 0))
			regs[rd] <= rd_data;
	assign rs1_data = regs[rs1];
	assign rs2_data = regs[rs2];
endmodule
module DatapathMultiCycle (
	clk,
	rst,
	halt,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	input wire clk;
	input wire rst;
	output reg halt;
	output wire [31:0] pc_to_imem;
	input wire [31:0] insn_from_imem;
	output reg [31:0] addr_to_dmem;
	input wire [31:0] load_data_from_dmem;
	output reg [31:0] store_data_to_dmem;
	output reg [3:0] store_we_to_dmem;
	parameter STAGES = 8;
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;
	wire [11:0] imm_i;
	assign imm_i = insn_from_imem[31:20];
	wire [4:0] imm_shamt = insn_from_imem[24:20];
	wire [11:0] imm_s;
	assign imm_s[11:5] = insn_funct7;
	assign imm_s[4:0] = insn_rd;
	wire [12:0] imm_b;
	assign {imm_b[12], imm_b[10:5]} = insn_funct7;
	assign {imm_b[4:1], imm_b[11]} = insn_rd;
	assign imm_b[0] = 1'b0;
	wire [20:0] imm_j;
	assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};
	wire [31:0] imm_i_sext = {{20 {imm_i[11]}}, imm_i[11:0]};
	wire [31:0] imm_s_sext = {{20 {imm_s[11]}}, imm_s[11:0]};
	wire [31:0] imm_b_sext = {{19 {imm_b[12]}}, imm_b[12:0]};
	wire [31:0] imm_j_sext = {{11 {imm_j[20]}}, imm_j[20:0]};
	localparam [6:0] OpLoad = 7'b0000011;
	localparam [6:0] OpStore = 7'b0100011;
	localparam [6:0] OpBranch = 7'b1100011;
	localparam [6:0] OpJalr = 7'b1100111;
	localparam [6:0] OpMiscMem = 7'b0001111;
	localparam [6:0] OpJal = 7'b1101111;
	localparam [6:0] OpRegImm = 7'b0010011;
	localparam [6:0] OpRegReg = 7'b0110011;
	localparam [6:0] OpEnviron = 7'b1110011;
	localparam [6:0] OpAuipc = 7'b0010111;
	localparam [6:0] OpLui = 7'b0110111;
	wire insn_lui = insn_opcode == OpLui;
	wire insn_auipc = insn_opcode == OpAuipc;
	wire insn_jal = insn_opcode == OpJal;
	wire insn_jalr = insn_opcode == OpJalr;
	wire insn_beq = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b000);
	wire insn_bne = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b001);
	wire insn_blt = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b100);
	wire insn_bge = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b101);
	wire insn_bltu = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b110);
	wire insn_bgeu = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b111);
	wire insn_lb = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b000);
	wire insn_lh = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b001);
	wire insn_lw = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b010);
	wire insn_lbu = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b100);
	wire insn_lhu = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b101);
	wire insn_sb = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b000);
	wire insn_sh = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b001);
	wire insn_sw = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b010);
	wire insn_addi = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b000);
	wire insn_slti = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b010);
	wire insn_sltiu = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b011);
	wire insn_xori = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b100);
	wire insn_ori = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b110);
	wire insn_andi = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b111);
	wire insn_slli = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b001)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srli = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srai = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_add = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b000)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sub = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b000)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_sll = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b001)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_slt = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b010)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sltu = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b011)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_xor = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b100)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srl = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sra = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_or = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b110)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_and = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b111)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_mul = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b000);
	wire insn_mulh = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b001);
	wire insn_mulhsu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b010);
	wire insn_mulhu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b011);
	wire insn_div = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b100);
	wire insn_divu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b101);
	wire insn_rem = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b110);
	wire insn_remu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b111);
	wire insn_ecall = (insn_opcode == OpEnviron) && (insn_from_imem[31:7] == 25'd0);
	wire insn_fence = insn_opcode == OpMiscMem;
	reg [31:0] pcNext;
	reg [31:0] pcCurrent;
	always @(posedge clk)
		if (rst)
			pcCurrent <= 32'd0;
		else
			pcCurrent <= pcNext;
	assign pc_to_imem = pcCurrent;
	reg [31:0] cycles_current;
	reg [31:0] num_insns_current;
	always @(posedge clk)
		if (rst) begin
			cycles_current <= 0;
			num_insns_current <= 0;
		end
		else begin
			cycles_current <= cycles_current + 1;
			if (!rst)
				num_insns_current <= num_insns_current + 1;
		end
	wire [31:0] rs1_data;
	wire [31:0] rs2_data;
	reg [31:0] rd_data;
	reg we;
	RegFile rf(
		.clk(clk),
		.rst(rst),
		.we(we),
		.rd(insn_rd),
		.rd_data(rd_data),
		.rs1(insn_rs1),
		.rs2(insn_rs2),
		.rs1_data(rs1_data),
		.rs2_data(rs2_data)
	);
	reg illegal_insn;
	reg [31:0] a;
	reg [31:0] b;
	wire [31:0] sum;
	reg [31:0] a1;
	reg [31:0] b1;
	wire [31:0] sum1;
	reg [31:0] addr_to_dmem_tmp;
	reg cin;
	reg cin1;
	cla cla1(
		.a(a),
		.b(b),
		.cin(cin),
		.sum(sum)
	);
	cla cla2(
		.a(a1),
		.b(b1),
		.cin(cin1),
		.sum(sum1)
	);
	reg carry;
	wire carry1;
	reg [63:0] mul_tmp;
	reg [31:0] i_dividend;
	reg [31:0] i_divisor;
	wire [31:0] o_remainder;
	wire [31:0] o_quotient;
	reg sign;
	wire stall;
	divider div1(
		.clk(clk),
		.rst(rst),
		.i_dividend(i_dividend),
		.i_divisor(i_divisor),
		.o_remainder(o_remainder),
		.o_quotient(o_quotient),
		.sign(sign),
		.stall(stall)
	);
	reg [$clog2(STAGES) - 1:0] counter;
	reg rdy;
	wire we_rdy;
	always @(posedge clk)
		if (rst)
			counter <= 1'sb0;
		else if (rdy)
			counter <= counter + 1;
	assign we_rdy = &counter;
	always @(*) begin
		if (_sv2v_0)
			;
		illegal_insn = 1'b0;
		pcNext = pcCurrent + 32'd4;
		rd_data = 32'b00000000000000000000000000000000;
		we = 0;
		halt = 0;
		a = 0;
		b = 0;
		cin = 0;
		a1 = 0;
		b1 = 0;
		cin1 = 0;
		carry = 0;
		sign = 0;
		mul_tmp = 0;
		i_dividend = 0;
		i_divisor = 32'd0;
		addr_to_dmem_tmp = 0;
		addr_to_dmem = 0;
		store_we_to_dmem = 0;
		store_data_to_dmem = 0;
		rdy = 0;
		case (insn_opcode)
			OpLui: begin
				rd_data = {insn_from_imem[31:12], 12'd0};
				we = 1'b1;
			end
			OpAuipc: begin
				we = 1'b1;
				rd_data = pcCurrent + {insn_from_imem[31:12], 12'd0};
			end
			OpRegImm: begin
				we = 1'b1;
				if (insn_addi) begin
					a = rs1_data;
					b = {{20 {imm_i[11]}}, imm_i[11:0]};
					cin = 1'b0;
					rd_data = sum;
				end
				else if (insn_slti)
					rd_data = ($signed(rs1_data) < $signed({{20 {imm_i[11]}}, imm_i}) ? 1 : 0);
				else if (insn_sltiu)
					rd_data = (rs1_data < {{20 {imm_i[11]}}, imm_i} ? 1 : 0);
				else if (insn_xori)
					rd_data = rs1_data ^ {{20 {imm_i[11]}}, imm_i};
				else if (insn_ori)
					rd_data = rs1_data | {{20 {imm_i[11]}}, imm_i};
				else if (insn_andi)
					rd_data = rs1_data & {{20 {imm_i[11]}}, imm_i};
				else if (insn_slli)
					rd_data = rs1_data << imm_shamt;
				else if (insn_srli)
					rd_data = rs1_data >> imm_shamt;
				else if (insn_srai)
					rd_data = $signed(rs1_data) >>> imm_shamt;
			end
			OpRegReg: begin
				we = 1'b1;
				if (insn_add) begin
					a = rs1_data;
					b = rs2_data;
					cin = 1'b0;
					rd_data = sum;
				end
				else if (insn_sub) begin
					a = rs1_data;
					b = ~rs2_data;
					cin = 1'b1;
					rd_data = sum;
				end
				else if (insn_sll)
					rd_data = rs1_data << rs2_data[4:0];
				else if (insn_slt)
					rd_data = ($signed(rs1_data) < $signed(rs2_data) ? 1 : 0);
				else if (insn_sltu)
					rd_data = (rs1_data < rs2_data ? 1 : 0);
				else if (insn_srl)
					rd_data = rs1_data >> rs2_data[4:0];
				else if (insn_sra)
					rd_data = $signed(rs1_data) >>> rs2_data[4:0];
				else if (insn_xor)
					rd_data = rs1_data ^ rs2_data;
				else if (insn_or)
					rd_data = rs1_data | rs2_data;
				else if (insn_and)
					rd_data = rs1_data & rs2_data;
				else if (insn_mul) begin
					mul_tmp = rs1_data * rs2_data;
					rd_data = mul_tmp[31:0];
				end
				else if (insn_mulh) begin
					mul_tmp = $signed(rs1_data) * $signed(rs2_data);
					rd_data = mul_tmp[63:32];
				end
				else if (insn_mulhsu) begin
					mul_tmp = $signed({{32 {rs1_data[31]}}, rs1_data}) * $unsigned(rs2_data);
					rd_data = mul_tmp[63:32];
				end
				else if (insn_mulhu) begin
					mul_tmp = rs1_data * rs2_data;
					rd_data = mul_tmp[63:32];
				end
				else if (insn_div) begin
					we = we_rdy;
					rdy = 1;
					sign = 1;
					i_dividend = rs1_data;
					i_divisor = rs2_data;
					rd_data = o_quotient;
					pcNext = (we_rdy ? pcCurrent + 32'd4 : pcCurrent);
				end
				else if (insn_divu) begin
					we = we_rdy;
					rdy = 1;
					sign = 0;
					i_dividend = rs1_data;
					i_divisor = rs2_data;
					rd_data = o_quotient;
					pcNext = (we_rdy ? pcCurrent + 32'd4 : pcCurrent);
				end
				else if (insn_rem) begin
					we = we_rdy;
					rdy = 1;
					sign = 1;
					i_dividend = rs1_data;
					i_divisor = rs2_data;
					rd_data = o_remainder;
					pcNext = (we_rdy ? pcCurrent + 32'd4 : pcCurrent);
				end
				else if (insn_remu) begin
					we = we_rdy;
					rdy = 1;
					sign = 0;
					i_dividend = rs1_data;
					i_divisor = rs2_data;
					rd_data = o_remainder;
					pcNext = (we_rdy ? pcCurrent + 32'd4 : pcCurrent);
				end
			end
			OpBranch:
				if (insn_beq) begin
					if (rs1_data == rs2_data)
						{carry, pcNext} = pcCurrent + imm_b_sext;
				end
				else if (insn_bne) begin
					if (rs1_data != rs2_data)
						{carry, pcNext} = pcCurrent + imm_b_sext;
				end
				else if (insn_blt) begin
					if ($signed(rs1_data) < $signed(rs2_data))
						{carry, pcNext} = pcCurrent + imm_b_sext;
				end
				else if (insn_bge) begin
					if ($signed(rs1_data) >= $signed(rs2_data))
						{carry, pcNext} = pcCurrent + imm_b_sext;
				end
				else if (insn_bltu) begin
					if (rs1_data < rs2_data)
						{carry, pcNext} = pcCurrent + imm_b_sext;
				end
				else if (insn_bgeu) begin
					if (rs1_data >= rs2_data)
						{carry, pcNext} = pcCurrent + imm_b_sext;
				end
			OpEnviron:
				if (insn_ecall)
					halt = 1;
			OpMiscMem: pcNext = pcCurrent + 32'd4;
			OpJal: begin
				we = 1;
				rd_data = pcCurrent + 32'd4;
				pcNext = pcCurrent + imm_j_sext;
			end
			OpJalr: begin
				we = 1;
				rd_data = pcCurrent + 32'd4;
				pcNext = (rs1_data + imm_i_sext) & ~32'd1;
			end
			OpLoad: begin
				we = 1'b1;
				addr_to_dmem_tmp = rs1_data + imm_i_sext;
				addr_to_dmem = {addr_to_dmem_tmp[31:2], 2'd0};
				if (insn_lb)
					case (addr_to_dmem_tmp[1:0])
						2'b00: rd_data = {{24 {load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
						2'b01: rd_data = {{24 {load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
						2'b10: rd_data = {{24 {load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
						2'b11: rd_data = {{24 {load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
					endcase
				else if (insn_lh)
					case (addr_to_dmem_tmp[1:0])
						2'b00: rd_data = {{16 {load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
						2'b10: rd_data = {{16 {load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
						default: rd_data = 0;
					endcase
				else if (insn_lw)
					rd_data = load_data_from_dmem[31:0];
				else if (insn_lbu)
					case (addr_to_dmem_tmp[1:0])
						2'b00: rd_data = {24'd0, load_data_from_dmem[7:0]};
						2'b01: rd_data = {24'd0, load_data_from_dmem[15:8]};
						2'b10: rd_data = {24'd0, load_data_from_dmem[23:16]};
						2'b11: rd_data = {24'd0, load_data_from_dmem[31:24]};
					endcase
				else if (insn_lhu)
					case (addr_to_dmem_tmp[1:0])
						2'b00: rd_data = {16'd0, load_data_from_dmem[15:0]};
						2'b10: rd_data = {16'd0, load_data_from_dmem[31:16]};
						default: rd_data = 0;
					endcase
				else
					we = 0;
			end
			OpStore: begin
				addr_to_dmem_tmp = rs1_data + imm_s_sext;
				addr_to_dmem = {addr_to_dmem_tmp[31:2], 2'd0};
				if (insn_sb)
					case (addr_to_dmem_tmp[1:0])
						2'b00: begin
							store_data_to_dmem = {24'd0, rs2_data[7:0]};
							store_we_to_dmem = 4'd1;
						end
						2'b01: begin
							store_data_to_dmem = {16'd0, rs2_data[7:0], 8'd0};
							store_we_to_dmem = 4'd2;
						end
						2'b10: begin
							store_data_to_dmem = {8'd0, rs2_data[7:0], 16'd0};
							store_we_to_dmem = 4'd4;
						end
						2'b11: begin
							store_data_to_dmem = {rs2_data[7:0], 24'd0};
							store_we_to_dmem = 4'd8;
						end
					endcase
				else if (insn_sh)
					case (addr_to_dmem_tmp[1:0])
						2'b00: begin
							store_data_to_dmem = {16'd0, rs2_data[15:0]};
							store_we_to_dmem = 4'd3;
						end
						2'b10: begin
							store_data_to_dmem = {rs2_data[15:0], 16'd0};
							store_we_to_dmem = 4'd12;
						end
						default: begin
							store_we_to_dmem = 0;
							store_data_to_dmem = 0;
						end
					endcase
				else if (insn_sw) begin
					store_data_to_dmem = rs2_data;
					store_we_to_dmem = 4'b1111;
				end
			end
			default: begin
				illegal_insn = 1'b0;
				pcNext = pcCurrent + 32'd4;
				rd_data = 32'b00000000000000000000000000000000;
				we = 0;
				halt = 0;
				a = 0;
				b = 0;
				cin = 0;
				a1 = 0;
				b1 = 0;
				cin1 = 0;
				carry = 0;
				sign = 0;
			end
		endcase
	end
	initial _sv2v_0 = 0;
endmodule
module MemorySingleCycle (
	rst,
	clock_mem,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	parameter signed [31:0] NUM_WORDS = 512;
	input wire rst;
	input wire clock_mem;
	input wire [31:0] pc_to_imem;
	output reg [31:0] insn_from_imem;
	input wire [31:0] addr_to_dmem;
	output reg [31:0] load_data_from_dmem;
	input wire [31:0] store_data_to_dmem;
	input wire [3:0] store_we_to_dmem;
	reg [31:0] mem_array [0:NUM_WORDS - 1];
	initial $readmemh("mem_initial_contents.hex", mem_array);
	always @(*)
		if (_sv2v_0)
			;
	localparam signed [31:0] AddrMsb = $clog2(NUM_WORDS) + 1;
	localparam signed [31:0] AddrLsb = 2;
	always @(posedge clock_mem)
		if (rst)
			;
		else
			insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
	always @(negedge clock_mem)
		if (rst)
			;
		else begin
			if (store_we_to_dmem[0])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
			if (store_we_to_dmem[1])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
			if (store_we_to_dmem[2])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
			if (store_we_to_dmem[3])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
			load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
		end
	initial _sv2v_0 = 0;
endmodule
module SystemResourceCheck (
	external_clk_25MHz,
	btn,
	led
);
	input wire external_clk_25MHz;
	input wire [6:0] btn;
	output wire [7:0] led;
	wire clk_proc;
	wire clk_mem;
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk_proc),
		.clk_mem(clk_mem),
		.locked(clk_locked)
	);
	wire [31:0] pc_to_imem;
	wire [31:0] insn_from_imem;
	wire [31:0] mem_data_addr;
	wire [31:0] mem_data_loaded_value;
	wire [31:0] mem_data_to_write;
	wire [3:0] mem_data_we;
	MemorySingleCycle #(.NUM_WORDS(128)) memory(
		.rst(!clk_locked),
		.clock_mem(clk_mem),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.load_data_from_dmem(mem_data_loaded_value),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we)
	);
	DatapathMultiCycle datapath(
		.clk(clk_proc),
		.rst(!clk_locked),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we),
		.load_data_from_dmem(mem_data_loaded_value),
		.halt(led[0])
	);
endmodule