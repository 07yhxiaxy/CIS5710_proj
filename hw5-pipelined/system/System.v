module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "30" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
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
		.CLKOP_DIV(20),
		.CLKOP_CPHASE(9),
		.CLKOP_FPHASE(0),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(6)
	) pll_i(
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(input_clk_25MHz),
		.CLKOP(clk_proc),
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
module Disasm (
	insn,
	disasm
);
	parameter signed [7:0] PREFIX = "D";
	input wire [31:0] insn;
	output wire [255:0] disasm;
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
module DatapathPipelined (
	clk,
	rst,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem,
	halt,
	trace_writeback_pc,
	trace_writeback_insn,
	trace_writeback_cycle_status
);
	reg _sv2v_0;
	input wire clk;
	input wire rst;
	output wire [31:0] pc_to_imem;
	input wire [31:0] insn_from_imem;
	output wire [31:0] addr_to_dmem;
	input wire [31:0] load_data_from_dmem;
	output wire [31:0] store_data_to_dmem;
	output wire [3:0] store_we_to_dmem;
	output reg halt;
	output wire [31:0] trace_writeback_pc;
	output wire [31:0] trace_writeback_insn;
	output wire [31:0] trace_writeback_cycle_status;
	reg [31:0] cycles_current;
	always @(posedge clk)
		if (rst)
			cycles_current <= 0;
		else
			cycles_current <= cycles_current + 1;
	reg [31:0] f_pc_current;
	wire [31:0] f_insn;
	reg [31:0] f_cycle_status;
	reg [31:0] f_insn_branch;
	reg [31:0] f_pc_branch;
	always @(posedge clk)
		if (rst) begin
			f_pc_current <= 32'd0;
			f_cycle_status <= 32'd2;
		end
		else begin
			f_cycle_status <= (|f_insn_branch ? 32'd2 : 32'd4);
			f_pc_current <= f_pc_branch;
		end
	assign pc_to_imem = f_pc_current;
	assign f_insn = insn_from_imem;
	wire [255:0] f_disasm;
	Disasm #(.PREFIX("F")) disasm_0fetch(
		.insn(f_insn_branch),
		.disasm(f_disasm)
	);
	wire [255:0] d_disasm;
	reg [154:0] decode_state;
	Disasm #(.PREFIX("d")) disasm_0decode(
		.insn(decode_state[122-:32]),
		.disasm(d_disasm)
	);
	reg [5:0] dec_alu;
	reg d_branch;
	reg [31:0] d_cycle_status;
	reg [31:0] d_imm;
	reg d_jump;
	reg d_mem_read;
	reg d_mem_write;
	reg [4:0] d_rd;
	reg d_reg_write;
	reg [4:0] d_rs1;
	reg [4:0] d_rs2;
	reg [240:0] writeback_state;
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	always @(posedge clk)
		if (rst)
			decode_state <= 155'h000000000000000000000000000000000000004;
		else
			decode_state <= {sv2v_cast_32((|f_insn_branch ? f_pc_current : 0)), f_insn_branch, d_rs1, d_rs2, d_imm, d_rd, d_reg_write, d_mem_write, d_mem_read, d_branch, d_jump, dec_alu, d_cycle_status, writeback_state[32]};
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = f_insn_branch;
	wire [11:0] imm_i;
	assign imm_i = f_insn_branch[31:20];
	wire [4:0] imm_shamt = f_insn_branch[24:20];
	wire [11:0] imm_s;
	assign imm_s[11:5] = insn_funct7;
	assign imm_s[4:0] = insn_rd;
	wire [12:0] imm_b;
	assign {imm_b[12], imm_b[10:5]} = insn_funct7;
	assign {imm_b[4:1], imm_b[11]} = insn_rd;
	assign imm_b[0] = 1'b0;
	wire [20:0] imm_j;
	assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {f_insn_branch[31:12], 1'b0};
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
	always @(*) begin
		if (_sv2v_0)
			;
		dec_alu = 6'd0;
		if (insn_opcode == OpLui)
			dec_alu = 6'd2;
		else if (insn_opcode == OpAuipc)
			dec_alu = 6'd3;
		else if (insn_opcode == OpJal)
			dec_alu = 6'd4;
		else if (insn_opcode == OpJalr)
			dec_alu = 6'd5;
		else if ((insn_opcode == OpBranch) && (f_insn_branch[14:12] == 3'b000))
			dec_alu = 6'd6;
		else if ((insn_opcode == OpBranch) && (f_insn_branch[14:12] == 3'b001))
			dec_alu = 6'd7;
		else if ((insn_opcode == OpBranch) && (f_insn_branch[14:12] == 3'b100))
			dec_alu = 6'd8;
		else if ((insn_opcode == OpBranch) && (f_insn_branch[14:12] == 3'b101))
			dec_alu = 6'd9;
		else if ((insn_opcode == OpBranch) && (f_insn_branch[14:12] == 3'b110))
			dec_alu = 6'd10;
		else if ((insn_opcode == OpBranch) && (f_insn_branch[14:12] == 3'b111))
			dec_alu = 6'd11;
		else if ((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b000))
			dec_alu = 6'd13;
		else if ((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b010))
			dec_alu = 6'd18;
		else if ((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b011))
			dec_alu = 6'd19;
		else if ((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b100))
			dec_alu = 6'd22;
		else if ((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b110))
			dec_alu = 6'd28;
		else if ((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b111))
			dec_alu = 6'd30;
		else if (((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b001)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd16;
		else if (((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b101)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd24;
		else if (((insn_opcode == OpRegImm) && (f_insn_branch[14:12] == 3'b101)) && (f_insn_branch[31:25] == 7'b0100000))
			dec_alu = 6'd26;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b000)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd12;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b000)) && (f_insn_branch[31:25] == 7'b0100000))
			dec_alu = 6'd14;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b001)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd15;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b010)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd17;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b011)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd20;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b100)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd21;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b101)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd23;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b101)) && (f_insn_branch[31:25] == 7'b0100000))
			dec_alu = 6'd25;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b110)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd27;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[14:12] == 3'b111)) && (f_insn_branch[31:25] == 7'd0))
			dec_alu = 6'd29;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b000))
			dec_alu = 6'd31;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b001))
			dec_alu = 6'd32;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b010))
			dec_alu = 6'd34;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b011))
			dec_alu = 6'd33;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b100))
			dec_alu = 6'd35;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b101))
			dec_alu = 6'd36;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b110))
			dec_alu = 6'd37;
		else if (((insn_opcode == OpRegReg) && (f_insn_branch[31:25] == 7'd1)) && (f_insn_branch[14:12] == 3'b111))
			dec_alu = 6'd38;
		else if ((insn_opcode == OpEnviron) && (f_insn_branch[31:7] == 25'd0))
			dec_alu = 6'd1;
	end
	wire insn_lb = (insn_opcode == OpLoad) && (f_insn_branch[14:12] == 3'b000);
	wire insn_lh = (insn_opcode == OpLoad) && (f_insn_branch[14:12] == 3'b001);
	wire insn_lw = (insn_opcode == OpLoad) && (f_insn_branch[14:12] == 3'b010);
	wire insn_lbu = (insn_opcode == OpLoad) && (f_insn_branch[14:12] == 3'b100);
	wire insn_lhu = (insn_opcode == OpLoad) && (f_insn_branch[14:12] == 3'b101);
	wire insn_sb = (insn_opcode == OpStore) && (f_insn_branch[14:12] == 3'b000);
	wire insn_sh = (insn_opcode == OpStore) && (f_insn_branch[14:12] == 3'b001);
	wire insn_sw = (insn_opcode == OpStore) && (f_insn_branch[14:12] == 3'b010);
	wire insn_fence = insn_opcode == OpMiscMem;
	wire [31:0] rd_data;
	wire [31:0] rs2_data;
	wire [31:0] rs1_data;
	reg [208:0] memory_state;
	always @(*) begin
		if (_sv2v_0)
			;
		d_rs1 = 0;
		d_rs2 = 0;
		d_imm = 0;
		d_mem_write = 0;
		d_mem_read = 0;
		d_reg_write = 0;
		d_branch = 0;
		d_jump = 0;
		d_rd = 0;
		halt = memory_state[32];
		case (insn_opcode)
			OpLui: begin
				d_reg_write = 1'b1;
				d_imm = {f_insn_branch[31:12], 12'd0};
				d_rd = insn_rd;
			end
			OpAuipc: begin
				d_reg_write = 1'b1;
				d_imm = {f_insn_branch[31:12], 12'd0};
				d_rd = insn_rd;
			end
			OpRegImm: begin
				d_rd = insn_rd;
				d_reg_write = 1'b1;
				d_rs1 = insn_rs1;
				if ((((((dec_alu == 6'd13) || (dec_alu == 6'd18)) || (dec_alu == 6'd19)) || (dec_alu == 6'd22)) || (dec_alu == 6'd28)) || (dec_alu == 6'd30))
					d_imm = {{20 {imm_i[11]}}, imm_i[11:0]};
				else if (((dec_alu == 6'd16) || (dec_alu == 6'd24)) || (dec_alu == 6'd26))
					d_imm = {27'd0, imm_shamt};
			end
			OpRegReg: begin
				d_rd = insn_rd;
				d_reg_write = 1'b1;
				d_rs1 = insn_rs1;
				d_rs2 = insn_rs2;
			end
			OpBranch: begin
				d_branch = 1;
				d_imm = imm_b_sext;
				d_rs1 = insn_rs1;
				d_rs2 = insn_rs2;
			end
			default: begin
				d_rs1 = 0;
				d_rs2 = 0;
				d_imm = 0;
				d_mem_write = 0;
				d_mem_read = 0;
				d_reg_write = 0;
				d_branch = 0;
				d_jump = 0;
				d_rd = 0;
			end
		endcase
	end
	RegFile rf(
		.rs1(decode_state[90-:5]),
		.rs1_data(rs1_data),
		.rs2(decode_state[85-:5]),
		.rs2_data(rs2_data),
		.rd(writeback_state[112-:5]),
		.rd_data(writeback_state[107-:32]),
		.we(writeback_state[75]),
		.rst(rst),
		.clk(clk)
	);
	wire [255:0] x_disasm;
	reg [217:0] execution_state;
	Disasm #(.PREFIX("X")) disasm_0execute(
		.insn(execution_state[185-:32]),
		.disasm(x_disasm)
	);
	reg [5:0] d_alu_op;
	reg [31:0] d_insn_branch;
	reg d_reg_write_branch;
	reg [31:0] d_rs1_data_bypassed;
	reg [31:0] d_rs2_data_bypassed;
	function automatic [4:0] sv2v_cast_5;
		input reg [4:0] inp;
		sv2v_cast_5 = inp;
	endfunction
	always @(posedge clk)
		if (rst)
			execution_state <= 218'h0000000000000000000000000000000000000000000000000000002;
		else
			execution_state <= {sv2v_cast_32((|d_insn_branch ? decode_state[154-:32] : 0)), d_insn_branch, sv2v_cast_5(decode_state[90-:5]), d_rs1_data_bypassed, sv2v_cast_5(decode_state[85-:5]), d_rs2_data_bypassed, sv2v_cast_5(decode_state[48-:5]), sv2v_cast_32(decode_state[80-:32]), d_reg_write_branch, decode_state[42], decode_state[41], decode_state[40], decode_state[39], d_alu_op, sv2v_cast_32((d_alu_op == 6'd0 ? 32'd4 : decode_state[32-:32]))};
	reg [63:0] mul_tmp;
	reg [31:0] x_pc;
	reg [31:0] x_rd_data;
	reg x_branch_taken;
	reg x_ecall;
	reg [31:0] x_rs1_data_bypassed;
	reg [31:0] x_rs2_data_bypassed;
	always @(*) begin
		if (_sv2v_0)
			;
		x_ecall = 0;
		x_rd_data = 1'sb0;
		x_pc = decode_state[154-:32];
		mul_tmp = 1'sb0;
		x_branch_taken = 0;
		case (execution_state[37-:6])
			6'd0:
				;
			6'd2: x_rd_data = execution_state[74-:32];
			6'd3: x_rd_data = execution_state[217-:32] + execution_state[74-:32];
			6'd13: x_rd_data = x_rs1_data_bypassed + execution_state[74-:32];
			6'd18: x_rd_data = (x_rs1_data_bypassed < $signed(execution_state[74-:32]) ? 1 : 0);
			6'd19: x_rd_data = (x_rs1_data_bypassed < $unsigned(execution_state[74-:32]) ? 1 : 0);
			6'd22: x_rd_data = x_rs1_data_bypassed ^ execution_state[74-:32];
			6'd28: x_rd_data = x_rs1_data_bypassed | execution_state[74-:32];
			6'd30: x_rd_data = x_rs1_data_bypassed & execution_state[74-:32];
			6'd16: x_rd_data = x_rs1_data_bypassed << execution_state[47:43];
			6'd24: x_rd_data = x_rs1_data_bypassed >> execution_state[47:43];
			6'd26: x_rd_data = x_rs1_data_bypassed >>> execution_state[47:43];
			6'd12: x_rd_data = x_rs1_data_bypassed + x_rs2_data_bypassed;
			6'd14: x_rd_data = x_rs1_data_bypassed - x_rs2_data_bypassed;
			6'd15: x_rd_data = x_rs1_data_bypassed << x_rs2_data_bypassed[4:0];
			6'd17: x_rd_data = (x_rs1_data_bypassed < $signed(x_rs2_data_bypassed) ? 1 : 0);
			6'd20: x_rd_data = (x_rs1_data_bypassed < $unsigned(x_rs2_data_bypassed) ? 1 : 0);
			6'd21: x_rd_data = x_rs1_data_bypassed ^ x_rs2_data_bypassed;
			6'd23: x_rd_data = x_rs1_data_bypassed >> x_rs2_data_bypassed[4:0];
			6'd25: x_rd_data = x_rs1_data_bypassed >>> x_rs2_data_bypassed[4:0];
			6'd27: x_rd_data = x_rs1_data_bypassed | x_rs2_data_bypassed;
			6'd29: x_rd_data = x_rs1_data_bypassed & x_rs2_data_bypassed;
			6'd31: begin
				mul_tmp = x_rs1_data_bypassed * x_rs2_data_bypassed;
				x_rd_data = mul_tmp[31:0];
			end
			6'd32: begin
				mul_tmp = $signed(x_rs1_data_bypassed) * $signed(x_rs2_data_bypassed);
				x_rd_data = mul_tmp[63:32];
			end
			6'd33: begin
				mul_tmp = $unsigned(x_rs1_data_bypassed) * $unsigned(x_rs2_data_bypassed);
				x_rd_data = mul_tmp[63:32];
			end
			6'd34: begin
				mul_tmp = $signed({{32 {x_rs1_data_bypassed[31]}}, x_rs1_data_bypassed}) * $unsigned(x_rs2_data_bypassed);
				x_rd_data = mul_tmp[63:32];
			end
			6'd6:
				if (x_rs1_data_bypassed == x_rs2_data_bypassed) begin
					x_pc = decode_state[154-:32] + decode_state[80-:32];
					x_branch_taken = 1;
				end
				else
					x_pc = decode_state[154-:32];
			6'd7:
				if (x_rs1_data_bypassed != x_rs2_data_bypassed) begin
					x_pc = execution_state[217-:32] + execution_state[74-:32];
					x_branch_taken = 1;
				end
				else
					x_pc = execution_state[217-:32];
			6'd8:
				if ($signed(x_rs1_data_bypassed) < $signed(x_rs2_data_bypassed)) begin
					x_pc = decode_state[154-:32] + decode_state[80-:32];
					x_branch_taken = 1;
				end
				else
					x_pc = decode_state[154-:32];
			6'd9:
				if ($signed(x_rs1_data_bypassed) >= $signed(x_rs2_data_bypassed)) begin
					x_pc = decode_state[154-:32] + decode_state[80-:32];
					x_branch_taken = 1;
				end
				else
					x_pc = decode_state[154-:32];
			6'd10:
				if (x_rs1_data_bypassed < x_rs2_data_bypassed) begin
					x_pc = decode_state[154-:32] + decode_state[80-:32];
					x_branch_taken = 1;
				end
				else
					x_pc = decode_state[154-:32];
			6'd11:
				if (x_rs1_data_bypassed >= x_rs2_data_bypassed) begin
					x_pc = decode_state[154-:32] + decode_state[80-:32];
					x_branch_taken = 1;
				end
				else
					x_pc = decode_state[154-:32];
			6'd1: x_ecall = 1;
			6'd35: x_pc = 1'sb0;
			6'd36: x_pc = 1'sb0;
			6'd37: x_pc = 1'sb0;
			6'd38: x_pc = 1'sb0;
			default: x_pc = 1'sb0;
		endcase
	end
	wire [255:0] m_disasm;
	Disasm #(.PREFIX("M")) disasm_0memory(
		.insn(memory_state[176-:32]),
		.disasm(m_disasm)
	);
	function automatic [5:0] sv2v_cast_6;
		input reg [5:0] inp;
		sv2v_cast_6 = inp;
	endfunction
	always @(posedge clk)
		if (rst)
			memory_state <= 209'h00000000000000000000000000000000000000000000000000002;
		else
			memory_state <= {sv2v_cast_32(execution_state[217-:32]), sv2v_cast_32(execution_state[185-:32]), x_rd_data, sv2v_cast_32(execution_state[111-:32]), sv2v_cast_5(execution_state[79-:5]), x_rd_data, execution_state[42], execution_state[41], execution_state[40], sv2v_cast_6(execution_state[37-:6]), 2'b00, x_ecall, sv2v_cast_32(execution_state[31-:32])};
	wire [255:0] w_disasm;
	Disasm #(.PREFIX("W")) disasm_0writeback(
		.insn(writeback_state[208-:32]),
		.disasm(w_disasm)
	);
	function automatic [1:0] sv2v_cast_2;
		input reg [1:0] inp;
		sv2v_cast_2 = inp;
	endfunction
	always @(posedge clk)
		if (rst)
			writeback_state <= 241'h0000000000000000000000000000000000000000000000000000000000002;
		else
			writeback_state <= {sv2v_cast_32(memory_state[208-:32]), sv2v_cast_32(memory_state[176-:32]), sv2v_cast_32(memory_state[144-:32]), sv2v_cast_32(memory_state[112-:32]), sv2v_cast_5(memory_state[80-:5]), sv2v_cast_32(memory_state[75-:32]), memory_state[43], memory_state[42], memory_state[41], 32'b00000000000000000000000000000000, sv2v_cast_6(memory_state[40-:6]), sv2v_cast_2(memory_state[34-:2]), memory_state[32], sv2v_cast_32(memory_state[31-:32])};
	always @(*) begin
		if (_sv2v_0)
			;
		x_rs1_data_bypassed = execution_state[148-:32];
		x_rs2_data_bypassed = execution_state[111-:32];
		d_rs1_data_bypassed = rs1_data;
		d_rs2_data_bypassed = rs2_data;
		if ((memory_state[80-:5] == execution_state[116-:5]) || (memory_state[80-:5] == execution_state[153-:5])) begin
			if ((memory_state[80-:5] == execution_state[153-:5]) && (execution_state[153-:5] != {5 {1'sb0}}))
				x_rs1_data_bypassed = memory_state[75-:32];
			if ((memory_state[80-:5] == execution_state[116-:5]) && (execution_state[116-:5] != {5 {1'sb0}}))
				x_rs2_data_bypassed = memory_state[75-:32];
		end
		else if ((writeback_state[112-:5] == execution_state[153-:5]) || (writeback_state[112-:5] == execution_state[116-:5])) begin
			if ((writeback_state[112-:5] == execution_state[153-:5]) && (execution_state[153-:5] != {5 {1'sb0}}))
				x_rs1_data_bypassed = writeback_state[107-:32];
			if ((writeback_state[112-:5] == execution_state[116-:5]) && (execution_state[116-:5] != {5 {1'sb0}}))
				x_rs2_data_bypassed = writeback_state[107-:32];
		end
		else if ((writeback_state[112-:5] == decode_state[90-:5]) || (writeback_state[112-:5] == decode_state[85-:5])) begin
			if ((writeback_state[112-:5] == decode_state[90-:5]) && (decode_state[90-:5] != {5 {1'sb0}}))
				d_rs1_data_bypassed = writeback_state[107-:32];
			if ((writeback_state[112-:5] == decode_state[85-:5]) && (decode_state[85-:5] != {5 {1'sb0}}))
				d_rs2_data_bypassed = writeback_state[107-:32];
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		f_pc_branch = f_pc_current + 4;
		f_insn_branch = f_insn;
		d_insn_branch = decode_state[122-:32];
		d_alu_op = decode_state[38-:6];
		d_reg_write_branch = decode_state[43];
		d_cycle_status = f_cycle_status;
		if (x_branch_taken) begin
			f_insn_branch = 1'sb0;
			d_insn_branch = 1'sb0;
			f_pc_branch = x_pc;
			d_alu_op = 6'd0;
			d_reg_write_branch = 0;
			d_cycle_status = 32'd4;
		end
	end
	initial _sv2v_0 = 0;
endmodule
module MemorySingleCycle (
	rst,
	clk,
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
	input wire clk;
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
	always @(negedge clk)
		if (rst)
			;
		else
			insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
	always @(negedge clk)
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
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk_proc),
		.locked(clk_locked)
	);
	wire [31:0] pc_to_imem;
	wire [31:0] insn_from_imem;
	wire [31:0] mem_data_addr;
	wire [31:0] mem_data_loaded_value;
	wire [31:0] mem_data_to_write;
	wire [3:0] mem_data_we;
	wire [31:0] trace_writeback_pc;
	wire [31:0] trace_writeback_insn;
	wire [31:0] trace_writeback_cycle_status;
	MemorySingleCycle #(.NUM_WORDS(128)) memory(
		.rst(!clk_locked),
		.clk(clk_proc),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.load_data_from_dmem(mem_data_loaded_value),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we)
	);
	DatapathPipelined datapath(
		.clk(clk_proc),
		.rst(!clk_locked),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we),
		.load_data_from_dmem(mem_data_loaded_value),
		.halt(led[0]),
		.trace_writeback_pc(trace_writeback_pc),
		.trace_writeback_insn(trace_writeback_insn),
		.trace_writeback_cycle_status(trace_writeback_cycle_status)
	);
endmodule