`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
`include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "cycle_status.sv"

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef SYNTHESIS
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
`endif
endmodule

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  // genvar i;
  logic [`REG_SIZE] regs[NumRegs];

  // TODO: copy your HW3B code here
  always @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < NumRegs; i++) begin
        regs[i] <= '0;
      end
    end else if (we && rd != 0) begin
      regs[rd] <= rd_data;
    end
  end

  assign rs1_data = regs[rs1];
  assign rs2_data = regs[rs2];

  // assign rs1_data = (rs1 == 0) ? '0 : regs[rs1]; // x0 is hardwired to 0
  // assign rs2_data = (rs2 == 0) ? '0 : regs[rs2]; // x0 is hardwired to 0

endmodule

module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  // Parameters
  // parameter STAGES = 8;

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  /***************/
  /* FETCH STAGE */
  /***************/

  logic [`REG_SIZE] f_pc_current;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  // program counter
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      // NB: use CYCLE_NO_STALL since this is the value that will persist after the last reset cycle
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      f_cycle_status <= |f_insn_branch ? CYCLE_NO_STALL : CYCLE_TAKEN_BRANCH;
      f_pc_current <= f_pc_branch;
    end
  end
  // send PC to imem
  assign pc_to_imem = f_pc_current;
  // assert (pc_to_imem[1:0] == 2'b00);
  assign f_insn = insn_from_imem;

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn_branch),
      .disasm(f_disasm)
  );

  /****************/
  /* DECODE STAGE */
  /****************/

  wire [255:0] d_disasm;
  Disasm #(
      .PREFIX("d")
  ) disasm_0decode (
      .insn  (decode_state.insn),
      .disasm(d_disasm)
  );

  // stores the alu operation
  typedef enum logic [5:0] {
    ALU_NOP,    // No operation (default)
    ALU_ECALL,
    ALU_LUI,
    ALU_AUIPC,
    ALU_JAL,
    ALU_JALR,
    ALU_BEQ,
    ALU_BNE,
    ALU_BLT,
    ALU_BGE,
    ALU_BLTU,
    ALU_BGEU,
    ALU_ADD,   // Addition
    ALU_ADDI,
    ALU_SUB,   // Subtraction
    ALU_SLL,   // Shift Left Logical
    ALU_SLLI,
    ALU_SLT,   // Set Less Than (signed)
    ALU_SLTI,
    ALU_SLTIU,
    ALU_SLTU,  // Set Less Than (unsigned)
    ALU_XOR,   // Bitwise XOR
    ALU_XORI,
    ALU_SRL,   // Shift Right Logical
    ALU_SRLI,
    ALU_SRA,   // Shift Right Arithmetic
    ALU_SRAI,
    ALU_OR,    // Bitwise OR
    ALU_ORI,
    ALU_AND,   // Bitwise AND
    ALU_ANDI,
    ALU_MUL,   // Multiplication
    ALU_MULH,
    ALU_MULHU,
    ALU_MULHSU,
    ALU_DIV,   // Division (signed)
    ALU_DIVU,  // Division (unsigned)
    ALU_REM,   // Remainder (signed)
    ALU_REMU  // Remainder (unsigned)
  } alu_op_t;

  /** state at the start of Decode stage */
  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [4:0] rs1; // operand 1
    logic [4:0] rs2; // operand 2
    logic [`REG_SIZE] imm;
    logic [4:0] rd;
    logic reg_write;
    logic mem_write;
    logic mem_read;
    logic branch;
    logic jump;
    alu_op_t alu_op;
    cycle_status_e cycle_status;
    logic ecall;
  } stage_decode_t;

  // this shows how to package up state in a `struct packed`, and how to pass it between stages
  stage_decode_t decode_state;
  alu_op_t dec_alu;
  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{
        pc: 0,
        insn: 0,
        rs1: 0,
        rs2: 0,
        imm: 0,
        rd: 0,
        reg_write: 0,
        mem_write: 0,
        mem_read: 0,
        branch: 0,
        jump: 0,
        alu_op: ALU_NOP,
        cycle_status: CYCLE_NO_STALL,
        ecall: 0
      };
    end else begin
      begin
        decode_state <= '{
          pc: (|f_insn_branch) ? f_pc_current : 0,
          insn: f_insn_branch,
          rs1: d_rs1, // operand 1
          rs2: d_rs2, // operand 2
          imm: d_imm,
          rd: d_rd,
          reg_write: d_reg_write,
          mem_write: d_mem_write,
          mem_read: d_mem_read,
          branch: d_branch,
          jump: d_jump,
          alu_op: dec_alu,
          cycle_status: d_cycle_status,
          ecall: writeback_state.ecall
        };
      end
    end
  end

/*-------------------------- Begin - Instruction Decoded into Components --------------------------*/
  // components of the instruction
  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2;
  wire [4:0] insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = f_insn_branch;

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = f_insn_branch[31:20];
  wire [4:0] imm_shamt = f_insn_branch[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s[11:5] = insn_funct7, imm_s[4:0] = insn_rd;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {f_insn_branch[31:12], 1'b0};

  wire [`REG_SIZE] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

  // Assign Alu_op operations
  always_comb begin
    dec_alu = ALU_NOP; // default
    if (insn_opcode == OpLui) begin
      dec_alu = ALU_LUI;
    end
    else if (insn_opcode == OpAuipc) begin
      dec_alu = ALU_AUIPC;
    end
    else if (insn_opcode == OpJal) begin
      dec_alu = ALU_JAL;
    end
    else if (insn_opcode == OpJalr) begin
      dec_alu = ALU_JALR;
    end
    else if (insn_opcode == OpBranch && f_insn_branch[14:12] == 3'b000) begin
      dec_alu = ALU_BEQ;
    end
    else if (insn_opcode == OpBranch && f_insn_branch[14:12] == 3'b001) begin
      dec_alu = ALU_BNE;
    end
    else if (insn_opcode == OpBranch && f_insn_branch[14:12] == 3'b100) begin
      dec_alu = ALU_BLT;
    end
    else if (insn_opcode == OpBranch && f_insn_branch[14:12] == 3'b101) begin
      dec_alu = ALU_BGE;
    end
    else if (insn_opcode == OpBranch && f_insn_branch[14:12] == 3'b110) begin
      dec_alu = ALU_BLTU;
    end
    else if (insn_opcode == OpBranch && f_insn_branch[14:12] == 3'b111) begin
      dec_alu = ALU_BGEU;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b000) begin
      dec_alu = ALU_ADDI;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b010) begin
      dec_alu = ALU_SLTI;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b011) begin
      dec_alu = ALU_SLTIU;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b100) begin
      dec_alu = ALU_XORI;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b110) begin
      dec_alu = ALU_ORI;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b111) begin
      dec_alu = ALU_ANDI;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b001 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_SLLI;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b101 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_SRLI;
    end
    else if (insn_opcode == OpRegImm && f_insn_branch[14:12] == 3'b101 && f_insn_branch[31:25] == 7'b0100000) begin
      dec_alu = ALU_SRAI;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b000 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_ADD;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b000 && f_insn_branch[31:25] == 7'b0100000) begin
      dec_alu = ALU_SUB;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b001 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_SLL;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b010 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_SLT;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b011 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_SLTU;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b100 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_XOR;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b101 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_SRL;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b101 && f_insn_branch[31:25] == 7'b0100000) begin
      dec_alu = ALU_SRA;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b110 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_OR;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[14:12] == 3'b111 && f_insn_branch[31:25] == 7'd0) begin
      dec_alu = ALU_AND;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b000) begin
      dec_alu = ALU_MUL;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b001) begin
      dec_alu = ALU_MULH;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b010) begin
      dec_alu = ALU_MULHSU;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b011) begin
      dec_alu = ALU_MULHU;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b100) begin
      dec_alu = ALU_DIV;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b101) begin
      dec_alu = ALU_DIVU;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b110) begin
      dec_alu = ALU_REM;
    end
    else if (insn_opcode == OpRegReg && f_insn_branch[31:25] == 7'd1 && f_insn_branch[14:12] == 3'b111) begin
      dec_alu = ALU_REMU;
    end
    else if (insn_opcode == OpEnviron && f_insn_branch[31:7] == 25'd0) begin
      dec_alu = ALU_ECALL;
    end
  end

  wire insn_lb  = insn_opcode == OpLoad && f_insn_branch[14:12] == 3'b000;
  wire insn_lh  = insn_opcode == OpLoad && f_insn_branch[14:12] == 3'b001;
  wire insn_lw  = insn_opcode == OpLoad && f_insn_branch[14:12] == 3'b010;
  wire insn_lbu = insn_opcode == OpLoad && f_insn_branch[14:12] == 3'b100;
  wire insn_lhu = insn_opcode == OpLoad && f_insn_branch[14:12] == 3'b101;

  wire insn_sb = insn_opcode == OpStore && f_insn_branch[14:12] == 3'b000;
  wire insn_sh = insn_opcode == OpStore && f_insn_branch[14:12] == 3'b001;
  wire insn_sw = insn_opcode == OpStore && f_insn_branch[14:12] == 3'b010;

  wire insn_fence = insn_opcode == OpMiscMem;

/*-------------------------- End - Instruction Decoded into Components --------------------------*/

  // TODO: your code here, though you will also need to modify some of the code above
  // TODO: the testbench requires that your register file instance is named `rf`

  logic [31:0] d_imm; // For pipeline's use
  logic [4:0] d_rs1, d_rs2, d_rd;
  logic d_mem_write, d_mem_read, d_reg_write, d_branch, d_jump; // For pipeline's use
  logic [31:0] rd_data, rs2_data, rs1_data; // For rf's port connection naming
  cycle_status_e d_cycle_status;

    // Parse instruction Opcode and set the decode pipeline register values
  always_comb begin
    // Default assignment to avoid latches
    // Decode Stage
    d_rs1 = 0;
    d_rs2 = 0;
    d_imm = 0;
    d_mem_write = 0; // remember to set this in hw5b
    d_mem_read = 0;
    d_reg_write = 0;
    d_branch = 0;
    d_jump = 0;
    d_rd = 0;
    halt = memory_state.ecall;
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
        // Determine the immediate value based on the instruction type
        if (dec_alu == ALU_ADDI || dec_alu == ALU_SLTI || dec_alu == ALU_SLTIU ||
            dec_alu == ALU_XORI || dec_alu == ALU_ORI || dec_alu == ALU_ANDI) begin
          d_imm = {{20{imm_i[11]}}, imm_i[11:0]}; // Sign-extended 12-bit immediate
        end
        else if (dec_alu == ALU_SLLI || dec_alu == ALU_SRLI || dec_alu == ALU_SRAI) begin
          d_imm = {27'd0, imm_shamt}; // Shift amount (5 bits)
        end
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
      // OpEnviron:
      //   if (insn_ecall) halt = 1;
      // OpMiscMem:
      //   pcNext = pcCurrent + 32'd4;
      // OpJal: begin
      //   we = 1;
      //   rd_data = pcCurrent + 32'd4;
      //   pcNext = pcCurrent + imm_j_sext;
      // end
      // OpJalr: begin
      //   we = 1;
      //   rd_data = pcCurrent + 32'd4;
      //   pcNext = (rs1_data + imm_i_sext) & ~32'd1;
      // end
      // OpLoad: begin
      //   we = 1'b1;
      //   addr_to_dmem_tmp = rs1_data + imm_i_sext;
      //   addr_to_dmem = {addr_to_dmem_tmp[31:2], 2'd0};
      //   if (insn_lb) begin
      //     case (addr_to_dmem_tmp[1:0])
      //       2'b00: rd_data = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
      //       2'b01: rd_data = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
      //       2'b10: rd_data = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
      //       2'b11: rd_data = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
      //     endcase
      //   end
      //   else if (insn_lh) begin
      //     case (addr_to_dmem_tmp[1:0])
      //       2'b00: rd_data = {{16{load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
      //       2'b10: rd_data = {{16{load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
      //       default: rd_data = 0;
      //     endcase
      //   end
      //   else if (insn_lw) begin
      //     rd_data = load_data_from_dmem[31:0];
      //   end
      //   else if (insn_lbu) begin
      //     case (addr_to_dmem_tmp[1:0])
      //       2'b00: rd_data = {24'd0, load_data_from_dmem[7:0]};
      //       2'b01: rd_data = {24'd0, load_data_from_dmem[15:8]};
      //       2'b10: rd_data = {24'd0, load_data_from_dmem[23:16]};
      //       2'b11: rd_data = {24'd0, load_data_from_dmem[31:24]};
      //     endcase
      //   end
      //   else if (insn_lhu) begin
      //     case (addr_to_dmem_tmp[1:0])
      //       2'b00: rd_data = {16'd0, load_data_from_dmem[15:0]};
      //       2'b10: rd_data = {16'd0, load_data_from_dmem[31:16]};
      //       default: rd_data = 0;
      //     endcase
      //   end
      //   else
      //     we = 0;
      // end
      // OpStore: begin
      //   addr_to_dmem_tmp = rs1_data + imm_s_sext;
      //   addr_to_dmem = {addr_to_dmem_tmp[31:2], 2'd0};
      //   if (insn_sb) begin
      //     case (addr_to_dmem_tmp[1:0])
      //       2'b00: begin
      //         store_data_to_dmem = {24'd0, rs2_data[7:0]};
      //         store_we_to_dmem = 4'd1;
      //       end
      //       2'b01: begin
      //         store_data_to_dmem = {16'd0, rs2_data[7:0], 8'd0};
      //         // store_data_to_dmem = {16'd0, rs2_data[15:8], 8'd0};
      //         store_we_to_dmem = 4'd2;
      //       end
      //       2'b10: begin
      //         store_data_to_dmem = {8'd0, rs2_data[7:0], 16'd0};
      //         // store_data_to_dmem = {8'd0, rs2_data[23:16], 16'd0};
      //         store_we_to_dmem = 4'd4;
      //       end
      //       2'b11: begin
      //         store_data_to_dmem = {rs2_data[7:0], 24'd0};
      //         // store_data_to_dmem = {rs2_data[31:24], 24'd0};
      //         store_we_to_dmem = 4'd8;
      //       end
      //     endcase
      //   end
      //   else if (insn_sh) begin
      //     case (addr_to_dmem_tmp[1:0])
      //       2'b00: begin
      //         store_data_to_dmem = {16'd0, rs2_data[15:0]};
      //         store_we_to_dmem = 4'd3;
      //       end
      //       2'b10: begin
      //         store_data_to_dmem = {rs2_data[15:0], 16'd0};
      //         store_we_to_dmem = 4'd12;
      //       end
      //       default: begin
      //         store_we_to_dmem = 0;
      //         store_data_to_dmem = 0;
      //       end
      //     endcase
      //   end
      //   else if (insn_sw) begin
      //     store_data_to_dmem = rs2_data;
      //     store_we_to_dmem = 4'b1111;
      //   end
      // end

      default: begin
        // Default assignment that catches states not designed
        d_rs1 = 0;
        d_rs2 = 0;
        d_imm = 0;
        d_mem_write = 0; // remember to set this in hw5b
        d_mem_read = 0;
        d_reg_write = 0;
        d_branch = 0;
        d_jump = 0;
        d_rd = 0;
      end
    endcase
  end

  // Instantiate regfile
  RegFile rf(.rs1(decode_state.rs1), .rs1_data(rs1_data), .rs2(decode_state.rs2), .rs2_data(rs2_data),
            .rd(writeback_state.rd), .rd_data(writeback_state.rd_data), .we(writeback_state.reg_write), .rst(rst), .clk(clk));

  /*******************/
  /* EXECUTION STAGE */
  /*******************/

  wire [255:0] x_disasm;
  Disasm #(
      .PREFIX("X")
  ) disasm_0execute (
      .insn  (execution_state.insn),
      .disasm(x_disasm)
  );

  // Declare execution state signals
  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [4:0] rs1;
    logic [`REG_SIZE] rs1_data;
    logic [4:0] rs2;
    logic [`REG_SIZE] rs2_data;
    logic [4:0] rd;
    logic [`REG_SIZE] imm;
    logic reg_write;
    logic mem_write;
    logic mem_read;
    logic branch;
    logic jump;
    alu_op_t alu_op;
    cycle_status_e cycle_status;
  } stage_exec_t;
  // Instantiate execution state object
  stage_exec_t execution_state;

  // Declare execution stage reg file
  always_ff @(posedge clk) begin
    if (rst) begin
      execution_state <= '{
          pc: 0,
          insn: 0,
          rd: 0,
          rs1: '0,
          rs1_data: '0,
          rs2: '0,
          rs2_data: 0,
          imm: '0,
          reg_write: 0,
          mem_write: 0,
          mem_read: 0,
          branch: 0,
          jump: 0,
          alu_op: ALU_NOP,
          cycle_status: CYCLE_NO_STALL
      };
    end else begin
      execution_state <= '{
          pc: (|d_insn_branch) ? decode_state.pc : 0,
          insn: d_insn_branch,
          rd: decode_state.rd,
          rs1: decode_state.rs1,
          rs1_data: d_rs1_data_bypassed,
          rs2: decode_state.rs2,
          rs2_data: d_rs2_data_bypassed,
          imm: decode_state.imm,
          reg_write: d_reg_write_branch,
          mem_write: decode_state.mem_write,
          mem_read: decode_state.mem_read,
          branch: decode_state.branch,
          jump: decode_state.jump,
          alu_op: d_alu_op,
          cycle_status: (d_alu_op == ALU_NOP) ? CYCLE_TAKEN_BRANCH : decode_state.cycle_status
      };
    end
  end

  logic [63:0] mul_tmp;
  logic [`REG_SIZE] x_pc, x_rd_data;
  logic x_branch_taken, x_ecall;
  // Execution Stage ALU operations based on execustion stage's ones
  always_comb begin
    // default values
    x_ecall = 0;
    x_rd_data = '0;
    x_pc = decode_state.pc;
    mul_tmp = '0;
    x_branch_taken = 0;
    case(execution_state.alu_op)
      ALU_NOP: begin
      end
      ALU_LUI: begin
        x_rd_data = execution_state.imm;
      end
      ALU_AUIPC: begin
        x_rd_data = execution_state.pc + execution_state.imm;
      end
      ALU_ADDI: begin
        x_rd_data =  x_rs1_data_bypassed + execution_state.imm;
      end
      ALU_SLTI: begin
        x_rd_data =  x_rs1_data_bypassed < $signed(execution_state.imm) ? 1 : 0;
      end
      ALU_SLTIU: begin
        x_rd_data =  x_rs1_data_bypassed < $unsigned(execution_state.imm) ? 1 : 0;
      end
      ALU_XORI: begin
        x_rd_data =  x_rs1_data_bypassed ^ execution_state.imm;
      end
      ALU_ORI: begin
        x_rd_data =  x_rs1_data_bypassed | execution_state.imm;
      end
      ALU_ANDI: begin
        x_rd_data =  x_rs1_data_bypassed & execution_state.imm;
      end
      ALU_SLLI: begin
        x_rd_data =  x_rs1_data_bypassed << execution_state.imm[4:0];
      end
      ALU_SRLI: begin
        x_rd_data =  x_rs1_data_bypassed >> execution_state.imm[4:0];
      end
      ALU_SRAI: begin
        x_rd_data =  x_rs1_data_bypassed >>> execution_state.imm[4:0];
      end
      ALU_ADD: begin
        x_rd_data =  x_rs1_data_bypassed + x_rs2_data_bypassed;
      end
      ALU_SUB: begin
        x_rd_data =  x_rs1_data_bypassed - x_rs2_data_bypassed;
      end
      ALU_SLL: begin
        x_rd_data =  x_rs1_data_bypassed << x_rs2_data_bypassed[4:0];
      end
      ALU_SLT: begin
        x_rd_data = ( x_rs1_data_bypassed < $signed( x_rs2_data_bypassed)) ? 1 : 0;
      end
      ALU_SLTU: begin
        x_rd_data = ( x_rs1_data_bypassed < $unsigned( x_rs2_data_bypassed)) ? 1 : 0;
      end
      ALU_XOR: begin
        x_rd_data =  x_rs1_data_bypassed ^  x_rs2_data_bypassed;
      end
      ALU_SRL: begin
        x_rd_data =  x_rs1_data_bypassed >>  x_rs2_data_bypassed[4:0];
      end
      ALU_SRA: begin
        x_rd_data =  x_rs1_data_bypassed >>>  x_rs2_data_bypassed[4:0];
      end
      ALU_OR: begin
        x_rd_data =  x_rs1_data_bypassed |  x_rs2_data_bypassed;
      end
      ALU_AND: begin
        x_rd_data =  x_rs1_data_bypassed &  x_rs2_data_bypassed;
      end
      ALU_MUL: begin
        mul_tmp =  x_rs1_data_bypassed *  x_rs2_data_bypassed;
        x_rd_data = mul_tmp[`REG_SIZE];
      end
      ALU_MULH: begin
        mul_tmp = $signed(x_rs1_data_bypassed) * $signed( x_rs2_data_bypassed);
        x_rd_data = mul_tmp[63:32];
      end
      ALU_MULHU: begin
        mul_tmp = $unsigned(x_rs1_data_bypassed) * $unsigned( x_rs2_data_bypassed);
        x_rd_data = mul_tmp[63:32];
      end
      ALU_MULHSU: begin
        mul_tmp = $signed({{32{ x_rs1_data_bypassed[31]}},  x_rs1_data_bypassed}) * $unsigned(x_rs2_data_bypassed);
        x_rd_data = mul_tmp[63:32];
      end
      ALU_BEQ: begin
        if (x_rs1_data_bypassed ==  x_rs2_data_bypassed) begin
          x_pc = decode_state.pc + decode_state.imm;
          x_branch_taken = 1;
        end
        else begin
          x_pc = decode_state.pc;
        end
      end
      ALU_BNE: begin
        if (x_rs1_data_bypassed !=  x_rs2_data_bypassed) begin
          x_pc = execution_state.pc + execution_state.imm;
          x_branch_taken = 1;
        end
        else begin
          x_pc = execution_state.pc;
        end
      end
      ALU_BLT: begin
        if ($signed(x_rs1_data_bypassed) < $signed(x_rs2_data_bypassed)) begin
          x_pc = decode_state.pc + decode_state.imm;
          x_branch_taken = 1;
        end
        else begin
          x_pc = decode_state.pc;
        end
      end
      ALU_BGE: begin
        if ($signed( x_rs1_data_bypassed) >= $signed( x_rs2_data_bypassed)) begin
          x_pc = decode_state.pc + decode_state.imm;
          x_branch_taken = 1;
        end
        else begin
          x_pc = decode_state.pc;
        end
      end
      ALU_BLTU: begin
        if (x_rs1_data_bypassed < x_rs2_data_bypassed) begin
          x_pc = decode_state.pc + decode_state.imm;
          x_branch_taken = 1;
        end
        else begin
          x_pc = decode_state.pc;
        end
      end
      ALU_BGEU: begin
        if (x_rs1_data_bypassed >= x_rs2_data_bypassed) begin
          x_pc = decode_state.pc + decode_state.imm;
          x_branch_taken = 1;
        end
        else begin
          x_pc = decode_state.pc;
        end
      end
      ALU_ECALL: begin
        x_ecall = 1;
      end
      ALU_DIV:
        x_pc = '0;
      ALU_DIVU:
        x_pc = '0;
      ALU_REM:
        x_pc = '0;
      ALU_REMU:
        x_pc = '0;
      default:
        x_pc = '0;
    endcase
  end

  /****************/
  /* MEMORY STAGE */
  /****************/

  wire [255:0] m_disasm;
  Disasm #(
      .PREFIX("M")
  ) disasm_0memory (
      .insn  (memory_state.insn),
      .disasm(m_disasm)
  );

  typedef struct packed {
  logic [`REG_SIZE] pc;           // Program counter
  logic [`INSN_SIZE] insn;        // Instruction
  logic [`REG_SIZE] alu_result;   // Result of ALU operation (memory address)
  logic [`REG_SIZE] rs2_data;     // Data to be written to memory (for store instructions)
  logic [4:0] rd;                 // Destination register
  logic [`REG_SIZE] rd_data;
  logic reg_write;                // Write to register file
  logic mem_write;                // Write to memory
  logic mem_read;                 // Read from memory
  alu_op_t alu_op;
  logic [1:0] mem_size;           // Size of memory access (00: byte, 01: halfword, 10: word)
  logic ecall;
  cycle_status_e cycle_status;
} stage_memory_t;

stage_memory_t memory_state;

always_ff @(posedge clk) begin
  if (rst) begin
    memory_state <= '{
      pc: '0,
      insn: '0,
      alu_result: '0,
      rs2_data: '0,
      rd: '0,
      rd_data: '0,
      reg_write: '0,
      mem_write: 0,
      mem_read: 0,
      alu_op: ALU_NOP,
      mem_size: '0,
      ecall: 0,
      cycle_status: CYCLE_NO_STALL
    };
  end else begin
    memory_state <= '{
      pc: execution_state.pc,
      insn: execution_state.insn,
      alu_result: x_rd_data,
      rs2_data: execution_state.rs2_data,
      rd: execution_state.rd,
      rd_data:  x_rd_data,
      reg_write: execution_state.reg_write,
      mem_write: execution_state.mem_write,
      mem_read: execution_state.mem_read,
      alu_op: execution_state.alu_op,
      mem_size: '0,
      ecall: x_ecall,
      cycle_status: execution_state.cycle_status
    };
  end
end

  /********************/
  /* WRITEBACK STAGE */
  /********************/

  wire [255:0] w_disasm;
  Disasm #(
      .PREFIX("W")
  ) disasm_0writeback (
      .insn  (writeback_state.insn),
      .disasm(w_disasm)
  );

  typedef struct packed {
    logic [`REG_SIZE] pc;           // Program counter
    logic [`INSN_SIZE] insn;        // Instruction
    logic [`REG_SIZE] alu_result;   // Result of ALU operation (memory address)
    logic [`REG_SIZE] rs2_data;     // Data to be written to memory (for store instructions)
    logic [4:0] rd;                 // Destination register
    logic [`REG_SIZE] rd_data;
    logic reg_write;                // Write to register file
    logic mem_write;                // Write to memory
    logic mem_read;                 // Read from memory
    logic [`REG_SIZE] mem_read_data;  // data read from memory
    alu_op_t alu_op;
    logic [1:0] mem_size;           // Size of memory access (00: byte, 01: halfword, 10: word)
    logic ecall;
    cycle_status_e cycle_status;
  } stage_writeback_t;

  stage_writeback_t writeback_state;

  always_ff @(posedge clk) begin
    if (rst) begin
      writeback_state <= '{
        pc: '0,
        insn: '0,
        alu_result: '0,
        rs2_data: 0,
        rd: 0,
        rd_data: 0,
        reg_write: 0,
        mem_write: 0,
        mem_read: 0,
        mem_read_data: '0,  // hw5b
        alu_op: ALU_NOP,
        mem_size: 0,
        ecall: 0,
        cycle_status: CYCLE_NO_STALL
      };
    end
    else begin
      writeback_state <= '{
        pc: memory_state.pc,
        insn: memory_state.insn,
        alu_result: memory_state.alu_result,
        rs2_data: memory_state.rs2_data,
        rd: memory_state.rd,
        rd_data: memory_state.rd_data,
        reg_write: memory_state.reg_write,
        mem_write: memory_state.mem_write,
        mem_read: memory_state.mem_read,
        mem_read_data: '0,  // hw5b
        alu_op: memory_state.alu_op,
        mem_size: memory_state.mem_size,
        ecall: memory_state.ecall,
        cycle_status: memory_state.cycle_status
      };
    end
  end

  // --------------- Bypassing Logics ----------------------
  logic [`REG_SIZE] x_rs1_data_bypassed; // Forwarded value for rs1
  logic [`REG_SIZE] x_rs2_data_bypassed; // Forwarded value for rs2
  logic [`REG_SIZE] d_rs1_data_bypassed;
  logic [`REG_SIZE] d_rs2_data_bypassed;

  always_comb begin
    // Default: use data from the register file
    x_rs1_data_bypassed = execution_state.rs1_data;
    x_rs2_data_bypassed = execution_state.rs2_data;
    d_rs1_data_bypassed = rs1_data;
    d_rs2_data_bypassed = rs2_data;

    // Forward from Memory Stage (MX bypass)
    if (memory_state.rd == execution_state.rs2 || memory_state.rd == execution_state.rs1) begin
      // Forward from Memory Stage (MX bypass) and check if write destination is 0
      if (memory_state.rd == execution_state.rs1 && execution_state.rs1 != '0) begin
        x_rs1_data_bypassed = memory_state.rd_data; // Forward memory stage result
      end
      // Forward from Memory Stage (MX bypass)
      if (memory_state.rd == execution_state.rs2 && execution_state.rs2 != '0) begin
        x_rs2_data_bypassed = memory_state.rd_data; // Forward memory stage result
      end
    end
    // Forward from WB Stage (WX bypass)
    else if (writeback_state.rd == execution_state.rs1 || writeback_state.rd == execution_state.rs2) begin
      // Forward from WB Stage (WX bypass)
      if (writeback_state.rd == execution_state.rs1 && execution_state.rs1 != '0) begin
        x_rs1_data_bypassed = writeback_state.rd_data; // Forward wb stage result
      end
      // Forward from WB Stage (WX bypass)
      if (writeback_state.rd == execution_state.rs2 && execution_state.rs2 != '0) begin
        x_rs2_data_bypassed = writeback_state.rd_data; // Forward wb stage result
      end
    end
    // Forward from WB Stage (WD bypass)
    else if (writeback_state.rd == decode_state.rs1 || writeback_state.rd == decode_state.rs2) begin
      // Forward from WB Stage (WD bypass)
      if (writeback_state.rd == decode_state.rs1 && decode_state.rs1 != '0) begin
        d_rs1_data_bypassed = writeback_state.rd_data; // Forward wb stage result
      end
      // Forward from WB Stage (WD bypass)
      if (writeback_state.rd == decode_state.rs2 && decode_state.rs2 != '0) begin
        d_rs2_data_bypassed = writeback_state.rd_data; // Forward wb stage result
      end
    end
  end

  // Branch logics
  logic [`REG_SIZE] f_insn_branch, d_insn_branch, f_pc_branch;
  logic d_reg_write_branch;
  alu_op_t d_alu_op;
  always_comb begin
    // Default to sequantial instructions
    f_pc_branch = f_pc_current + 4;
    f_insn_branch = f_insn;
    d_insn_branch = decode_state.insn;
    d_alu_op = decode_state.alu_op;
    d_reg_write_branch = decode_state.reg_write;
    d_cycle_status = f_cycle_status;
    // If branch taken flush the pipeline
    if (x_branch_taken) begin
      f_insn_branch = '0;
      d_insn_branch = '0;
      f_pc_branch = x_pc;
      d_alu_op = ALU_NOP;
      d_reg_write_branch = 0;
      d_cycle_status = CYCLE_TAKEN_BRANCH;
    end
  end

endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem_array[NUM_WORDS];

`ifdef SYNTHESIS
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end
`endif

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/* This design has just one clock for both processor and memory. */
module Processor (
    input  wire  clk,
    input  wire  rst,
    output logic halt,
    output wire [`REG_SIZE] trace_writeback_pc,
    output wire [`INSN_SIZE] trace_writeback_insn,
    output cycle_status_e trace_writeback_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule
