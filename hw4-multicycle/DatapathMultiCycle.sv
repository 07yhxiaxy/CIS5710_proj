/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`include "../hw2b-cla/cla.sv"
`include "DividerUnsignedPipelined.sv"

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

endmodule

module DatapathMultiCycle (
    input wire clk,
    input wire rst,
    output logic halt,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`REG_SIZE] insn_from_imem,
    // addr_to_dmem is a read-write port
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem
);

  // TODO: your code here (largely based on HW3B)
  parameter STAGES = 8;
  // components of the instruction
  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2;
  wire [4:0] insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = insn_from_imem[31:20];
  wire [4:0] imm_shamt = insn_from_imem[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s[11:5] = insn_funct7, imm_s[4:0] = insn_rd;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};

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

  wire insn_lui   = insn_opcode == OpLui;
  wire insn_auipc = insn_opcode == OpAuipc;
  wire insn_jal   = insn_opcode == OpJal;
  wire insn_jalr  = insn_opcode == OpJalr;

  wire insn_beq  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b000;
  wire insn_bne  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b001;
  wire insn_blt  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b100;
  wire insn_bge  = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b101;
  wire insn_bltu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b110;
  wire insn_bgeu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b111;

  wire insn_lb  = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b000;
  wire insn_lh  = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b001;
  wire insn_lw  = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b010;
  wire insn_lbu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b100;
  wire insn_lhu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b101;

  wire insn_sb = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b000;
  wire insn_sh = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b001;
  wire insn_sw = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b010;

  wire insn_addi  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b000;
  wire insn_slti  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b010;
  wire insn_sltiu = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b011;
  wire insn_xori  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b100;
  wire insn_ori   = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b110;
  wire insn_andi  = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b111;

  wire insn_slli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_srli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_srai = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;

  wire insn_add  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'd0;
  wire insn_sub  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_sll  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_slt  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b010 && insn_from_imem[31:25] == 7'd0;
  wire insn_sltu = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b011 && insn_from_imem[31:25] == 7'd0;
  wire insn_xor  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b100 && insn_from_imem[31:25] == 7'd0;
  wire insn_srl  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_sra  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_or   = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b110 && insn_from_imem[31:25] == 7'd0;
  wire insn_and  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b111 && insn_from_imem[31:25] == 7'd0;

  wire insn_mul    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b000;
  wire insn_mulh   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b001;
  wire insn_mulhsu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b010;
  wire insn_mulhu  = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b011;
  wire insn_div    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b100;
  wire insn_divu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b101;
  wire insn_rem    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b110;
  wire insn_remu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b111;

  wire insn_ecall = insn_opcode == OpEnviron && insn_from_imem[31:7] == 25'd0;
  wire insn_fence = insn_opcode == OpMiscMem;

  // this code is only for simulation, not synthesis
  `ifndef SYNTHESIS
  `include "../hw3-singlecycle/RvDisassembler.sv" 
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn_from_imem);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic...
  wire [(8*32)-1:0] disasm_wire;
  genvar i;
  for (i = 0; i < 32; i = i + 1) begin : gen_disasm
    assign disasm_wire[(((i+1))*8)-1:((i)*8)] = disasm_string[31-i];
  end
  `endif

  // program counter
  logic [`REG_SIZE] pcNext, pcCurrent;
  always @(posedge clk) begin
    if (rst) begin
      pcCurrent <= 32'd0;
    end else begin
      pcCurrent <= pcNext;
    end
  end
  assign pc_to_imem = pcCurrent;

  // cycle/insn_from_imem counters
  logic [`REG_SIZE] cycles_current, num_insns_current;
  always @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
      num_insns_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
      if (!rst) begin
        num_insns_current <= num_insns_current + 1;
      end
    end
  end

  // NOTE: don't rename your RegFile instance as the tests expect it to be `rf`
  // TODO: you will need to edit the port connections, however.
  wire [`REG_SIZE] rs1_data;
  wire [`REG_SIZE] rs2_data;
  logic [`REG_SIZE] rd_data;

  logic we;

  RegFile rf (
    .clk(clk),
    .rst(rst),
    // .we(we || r1[7].we_ready),
    .we(we),
    // .rd(r1[7].we_ready ? r1[7].rd_store : insn_rd),
    .rd(insn_rd),
    // .rd_data(r1[7].we_ready ? (r1[7].div_rem ? o_remainder : o_quotient) : rd_data),
    .rd_data(rd_data),
    .rs1(insn_rs1),
    .rs2(insn_rs2),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data));

  logic illegal_insn;

  // Declare CLA and Divider:
  // CLA
  logic [31:0] a, b, sum, a1, b1, sum1, addr_to_dmem_tmp;
  logic cin, cin1;
  cla cla1(.a(a), .b(b), .cin(cin), .sum(sum));
  cla cla2(.a(a1), .b(b1), .cin(cin1), .sum(sum1));
  logic carry, carry1;
  // Multiplier signal
  logic [63:0] mul_tmp;
  // Divider
  logic [31:0] i_dividend, i_divisor, o_remainder, o_quotient;
  logic sign, stall;
  divider div1(.clk(clk), .rst(rst), .i_dividend(i_dividend), .i_divisor(i_divisor), .o_remainder(o_remainder), .o_quotient(o_quotient), .sign(sign), .stall(stall));
  
  // // Extra logic for multicycle
  // typedef struct packed {
  //   logic we_ready;
  //   logic sign;
  //   logic div_rem; // 0 for div (quotient) 1 for remainder;
  //   logic [4:0] rd_store;
  // } div_state;
  // div_state r1 [0:7];
  // logic sign_in;
  // // assign r1[0].we_ready = insn_div || insn_divu || insn_rem || insn_remu;
  // assign r1[0].we_ready = we;
  // assign r1[0].rd_store = insn_rd;
  // assign r1[0].div_rem = (insn_divu || insn_div) ? 0 : 1;
  // always_ff @(posedge clk) begin
  //   if (rst) begin
  //     for(int j = 1; j < STAGES; j++)begin
  //       r1[j].we_ready <= '0;
  //       r1[j].rd_store <= '0;
  //     end
  //   end
  //   else begin
  //     for(int j = 1; j < STAGES; j++)begin
  //       r1[j].rd_store <= r1[j-1].rd_store;
  //       r1[j].we_ready <= r1[j-1].we_ready;
  //     end
  //   end
  // end

  logic [$clog2(STAGES)-1:0] counter;
  logic rdy, we_rdy; // set rdy to 1 when division in process
  // Counter for 8-cycle division
  always_ff @(posedge clk) begin
    if (rst)begin
      counter <= '0;
    end
    else if (rdy) begin
      counter <= counter + 1;
    end
  end

  assign we_rdy = &counter;

  always_comb begin
    // Default assignment to avoid latches
    illegal_insn = 1'b0;
    pcNext = pcCurrent + 32'd4;
    rd_data = 32'b0;
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
        // TODO: start here by implementing lui
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
          b = {{20{imm_i[11]}}, imm_i[11:0]};
          cin = 1'b0;
          rd_data = sum;
          // rd_data = a + b;
        end else if (insn_slti) begin
          rd_data = ($signed(rs1_data) < $signed({{20{imm_i[11]}}, imm_i})) ? 1 : 0;
        end else if (insn_sltiu) begin
          rd_data = rs1_data < {{20{imm_i[11]}}, imm_i} ? 1 : 0;
        end else if (insn_xori) begin
          rd_data = rs1_data ^ {{20{imm_i[11]}}, imm_i};
        end else if (insn_ori) begin
          rd_data = rs1_data | {{20{imm_i[11]}}, imm_i};
        end else if (insn_andi) begin
          rd_data = rs1_data & {{20{imm_i[11]}}, imm_i};
        end else if (insn_slli) begin
          rd_data = rs1_data << imm_shamt;
        end else if (insn_srli) begin
          rd_data = rs1_data >> imm_shamt;
        end else if (insn_srai) begin
          rd_data = $signed(rs1_data) >>> imm_shamt;
        end
      end
      OpRegReg: begin
        we = 1'b1;
        // Add and Substraction
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
        // Shifts
        else if (insn_sll) begin
          rd_data = rs1_data << rs2_data[4:0];
        end
        else if (insn_slt) begin
          rd_data = ($signed(rs1_data) < $signed(rs2_data)) ? 1 : 0;
        end
        else if (insn_sltu) begin
          rd_data = (rs1_data < rs2_data) ? 1 : 0;
        end
        else if (insn_srl) begin
          rd_data = rs1_data >> rs2_data[4:0];
        end
        else if (insn_sra) begin
          rd_data = $signed(rs1_data) >>> rs2_data[4:0];
        end
        // Logical Operations
        else if (insn_xor) begin
          rd_data = rs1_data ^ rs2_data;
        end
        else if (insn_or) begin
          rd_data = rs1_data | rs2_data;
        end
        else if (insn_and) begin
          rd_data = rs1_data & rs2_data;
        end
        // Multiplication
        else if (insn_mul) begin
          mul_tmp = rs1_data * rs2_data;
          rd_data = mul_tmp[31:0];
        end
        else if (insn_mulh) begin
          mul_tmp = $signed(rs1_data) * $signed(rs2_data);
          rd_data = mul_tmp[63:32];
        end
        else if (insn_mulhsu) begin
          mul_tmp = $signed({{32{rs1_data[31]}}, rs1_data}) * $unsigned(rs2_data);
          // mul_tmp = $signed(rs1_data) * $unsigned(rs2_data);
          rd_data = mul_tmp[63:32];
        end
        else if (insn_mulhu) begin
          mul_tmp = rs1_data * rs2_data;
          rd_data = mul_tmp[63:32];
        end
        // Division
        else if (insn_div) begin
          we = we_rdy;
          rdy = 1;
          sign = 1;
          i_dividend = rs1_data;
          i_divisor = rs2_data;
          rd_data = o_quotient;
          pcNext = we_rdy ? pcCurrent + 32'd4 : pcCurrent;
        end
        else if (insn_divu) begin
          we = we_rdy;
          rdy = 1;
          sign = 0;
          i_dividend = rs1_data;
          i_divisor = rs2_data;
          rd_data = o_quotient;
          pcNext = we_rdy ? pcCurrent + 32'd4 : pcCurrent;
        end
        else if (insn_rem) begin
          we = we_rdy;
          rdy = 1;
          sign = 1;
          i_dividend = rs1_data;
          i_divisor = rs2_data;
          rd_data = o_remainder;
          pcNext = we_rdy ? pcCurrent + 32'd4 : pcCurrent;
        end
        else if (insn_remu) begin
          we = we_rdy;
          rdy = 1;
          sign = 0;
          i_dividend = rs1_data;
          i_divisor = rs2_data;
          rd_data = o_remainder;
          pcNext = we_rdy ? pcCurrent + 32'd4 : pcCurrent;
        end
      end
      OpBranch: begin
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
      end
      OpEnviron:
        if (insn_ecall) halt = 1;
      OpMiscMem:
        pcNext = pcCurrent + 32'd4;
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
        if (insn_lb) begin
          case (addr_to_dmem_tmp[1:0])
            2'b00: rd_data = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
            2'b01: rd_data = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
            2'b10: rd_data = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
            2'b11: rd_data = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
          endcase
        end
        else if (insn_lh) begin
          case (addr_to_dmem_tmp[1:0])
            2'b00: rd_data = {{16{load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
            2'b10: rd_data = {{16{load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
            default: rd_data = 0;
          endcase
        end
        else if (insn_lw) begin
          rd_data = load_data_from_dmem[31:0];
        end
        else if (insn_lbu) begin
          case (addr_to_dmem_tmp[1:0])
            2'b00: rd_data = {24'd0, load_data_from_dmem[7:0]};
            2'b01: rd_data = {24'd0, load_data_from_dmem[15:8]};
            2'b10: rd_data = {24'd0, load_data_from_dmem[23:16]};
            2'b11: rd_data = {24'd0, load_data_from_dmem[31:24]};
          endcase
        end
        else if (insn_lhu) begin
          case (addr_to_dmem_tmp[1:0])
            2'b00: rd_data = {16'd0, load_data_from_dmem[15:0]};
            2'b10: rd_data = {16'd0, load_data_from_dmem[31:16]};
            default: rd_data = 0;
          endcase
        end
        else
          we = 0;
      end
      OpStore: begin
        addr_to_dmem_tmp = rs1_data + imm_s_sext;
        addr_to_dmem = {addr_to_dmem_tmp[31:2], 2'd0};
        if (insn_sb) begin
          case (addr_to_dmem_tmp[1:0])
            2'b00: begin
              store_data_to_dmem = {24'd0, rs2_data[7:0]};
              store_we_to_dmem = 4'd1;
            end
            2'b01: begin
              store_data_to_dmem = {16'd0, rs2_data[7:0], 8'd0};
              // store_data_to_dmem = {16'd0, rs2_data[15:8], 8'd0};
              store_we_to_dmem = 4'd2;
            end
            2'b10: begin
              store_data_to_dmem = {8'd0, rs2_data[7:0], 16'd0};
              // store_data_to_dmem = {8'd0, rs2_data[23:16], 16'd0};
              store_we_to_dmem = 4'd4;
            end
            2'b11: begin
              store_data_to_dmem = {rs2_data[7:0], 24'd0};
              // store_data_to_dmem = {rs2_data[31:24], 24'd0};
              store_we_to_dmem = 4'd8;
            end
          endcase
        end
        else if (insn_sh) begin
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
        end
        else if (insn_sw) begin
          store_data_to_dmem = rs2_data;
          store_we_to_dmem = 4'b1111;
        end
      end

      default: begin
        // Default assignment that catches states not designed
        illegal_insn = 1'b0;
        pcNext = pcCurrent + 32'd4;
        rd_data = 32'b0;
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

endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. See RiscvProcessor for clock details.
    input wire clock_mem,

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

  always @(posedge clock_mem) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clock_mem) begin
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

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

        ____
 proc: |    |______
           ____
 mem:  ___|    |___
*/
module Processor (
    input  wire  clock_proc,
    input  wire  clock_mem,
    input  wire  rst,
    output logic halt
);

  wire [`REG_SIZE] pc_to_imem, insn_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst      (rst),
      .clock_mem (clock_mem),
      // imem is read-only
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      // dmem is read-write
      .addr_to_dmem(mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem  (mem_data_we)
  );

  DatapathMultiCycle datapath (
      .clk(clock_proc),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt)
  );

endmodule
