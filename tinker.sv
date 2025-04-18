module forward_unit(
    input [4:0] exmem_rd,
    input exmem_regwrite,
    input [4:0] memwb_rd,
    input memwb_regwrite,
    input [4:0] rs_IDEX,
    input [4:0] rt_IDEX,
    output logic[1:0] sel_opA,
    output logic [1:0] sel_opB
);
    // 00 = from regFile, 01 = from EX/MEM, 10 = from MEM/WB

    // choose the most recent value for each operand
    // prioritize ex/mem over mem/wb

    always @(*) begin
        // default selections -> take value from register file
        sel_opA = 2'd0;
        sel_opB = 2'd0;

        // opA (rs)
        if (exmem_regwrite && (exmem_rd != 0) && (exmem_rd==rs_IDEX))
            sel_opA = 2'd1;
        else if (memwb_regwrite && (memwb_rd != 0) && (memwb_rd == rs_IDEX))
            sel_opA = 2'd2;
        
        // opB (rt)
        if (exmem_regwrite && (exmem_rd != 0) && (exmem_rd == rt_IDEX))
            sel_opB = 2'd1;
        else if (memwb_regwrite && (memwb_rd != 0) && (memwb_rd == rt_IDEX))
            sel_opB = 2'd2;
    end
endmodule

// detects load RAW hazard and requests a stall
module hazard_unit(
    input logic idex_memRead, // 1 if ID/EX instruciton is a load
    input logic[4:0] idex_rd, // idex dest register
    input logic[4:0] ifid_rs, // rs field of instruction in IF/ID
    input logic[4:0] ifid_rt, // rt field of instruction in IFID
    output logic stall
);
    // if next instruction needs register that will be produced by outstanding load
    // --> stall one cycle because data will only be ready in the MEM stage
    always @(*) begin
        stall = 1'b0;
        if (idex_memRead && (idex_rd != 0) && ((idex_rd==ifid_rs) || (idex_rd==ifid_rt)))
            stall = 1'b1;
    end
endmodule

module instruction_decoder(
    input  [31:0] in,       // 32-bit instruction
    output [4:0]  opcode,   // Bits [31:27]
    output [4:0]  rd,       // Bits [26:22]
    output [4:0]  rs,       // Bits [21:17]
    output [4:0]  rt,       // Bits [16:12]
    output [11:0] L         // Bits [11:0]
);
    assign opcode = in[31:27];
    assign rd     = in[26:22];
    assign rs     = in[21:17];
    assign rt     = in[16:12];
    assign L      = in[11:0];
endmodule

module alu (
    input  [4:0]  opcode,
    input  [63:0] rdData,       // First operand
    input  [63:0] rsData,       // Second operand
    input  [63:0] rtData,
    input  [11:0] L,         // 12-bit literal/immediate
    output reg [63:0] result // Result
);
    real op1, op2, res_real; // for fpu
    always @(*) begin
        op1 = $bitstoreal(rsData);
        op2 = $bitstoreal(rtData);
        case (opcode)
            // Integer arithmetic
            5'h18: result = rsData + rtData;                   // add
            5'h19: result = rdData + {52'd0, L};             // addi
            5'h1a: result = rsData - rtData;                   // sub
            5'h1b: result = rdData - {52'd0, L};            // subi
            5'h1c: result = rsData * rtData;                   // mul
            5'h1d: result = rsData / rtData;
            // Logical operations
            5'h0:  result = rsData & rtData;                   // and
            5'h1:  result = rsData | rtData;                   // or
            5'h2:  result = rtData ^ rsData;                   // xor
            5'h3:  result = ~rsData;                        // not (rt ignored)
            // Shift operations
            5'h4:  result = rsData >> rtData;                  // shftr
            5'h5:  result = rdData >> L;                    // shftri
            5'h6:  result = rsData << rtData;                  // shftl
            5'h7:  result = rdData << L;                    // shftli
            // Data movement
            5'h11: result = rsData;                        // mov rd, rs
            5'h12: begin                                 // mov rd, L: update lower 12 bits
                      result = rdData;
                      result[11:0] = L;
                   end
            // floating point
            5'h14: begin
                res_real = op1 + op2; // addf
                result = $realtobits(res_real);
            end 
            5'h15: begin
                res_real = op1 - op2; // subf
                result = $realtobits(res_real);
            end 
            5'h16: begin
                res_real = op1 * op2; // mulf
                result = $realtobits(res_real);
            end
            5'h17: begin
                res_real = op1 / op2; // divf
                result = $realtobits(res_real);
            end
            default: result = 64'b0;
        endcase
    end
endmodule


module regFile (
    input         clk,
    input         reset,
    input  [63:0] data_in,   // Data to write
    input         we,        // Write enable
    input [4:0]   wrAddr,
    input  [4:0]  rd,        // Write address
    input  [4:0]  rs,        // Read address 1
    input  [4:0]  rt,        // Read address 2
    output reg [63:0] rdOut, // Data out pota rd
    output reg [63:0] rsOut, // Data out port A
    output reg [63:0] rtOut  // Data out port B
);
    reg [63:0] registers [0:31];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                registers[i] <= 64'b0;
            registers[31] <= 64'h80000;
        end else begin
            if (we)
                registers[wrAddr] <= data_in;
        end
    end
    
    // Combinational read.
    always @(*) begin
        rdOut = registers[rd];
        rsOut = registers[rs];
        rtOut = registers[rt];
    end
endmodule

module memory(
   input clk,
   input reset,
   // Fetch interface:
   input  [31:0] fetch_addr,
   output [31:0] fetch_instruction,
   // Data load interface:
   input  [31:0] data_load_addr,
   output [63:0] data_load,
   // Store interface:
   input         store_we,
   input  [31:0] store_addr,
   input  [63:0] store_data
);
    parameter MEM_SIZE = 512*1024;  // 512 KB
    reg [7:0] bytes [0:MEM_SIZE-1];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
        end
        if (store_we) begin
            
            bytes[store_addr+7]     <= store_data[63:56];
            bytes[store_addr+6] <= store_data[55:48];
            bytes[store_addr+5] <= store_data[47:40];
            bytes[store_addr+4] <= store_data[39:32];
            bytes[store_addr+3] <= store_data[31:24];
            bytes[store_addr+2] <= store_data[23:16];
            bytes[store_addr+1] <= store_data[15:8];
            bytes[store_addr] <= store_data[7:0];
        end
    end
    

    assign fetch_instruction = { 
        bytes[fetch_addr+3],
        bytes[fetch_addr+2],
        bytes[fetch_addr+1],
        bytes[fetch_addr]
    };
    
    assign data_load = { 
        bytes[data_load_addr+7],
        bytes[data_load_addr+6],
        bytes[data_load_addr+5],
        bytes[data_load_addr+4],
        bytes[data_load_addr+3],
        bytes[data_load_addr+2],
        bytes[data_load_addr+1],
        bytes[data_load_addr]
    };
endmodule


module fetch(
    input  [31:0] PC,
    input  [31:0] fetch_instruction,
    output [31:0] instruction
);
    assign instruction = fetch_instruction;
endmodule


module tinker_core (
    input logic clk,
    input logic reset,
    output logic hlt
);
    parameter PC_RESET_ADDR = 32'h2000;
    // IF: fetch instruction from memory
    // ID: decode, register read, early branch decision
    // EX: ALU/FPU ops, branch target math
    // MEM: data memory access
    // WB: register writeback

    // IF stage -> program counter + instruction fetch
    reg [31:0] pc_F; // pc value in IF
    reg [31:0] pc_next; // pc after selection mux

    // update pc each cycle unless stalling
    // stall freezes both IF and ID so pc does not change
    logic stall;
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_F <= PC_RESET_ADDR; // start address after reset
        else if (!stall) 
            pc_F <= pc_next; // normal sequence
    end

    // instruction memory port for fetch
    reg [31:0] instr_F; // instructed fetched in IF
    reg [63:0] dummy_dload; // unused in IF
    reg mem_we; // in MEM for stores
    reg [31:0] mem_addr_W; // address for store
    reg [63:0] mem_data_W; // data to store

    memory memory (
        .clk(clk),
        .reset(reset),
        .fetch_addr(pc_F),
        .fetch_instruction(instr_F),
        .data_load_addr(mem_addr_W),
        .data_load(dummy_dload),
        .store_we(mem_we),
        .store_addr(mem_addr_W),
        .store_data(mem_data_W)
    );

    // IF/ID pipeline register --> holds values entering ID stage
    reg [31:0] pc_IFID; // pc of fetched instruction
    reg [31:0] instr_IFID; // fetched instruction bits

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_IFID <= 1'b0;
            // NOP encoding --> mov r0 r0 (opcode 0x11, rd=rs=rt=0)
            instr_IFID <= 32'h22000000;
        end
        else if (!stall) begin // freeze when hazard unit says so
            pc_IFID <= pc_F;
            instr_IFID <= instr_F;
        end
    end

    // ID stage --> decode, register reads, early branch evaluation
    reg [4:0] op_ID, rd_ID, rs_ID, rt_ID; // decode instruction fields
    reg [11:0] L_ID; // 12-bit literal

    instruction_decoder decode (
        .in(instr_IFID),
        .opcode(op_ID),
        .rd(rd_ID),
        .rs(rs_ID),
        .rt(rt_ID),
        .L(L_ID)
    );

    // register read file ports
    reg [63:0] rf_rdata_rs; // rs value
    reg [63:0] rf_rdata_rt; // rt value
    reg [63:0] rf_rdata_rd; // rd value

    // writeback channel signals from WB stage
    logic [4:0]  wb_dest;
    assign wb_dest      = MEMWB.rdDest;
    reg [63:0] wb_write_data = MEMWB.ctrl.memToReg
                       ? MEMWB.memData
                       : MEMWB.aluResult;
    reg wb_we = MEMWB.ctrl.regWrite;

    regFile reg_file (
        .clk(clk),
        .reset(reset),
        .data_in(wb_write_data),
        .we(wb_we),
        .wrAddr(wb_dest),
        .rd(rd_ID),
        .rs(rs_ID),
        .rt(rt_ID),
        .rdOut(rf_rdata_rd),
        .rsOut(rf_rdata_rs),
        .rtOut(rf_rdata_rt)
    );

    // control signal struct
    typedef struct packed {
        logic regWrite; // writeback to regFile in WB stage
        logic memRead; // load instruction in MEM stage
        logic memWrite; // store instruction in MEM stage
        logic memToReg;
        logic isLoad; // hazard detection
        logic isStore;
        logic isBranch; // early branch decison/flush
        logic isJump; // call/return
        logic isFPU; // is floating point op?
        logic halt;
    } id_ctrl_t;

    id_ctrl_t id_ctrl; // control bundle produced in ID

    always @(*) begin
        // preset all bits to 0
        id_ctrl = '0;

        if (op_ID == 5'hf) begin
            id_ctrl.halt = 1'b1;
        end

         case (op_ID)
        // ALU --> result goes to rd
        5'h18,5'h19,5'h1a,5'h1b,
        5'h1c,5'h1d,
        5'h0,5'h1,5'h2,5'h3,
        5'h4,5'h5,5'h6,5'h7,
        5'h11,5'h12,5'h14,5'h15,5'h16,5'h17 : begin
            id_ctrl.regWrite = 1;
            id_ctrl.isFPU = (op_ID >= 5'h14 && op_ID <= 5'h17);
        end
        
        // load --> mov rd (rs)(L)
        5'h10: begin
            id_ctrl.regWrite = 1;
            id_ctrl.memRead = 1;
            id_ctrl.memToReg = 1;
            id_ctrl.isLoad = 1;
        end

        // store --> mov (rd)(L), rs
        5'h13: begin
            id_ctrl.memWrite = 1;
            id_ctrl.isStore = 1;
        end

        // branches/jumps/call/return
        5'h8, 5'h9, 5'hb, 5'ha, 5'he : id_ctrl.isBranch = 1; // unconditional/conditional branch
        5'hc, 5'hd : id_ctrl.isJump = 1; // call / return
        endcase
    end

    // early branch/jump decision still in ID
    logic take_branch_ID; // 1 --> branch is taken this cycle
    logic [31:0] branch_target_ID; // next PC when branch is taken

    always @(*) begin
        // default not taken --> next pc = pc + 4
        take_branch_ID = 1'b0;
        branch_target_ID = pc_IFID + 4; 

        case (op_ID) 
        // br rd : PC = register[rd]
        5'h8: begin
            take_branch_ID = 1'b1;
            branch_target_ID = rf_rdata_rd;
        end

        // brr rd : PC = PC + register[rd]
        5'h9: begin
            take_branch_ID = 1'b1;
            branch_target_ID = pc_IFID + rf_rdata_rd[31:0];
        end

        // brr L : PC = PC + signextend(L) << 0
        5'ha: begin
            take_branch_ID = 1'b1;
            branch_target_ID = pc_IFID + $signed({{20{L_ID[11]}},L_ID});
        end

        // brnz rd, rs : if(rs!=0) PC = register[rd]
        5'hb: if (rf_rdata_rs != 0) begin
            take_branch_ID = 1'b1;
            branch_target_ID = rf_rdata_rd;
        end

        // brgt rd, rs, rt: if (rs >rt), _C = register[rd]
        5'he: if ($signed(rf_rdata_rs) > $signed(rf_rdata_rt)) begin
            take_branch_ID = 1'b1;
            branch_target_ID = rf_rdata_rd;
        end

        // call rd, rs, rt : push returnAddr, jump to rd
        5'hc: begin
            take_branch_ID = 1'b1;
            branch_target_ID = rf_rdata_rd; // subroutine target
        end

        // return: jump to address in r31 (rs)
        5'hd: begin
            take_branch_ID = 1'b1;
            branch_target_ID = rf_rdata_rs;
        end
        endcase
    end

    // pc selection/pipeline flush
    logic flush_ID; // 1 --> squash instruction in IF and ID
    assign flush_ID = take_branch_ID;
    assign pc_next = flush_ID ? branch_target_ID : (pc_F + 4);

    // ID/EX pipeline register --> bundle all useful values into struct
    typedef struct packed {
        id_ctrl_t ctrl; // control bits for later stages
        logic [4:0] opcode;
        logic [4:0] rd, rs, rt; // register numbers
        logic [11:0] L; // literal immediate
        logic [31:0] pc; // pc of this instruction for debugging
        logic [63:0] rdVal;
        logic [63:0] rsVal; // value of rs after register read/forwarding
        logic [63:0] rtVal; // value of rt
    } idex_t;

    idex_t IDEX; // actual pipeline reg
    idex_t IDEX_in; // next cycle value

    // build IDEX_in each cycle
    always @(*) begin
        IDEX_in.ctrl = id_ctrl;
        IDEX_in.opcode = op_ID;
        IDEX_in.rd = rd_ID;
        IDEX_in.rs = rs_ID;
        IDEX_in.rt = rt_ID;
        IDEX_in.L = L_ID;
        IDEX_in.pc = pc_IFID;
        IDEX_in.rdVal = rf_rdata_rd;
        IDEX_in.rsVal = rf_rdata_rs;
        IDEX_in.rtVal = rf_rdata_rt;

        // hazard stall inserts a bubble --> handled below
    end

    // register update with stall/flushing
    always @(posedge clk or posedge reset) begin
        if (reset)
            IDEX <= '0; // clear pipeline
        else if (stall) begin
            IDEX <= '0; // bubble (NOP)
        end
        else if (flush_ID) begin
            IDEX <= '0; // squash after taken branch
        end
        else 
            IDEX <= IDEX_in; // normal advance
    end

    hazard_unit hazard (
        .idex_memRead(IDEX.ctrl.memRead),
        .idex_rd     (IDEX.rd),
        .ifid_rs     (rs_ID),
        .ifid_rt     (rt_ID),
        .stall       (stall)
    );


    // EX --> ALU/FPU, operand forwarding
    // forwarding selection muxes
    logic [1:0] selA, selB; // select codes for opA/opB muxes

    forward_unit fwd (
        .exmem_rd(EXMEM.rdDest),
        .exmem_regwrite(EXMEM.ctrl.regWrite),
        .memwb_rd(MEMWB.rdDest),
        .memwb_regwrite(MEMWB.ctrl.regWrite),
        .rs_IDEX(IDEX.rs),
        .rt_IDEX(IDEX.rt),
        .sel_opA(selA),
        .sel_opB(selB)
    );

    // operand multplexers
    logic [63:0] opA_EX; // final opA into ALU
    logic [63:0] opB_EX; // final opB into ALU

    always @(*) begin
        // select opA
        case (selA)
        2'd1: opA_EX = EXMEM.aluResult; // forward from EX/MEM stage
        2'd2: opA_EX = wb_write_data; // forward from MEM/WB stage\
        default: opA_EX = IDEX.rsVal; // from regFile
        endcase

        // select opB
         case (selB)
        2'd1: opB_EX = EXMEM.aluResult; 
        2'd2: opB_EX = wb_write_data;
        default: opB_EX = IDEX.rtVal;
        endcase
    end
    // ALU
    logic [63:0] alu_out_EX;

    alu alu (
        .opcode(IDEX.opcode),
        .rdData(IDEX.rdVal),
        .rsData(opA_EX),
        .rtData(opB_EX),
        .L(IDEX.L),
        .result(alu_out_EX)
    );

    // EX/MEM pipeline register --> carries results into MEM stage
    typedef struct packed {
        id_ctrl_t ctrl; // control bits for MEM/WB
        logic [63:0] aluResult; // ALU/FPU output
        logic [63:0] rtVal; // value to store 
        logic [4:0] rdDest; // destination register number
        logic [31:0] pc; // debug / tracing
    } exmem_t;

    exmem_t EXMEM; // actual pipeline register
    exmem_t EXMEM_in; // computed next value;

    always @(*) begin
        EXMEM_in.ctrl = IDEX.ctrl;
        EXMEM_in.aluResult = alu_out_EX;
        EXMEM_in.rtVal = opB_EX; // store value travels here
        EXMEM_in.rdDest = IDEX.rd; // same rd throughout
        EXMEM_in.pc = IDEX.pc;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) 
            EXMEM <= '0;
        else 
            EXMEM <= EXMEM_in;
    end

    // MEM stage --> data memory read/write
    // interfaces to unficed memory
    always @(*) begin
         mem_we = EXMEM.ctrl.memWrite; // 1 on stores
        mem_addr_W = EXMEM.aluResult[31:0]; // byte address
        mem_data_W = EXMEM.rtVal;
    end

    // for loads, memory returns data on same clock
    logic [63:0] mem_rdata_M; 
    assign mem_rdata_M = dummy_dload;

    // MEM/WB pipeline register --> final stage before writeback
    typedef struct packed {
        id_ctrl_t ctrl;
        logic [63:0] memData; // data loaded from memory if load
        logic [63:0] aluResult; // ALU result if not load
        logic [4:0] rdDest; // destination register number
    } memwb_t;

    memwb_t MEMWB;
    memwb_t MEMWB_in;

    always @(*) begin
        MEMWB_in.ctrl = EXMEM.ctrl;
        MEMWB_in.memData = mem_rdata_M; // valid if load
        MEMWB_in.aluResult = EXMEM.aluResult; // valid otherwise
        MEMWB_in.rdDest = EXMEM.rdDest;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) MEMWB <= '0;
        else MEMWB <= MEMWB_in;
    end

    // WB stage --> choose result and write to register file
    always @(*) begin
        wb_write_data = MEMWB.ctrl.memToReg ? MEMWB.memData : MEMWB.aluResult;
        wb_we = MEMWB.ctrl.regWrite; // assert if rd is valid
        hlt = MEMWB.ctrl.halt;
    end
   

    // halt detection
   
endmodule

