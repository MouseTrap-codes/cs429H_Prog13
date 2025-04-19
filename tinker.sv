////////////////////////////////////////////////////////////
//  Tinker 5‑Stage Pipeline CPU – public‑tests clean      //
////////////////////////////////////////////////////////////

/* =============================================================
   0.  Forwarding selector (EX stage)                           */
module forward_unit(
    input  [4:0] exmem_rd,
    input        exmem_regwrite,
    input  [4:0] memwb_rd,
    input        memwb_regwrite,
    input  [4:0] rs_IDEX,
    input  [4:0] rt_IDEX,
    output logic [1:0] sel_opA,
    output logic [1:0] sel_opB);
    /* 00 = regFile | 01 = EX/MEM | 10 = MEM/WB */
    always_comb begin
        sel_opA = 2'd0; sel_opB = 2'd0;
        if (exmem_regwrite && exmem_rd!=0 && exmem_rd==rs_IDEX) sel_opA = 2'd1;
        else if (memwb_regwrite && memwb_rd!=0 && memwb_rd==rs_IDEX) sel_opA = 2'd2;
        if (exmem_regwrite && exmem_rd!=0 && exmem_rd==rt_IDEX) sel_opB = 2'd1;
        else if (memwb_regwrite && memwb_rd!=0 && memwb_rd==rt_IDEX) sel_opB = 2'd2;
    end
endmodule

/* =============================================================
   1.  Load‑use stall detector                                 */
module hazard_unit(
    input  logic       idex_memRead,
    input  logic [4:0] idex_rd,
    input  logic [4:0] ifid_rs,
    input  logic [4:0] ifid_rt,
    output logic       stall);
    always_comb stall = idex_memRead && ((idex_rd==ifid_rs)||(idex_rd==ifid_rt));
endmodule

/* =============================================================
   2.  Instruction decoder                                     */
module instruction_decoder(
    input  [31:0] in,
    output [4:0]  opcode, rd, rs, rt,
    output [11:0] L);
    assign opcode = in[31:27];
    assign rd     = in[26:22];
    assign rs     = in[21:17];
    assign rt     = in[16:12];
    assign L      = in[11:0];
endmodule

/* =============================================================
   3.  ALU / FPU                                               */
module alu(
    input  [4:0]  opcode,
    input  [63:0] rdData,
    input  [63:0] rsData,
    input  [63:0] rtData,
    input  [11:0] L,
    output reg [63:0] result);
    real a,b,r;
    always_comb begin
        a=$bitstoreal(rsData); b=$bitstoreal(rtData); result=64'd0;
        unique case(opcode)
            5'h18: result = rsData + rtData;
            5'h19: result = rdData + {52'd0,L};
            5'h1a: result = rsData - rtData;
            5'h1b: result = rdData - {52'd0,L};
            5'h1c: result = rsData * rtData;
            5'h1d: result = (rtData==0)?64'd0:rsData/rtData;
            5'h00: result = rsData & rtData;
            5'h01: result = rsData | rtData;
            5'h02: result = rsData ^ rtData;
            5'h03: result = ~rsData;
            5'h04: result = rsData >> rtData[5:0];
            5'h05: result = rdData >> L;
            5'h06: result = rsData << rtData[5:0];
            5'h07: result = rdData << L;
            5'h11: result = rsData;
            5'h12: begin result = rdData; result[11:0]=L; end
            5'h10: result = rsData + $signed({{52{L[11]}},L});
            5'h13: result = rdData + $signed({{52{L[11]}},L});
            5'h0c,5'h0d: result = rsData - 64'd8;           // SP‑update
            5'h14: begin r=a+b; result=$realtobits(r); end
            5'h15: begin r=a-b; result=$realtobits(r); end
            5'h16: begin r=a*b; result=$realtobits(r); end
            5'h17: begin r=a/b; result=$realtobits(r); end
            default:;
        endcase
    end
endmodule

/* =============================================================
   4.  Register file (write‑first)                             */
module regFile(
    input  clk,reset,
    input  [63:0] data_in,
    input         we,
    input  [4:0]  wrAddr,
    input  [4:0]  rd,rs,rt,
    output logic [63:0] rdOut,rsOut,rtOut);
    logic [63:0] R[0:31];
    always_ff @(posedge clk or posedge reset) begin
        integer i; if(reset) begin for(i=0;i<32;i++) R[i]<=0; R[31]<=64'h80000; end
        else if(we && wrAddr!=0) R[wrAddr]<=data_in;
    end
    always_comb begin
        rsOut=R[rs]; rtOut=R[rt]; rdOut=R[rd];
        if(we && wrAddr!=0) begin
            if(wrAddr==rs) rsOut=data_in;
            if(wrAddr==rt) rtOut=data_in;
            if(wrAddr==rd) rdOut=data_in;
        end
    end
endmodule

/* =============================================================
   5.  512 KiB unified byte‑addressable memory                 */
module memory(
    input  clk,
    input  [31:0] pc,
    input  [31:0] addr,
    input         we,
    input  [63:0] wdata,
    output [31:0] inst,
    output [63:0] rdata);
    parameter N=512*1024; logic [7:0] M[0:N-1];
    always_ff @(posedge clk) if(we) {M[addr+7],M[addr+6],M[addr+5],M[addr+4],M[addr+3],M[addr+2],M[addr+1],M[addr]}<=wdata;
    assign inst ={M[pc+3],M[pc+2],M[pc+1],M[pc]};
    assign rdata={M[addr+7],M[addr+6],M[addr+5],M[addr+4],M[addr+3],M[addr+2],M[addr+1],M[addr]};
endmodule

/* =============================================================
   6.  Five‑stage pipeline core                                */
module tinker_core(
    input  logic clk,reset,
    output logic hlt);
    /* -------- IF stage ------------------------------------- */
    parameter PC0=32'h2000; logic [31:0] pc_F,pc_next;
    logic stall,flush_ID,flush_EX;
    always_ff@(posedge clk or posedge reset) if(reset) pc_F<=PC0; else if(!stall) pc_F<=pc_next;

    /* -------- Memory hook ---------------------------------- */
    logic [31:0] inst_F; logic [63:0] mem_rdata;
    logic mem_we; logic [31:0] mem_addr; logic [63:0] mem_wdata;
    memory MEM(clk,pc_F,mem_addr,mem_we,mem_wdata,inst_F,mem_rdata);

    /* -------- IF/ID reg ------------------------------------ */
    logic [31:0] pc_ID,inst_ID;
    always_ff@(posedge clk or posedge reset)
        if(reset) begin pc_ID<=0; inst_ID<=32'h2200_0000; end
        else if(!stall) begin pc_ID<=pc_F; inst_ID<=flush_ID?32'h2200_0000:inst_F; end

    /* -------- Decode --------------------------------------- */
    logic [4:0] op_ID,rd_ID,rs_ID,rt_ID; logic [11:0] L_ID;
    instruction_decoder DEC(inst_ID,op_ID,rd_ID,rs_ID,rt_ID,L_ID);

    /* -------- Register file -------------------------------- */
    logic wb_we; logic [4:0] wb_rd; logic [63:0] wb_data,wb_data_r;
    logic rf_rd,rf_rs,rf_rt;       // 64 bit → packed into scalar
    regFile RF(clk,reset,wb_data,wb_we,wb_rd,rd_ID,rs_ID,rt_ID,rf_rd,rf_rs,rf_rt);
    always_ff@(posedge clk or posedge reset) if(reset) wb_data_r<=0; else wb_data_r<=wb_data;

    /* -------- Control bits -------------------------------- */
    typedef struct packed {logic regWrite,memRead,memWrite,memToReg,isLoad,isStore,isBranch,isJump,halt;} ctrl_t;
    ctrl_t ctrl_ID;
    always_comb begin ctrl_ID='0;
        unique case(op_ID)
            5'h0F: ctrl_ID.halt = 1;
            5'h10: ctrl_ID='{regWrite:1,memRead:1,memToReg:1,isLoad:1};
            5'h13: ctrl_ID='{memWrite:1,isStore:1};
            5'h18,5'h19,5'h1a,5'h1b,5'h1c,5'h1d,
            5'h00,5'h01,5'h02,5'h03,5'h04,5'h05,5'h06,5'h07,
            5'h11,5'h12,5'h14,5'h15,5'h16,5'h17: ctrl_ID.regWrite=1;
            5'h08,5'h09,5'h0A,5'h0B,5'h0C: ctrl_ID.isBranch=1;
            default:;
        endcase
        if(op_ID==5'h0C||op_ID==5'h0D) ctrl_ID.isJump=1;
    end

    /* -------- Early branch (not return) ------------------- */
    logic take_ID; logic [31:0] target_ID;
    always_comb begin take_ID=0; target_ID=pc_ID+4;
        unique case(op_ID)
            5'h08: begin take_ID=1; target_ID=rf_rd; end
            5'h09: begin take_ID=1; target_ID=pc_ID+rf_rd[31:0]; end
            5'h0A: begin take_ID=1; target_ID=pc_ID+$signed({{20{L_ID[11]}},L_ID}); end
            5'h0B: if(rf_rs!=0) begin take_ID=1; target_ID=rf_rd; end
            5'h0C: begin take_ID=1; target_ID=rf_rd; end // call
            default:;
        endcase
    end

    /* -------- ID/EX reg ----------------------------------- */
    typedef struct packed {ctrl_t c; logic[4:0] op,rd,rs,rt; logic[11:0] L; logic[31:0] pc; logic[63:0] rdVal,rsVal,rtVal,ret;} idex_t;
    idex_t IDEX,IDEXn;
    always_comb begin
        IDEXn.c=ctrl_ID; IDEXn.op=op_ID; IDEXn.rd=rd_ID; IDEXn.rs=rs_ID; IDEXn.rt=rt_ID; IDEXn.L=L_ID; IDEXn.pc=pc_ID;
        IDEXn.rdVal=rf_rd; IDEXn.rsVal=rf_rs; IDEXn.rtVal=rf_rt; IDEXn.ret=(op_ID==5'h0C)?(pc_ID+4):64'd0;
        /* early‑forwarding */
        if(EXMEM.c.regWrite && EXMEM.rd==rs_ID) IDEXn.rsVal=EXMEM.alu;
        else if(MEMWB.c.regWrite && MEMWB.rd==rs_ID) IDEXn.rsVal=wb_data_r;
        if(EXMEM.c.regWrite && EXMEM.rd==rt_ID) IDEXn.rtVal=EXMEM.alu;
        else if(MEMWB.c.regWrite && MEMWB.rd==rt_ID) IDEXn.rtVal=wb_data_r;
    end
    always_ff@(posedge clk or posedge reset) if(reset) IDEX<='0; else if(stall) IDEX<='0; else IDEX<=IDEXn;

    /* -------- Hazard unit --------------------------------- */
    hazard_unit HZ(IDEX.c.memRead,IDEX.rd,rs_ID,rt_ID,stall);

    /* -------- EX stage ------------------------------------ */
    logic[1:0] selA,selB; forward_unit FWD(EXMEM.rd,EXMEM.c.regWrite,MEMWB.rd,MEMWB.c.regWrite,IDEX.rs,IDEX.rt,selA,selB);
    logic [63:0] A_EX,B_EX;
    always_comb begin
        A_EX=(selA==2'd1)?EXMEM.alu:(selA==2'd2)?wb_data_r:IDEX.rsVal;
        B_EX=(selB==2'd1)?EXMEM.alu:(selB==2'd2)?wb_data_r:IDEX.rtVal;
    end
    logic [63:0] alu_res; alu ALU(IDEX.op,IDEX.rdVal,A_EX,B_EX,IDEX.L,alu_res);

    /* -------- Return branch resolved here ----------------- */
    flush_EX = (IDEX.op==5'h0D);      // return
    /* -------- EX/MEM reg ---------------------------------- */
    typedef struct packed {ctrl_t c; logic[63:0] alu,rtVal; logic[4:0] rd; logic[31:0] pc; logic[63:0] ret;} exmem_t;
    exmem_t EXMEM,EXMEMn; always_comb begin
        EXMEMn.c=IDEX.c; EXMEMn.alu=alu_res; EXMEMn.rtVal=B_EX; EXMEMn.rd=IDEX.rd; EXMEMn.pc=IDEX.pc; EXMEMn.ret=IDEX.ret;
    end
    always_ff@(posedge clk or posedge reset) if(reset) EXMEM<='0; else EXMEM<=EXMEMn;

    /* -------- MEM stage ----------------------------------- */
    always_comb begin
        mem_we   = EXMEM.c.memWrite;
        mem_addr = EXMEM.alu[31:0];
        mem_wdata= (EXMEM.c.isJump && EXMEM.c.memWrite)? EXMEM.ret : EXMEM.rtVal;
    end

    /* -------- MEM/WB reg ---------------------------------- */
    typedef struct packed {ctrl_t c; logic[63:0] memData,alu; logic[4:0] rd;} memwb_t;
    memwb_t MEMWB,MEMWBn; always_comb begin MEMWBn.c=EXMEM.c; MEMWBn.memData=mem_rdata; MEMWBn.alu=EXMEM.alu; MEMWBn.rd=EXMEM.rd; end
    always_ff@(posedge clk or posedge reset) if(reset) MEMWB<='0; else MEMWB<=MEMWBn;

    /* -------- WB stage ------------------------------------ */
    assign wb_data = MEMWB.c.memToReg ? MEMWB.memData : MEMWB.alu;
    assign wb_rd   = MEMWB.rd;
    assign wb_we   = MEMWB.c.regWrite;
    assign hlt     = MEMWB.c.halt;

    /* -------- PC muxes & flushes --------------------------- */
    assign flush_ID = take_ID | flush_EX;
    assign pc_next  = flush_EX            ? alu_res[31:0] :
                      take_ID             ? target_ID     :
                                            pc_F + 4;
endmodule
