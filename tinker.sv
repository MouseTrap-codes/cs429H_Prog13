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
                registers[rd] <= data_in;
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

module control(
    input         clk,
    input         reset,
    input  [31:0] instruction,
    input  [31:0] PC,
    input  [63:0] rdData,
    input  [63:0] rsData,         // Data from regFile port A (rs)
    input  [63:0] rtData,         // Data from regFile port B ()
    input  [63:0] data_load,   // Data loaded from memory
    output reg [31:0] next_PC,
    output reg [63:0] exec_result,
    output reg        write_en,
    // Register file read addresses:
    output reg[4:0]   rf_addrRd,
    output reg [4:0]  rf_addrRs,
    output reg [4:0]  rf_addrRt,
    // Memory store signals:
    output reg        mem_we,
    output reg [31:0] mem_addr,
    output reg [63:0] mem_write_data,
    // Data load address for load instructions:
    output reg [31:0] data_load_addr,
    // halt
    output reg hlt
);
    // multicycle stuff
    // enum list of states
    typedef enum logic [2:0] {
    FETCH          = 3'd0,
    DECODE         = 3'd1,
    ALU            = 3'd2,
    L_S            = 3'd3,
    REGISTER_WRITE = 3'd4,
    HALT = 3'd5
    } state_t;
    
    state_t curr_state, next_state;

    reg [31:0] IR; // latches fetched instruction
    reg [63:0] ALU_output; // hold ALU output for access across cycles

    reg halt_detected;
    // for two of the brr stuff
    reg [31:0] jump_target;
    reg jump_pending;

    // for the one memory thing that won't work
    reg [63:0] mem_data_latched;

    
    // update state and halt detection
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            curr_state <= FETCH;
            IR <= 32'd0;
            halt_detected <= 1'b0;
            jump_target <= 32'd0;
            jump_pending <= 1'b0;
        end else begin
            curr_state <= next_state;
            if (curr_state == FETCH)
                IR <= instruction;
            
            if (curr_state == ALU)
                ALU_output <= alu_out;
            
            if (curr_state == L_S && opcode == 5'h10)
                mem_data_latched <= data_load;
            
            // once halt is detected, keep it set
            if (curr_state == DECODE && opcode == 5'hf)
                halt_detected <= 1'b1;

            // latch jump target 
            if (curr_state == DECODE && (opcode == 5'ha || opcode == 5'h9 || opcode == 5'hc)) begin
                case (opcode)
                5'ha: jump_target <= PC + $signed({{52{L[11]}}, L});
                5'h9: jump_target <= PC + rdData[31:0];
                5'hc: jump_target <= rdData; // for call
                endcase
                jump_pending <= 1'b1;
            end else begin
                jump_pending <= 1'b0;
            end
        end
    end


    // Decode the instruction.
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] L;
    instruction_decoder dec (
       .in(IR),
       .opcode(opcode),
       .rd(rd),
       .rs(rs),
       .rt(rt),
       .L(L)
    );

    
    // Instantiate ALU and FPU.
    wire [63:0] alu_out;
    alu alu_inst (
       .opcode(opcode),
       .rdData(rdData),
       .rsData(rsData),
       .rtData(rtData),
       .L(L),
       .result(alu_out)
    );
    

   
    
    // Set register file read addresses.
    // Default: use rs for opA and rt for opB.
    // Then override for specific opcodes.
    always @(*) begin
        // to account for call and return
        if (opcode == 5'hc || opcode == 5'hd)
            rf_addrRs = 5'd31;
        else
            rf_addrRs = rs;

         rf_addrRd = rd;
         rf_addrRt = rt;
    end

   

    // Main control logic.
    always @(*) begin
         // Default assignments.
         next_state = curr_state; // hold next_state until curr_state ends
         next_PC         = PC + 4;
         exec_result     = 64'b0;
         write_en        = 1'b0;
         mem_we          = 1'b0;
         mem_addr        = 32'b0;
         mem_write_data  = 64'b0;
         data_load_addr  = 32'b0;
         hlt = halt_detected;

         if (hlt == 1'b1) begin
            curr_state = HALT;
         end 


         case (curr_state) 
         FETCH: begin
            if (opcode == 5'hf) begin
                hlt = 1'b1;
                next_PC = PC;
                next_state = HALT;
            end else if (jump_pending) begin
                next_PC = jump_target;
                jump_pending = 0;
                next_state = DECODE; // resume after jump
            end else begin
                next_PC = PC + 4;
                next_state = DECODE;
            end
         end 

         DECODE: begin
            if (opcode ==  5'h18 || opcode == 5'h1a || opcode == 5'h1c || opcode == 5'h1d
            || opcode == 5'h0 || opcode == 5'h1 || opcode == 5'h2 || opcode == 5'h3
            || opcode == 5'h4 || opcode == 5'h6 ||
                opcode == 5'h19 || opcode == 5'h1b || opcode == 5'h5
                || opcode == 5'h7 || opcode == 5'h12 || opcode == 5'h14
                || opcode == 5'h15 || opcode == 5'h16 || opcode == 5'h17
                || opcode == 5'h11) begin
                    next_state = ALU;
                end  else if (opcode == 5'h10 || opcode == 5'h13) begin
                    next_state = L_S;
                end 
            else if (opcode == 5'hf) begin
                hlt = 1'b1;
                next_PC = PC;
                next_state = HALT;
            end

             // Branch instructions:// Branch instructions:
            else if (opcode == 5'h8) begin // br rd: PC = register[rd]
                next_PC = rdData; 
                next_state = FETCH;
            end
            else if (opcode == 5'h9) begin // brr rd: PC = PC + register[rd]
                //next_PC = PC + rdData[31:0];
                next_state = FETCH;
            end
            else if (opcode == 5'ha) begin // brr L: PC = PC + sign-extended L
                jump_target = PC + $signed({{52{L[11]}}, L});
                jump_pending = 1;
                next_state = FETCH;
            end
            else if (opcode == 5'hb) begin // brnz rd, rs: if (register[rs] != 0) then PC = register[rd]
                if (rsData != 0)
                    next_PC = rdData; // opB = register rd
                else
                    next_PC = PC + 4;
                next_state = FETCH;
            end
            else if (opcode == 5'hc) begin // call rd, rs, rt:
                mem_we         = 1'b1;
                mem_addr       = rsData - 8;  // opB = register 31
                mem_write_data = PC;
                next_PC        = rdData;      // opA = register rd
                next_state = FETCH;
            end
            else if (opcode == 5'hd) begin // return:
                // For return, read r31 and then set load address to (r31 - 8)
                data_load_addr = rsData - 8;  // opA = register 31
                next_PC        = data_load[31:0];
                next_state = FETCH;
            end
            else if (opcode == 5'he) begin // brgt rd, rs, rt: if (register[rs] > register[rt]) then PC = register[rd]
                if ($signed(rsData) > $signed(rtData))
                    next_PC = rdData; // opB = register rd (after override)
                else
                    next_PC = PC + 4;
                next_state = FETCH;
            end
        end
       
       ALU: begin
        // for arithmetic instructions:
        if (opcode ==  5'h18 || opcode == 5'h1a || opcode == 5'h1c || opcode == 5'h1d
         || opcode == 5'h0 || opcode == 5'h1 || opcode == 5'h2 || opcode == 5'h3
         || opcode == 5'h4 || opcode == 5'h6 ||
            opcode == 5'h19 || opcode == 5'h1b || opcode == 5'h5
            || opcode == 5'h7 || opcode == 5'h12 || opcode == 5'h14
            || opcode == 5'h15 || opcode == 5'h16 || opcode == 5'h17) begin
                next_state = REGISTER_WRITE;
        end 

        else if (opcode == 5'h11) begin // mov rd, rs: move register.
                exec_result = rsData;
                next_state = REGISTER_WRITE;
            end
        end

        L_S: begin
        if (opcode == 5'h10) begin // mov rd, (rs)(L): load 64-bit word from memory.
                data_load_addr = rsData + {20'b0, L}; // opA = register rs
                // exec_result    = data_load;
                next_state = REGISTER_WRITE;
            end
        
         else if (opcode == 5'h13) begin // mov (rd)(L), rs: store 64-bit word.
                mem_we         = 1'b1;
                mem_addr       = rdData + {20'b0, L}; // opA = register rd (base)
                mem_write_data = rsData;             // opB = register rs (value)
                next_state = FETCH;
            end
        end

        REGISTER_WRITE: begin
            if (opcode == 5'h10) begin
                // this is a memory load. write the memory data we latched earlier
                exec_result = mem_data_latched;
            end 
            else begin
                exec_result = ALU_output; // latched!!!
            end
            write_en = 1'b1;
            next_state = FETCH;
        end
        
        // plsplspls halt
        HALT: begin
            hlt = 1'b1;
            next_PC = PC;   // don't advance
            next_state = HALT; // remain forever
        end
        
        endcase
    end
endmodule

//---------------------------------------------------------------------
// tinker_core Module (Top Level)
//---------------------------------------------------------------------
module tinker_core(
    input clk,
    input reset,
    output logic hlt
);

    
    
    // Program Counter (PC) register.
    reg [31:0] PC;
    
    // Instantiate memory module.
    // Instance name is "memory" (as expected by the testbench).
    wire [31:0] fetch_instruction;
    wire [63:0] data_load;
    wire [31:0] mem_data_load_addr;
    wire        mem_we;
    wire [31:0] mem_store_addr;
    wire [63:0] mem_store_data;
    
    memory memory (
        .clk(clk),
        .reset(reset),
        .fetch_addr(PC),
        .fetch_instruction(fetch_instruction),
        .data_load_addr(mem_data_load_addr),
        .data_load(data_load),
        .store_we(mem_we),
        .store_addr(mem_store_addr),
        .store_data(mem_store_data)
    );
    
    // Instantiate fetch module.
    wire [31:0] instruction;
    fetch fetch_inst (
        .PC(PC),
        .fetch_instruction(fetch_instruction),
        .instruction(instruction)
    );
    
    // Instantiate register file (instance name: reg_file).
    wire [4:0]  rfAddrRd;
    wire [4:0]  rfAddrRs;
    wire [4:0]  rfAddrRt;
    wire [63:0] rdData, rsData, rtData;
    reg  [63:0] write_data;
    reg         write_en;
    
    regFile reg_file (
        .clk(clk),
        .reset(reset),
        .data_in(write_data),
        .we(write_en),
        .rd(rfAddrRd),
        .rs(rfAddrRs),
        .rt(rfAddrRt),
        .rdOut(rdData),
        .rsOut(rsData),
        .rtOut(rtData)
    );
    
    // Instantiate control module.
    wire [31:0] next_PC;
    wire [63:0] exec_result;
    wire        ctrl_write_en;
    wire [4:0]  ctrl_rfAddrRd, ctrl_rfAddrRs, ctrl_rfAddrRt;
    wire        ctrl_mem_we;
    wire [31:0] ctrl_mem_addr;
    wire [63:0] ctrl_mem_write_data;
    wire [31:0] ctrl_data_load_addr;

    wire ctrl_hlt;
    
    control ctrl_inst (
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .PC(PC),
        .rdData(rdData),
        .rsData(rsData),
        .rtData(rtData),
        .data_load(data_load),
        .next_PC(next_PC),
        .exec_result(exec_result),
        .write_en(ctrl_write_en),
        .rf_addrRd(ctrl_rfAddrRd),
        .rf_addrRs(ctrl_rfAddrRs),
        .rf_addrRt(ctrl_rfAddrRt),
        .mem_we(ctrl_mem_we),
        .mem_addr(ctrl_mem_addr),
        .mem_write_data(ctrl_mem_write_data),
        .data_load_addr(ctrl_data_load_addr),
        .hlt(ctrl_hlt)
    );

    reg plsHaltIBegU;


    always @(posedge clk or posedge reset) begin
        if (reset)
            plsHaltIBegU <= 1'd0;
        else if (ctrl_hlt) 
            plsHaltIBegU <= 1'd1;
    end


    // latch hlt 
    assign hlt = plsHaltIBegU || ctrl_hlt;
    
    // Connect control outputs to register file and memory.
    assign rfAddrRd = ctrl_rfAddrRd;
    assign rfAddrRs = ctrl_rfAddrRs;
    assign rfAddrRt = ctrl_rfAddrRt;
    
    always @(*) begin
        write_data = exec_result;
        write_en   = ctrl_write_en;
    end
    
    assign mem_we             = ctrl_mem_we;
    assign mem_store_addr     = ctrl_mem_addr;
    assign mem_store_data     = ctrl_mem_write_data;
    assign mem_data_load_addr = ctrl_data_load_addr;
    
    // PC update.
    always @(posedge clk) begin
        if (reset) begin
            PC <= 32'h2000;
        end else if (!hlt) begin 
            PC <= next_PC;
        end
    end
endmodule