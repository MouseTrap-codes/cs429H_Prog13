`timescale 1ns/1ps

module tb_tinker_core;

  // Testbench signals.
  reg         clk;
  reg         reset;
  wire        hlt;

  // Instantiate the Tinker core (the top-level module is assumed to be named tinker_core).
  tinker_core uut (
    .clk(clk),
    .reset(reset),
    .hlt(hlt)
  );

  // Clock generator: a simple clock with a 10 ns period.
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  //--------------------------------------------------------------------------
  // Task: write_instruction
  // Writes a 32-bit instruction word to the processor's internal memory.
  // The memory is implemented as an 8-bit array (big-endian order):
  //   fetch_instruction = { bytes[addr+3], bytes[addr+2],
  //                         bytes[addr+1], bytes[addr] }
  //--------------------------------------------------------------------------
  task write_instruction;
    input integer addr;
    input [31:0] instr;
    begin
      uut.memory.bytes[addr]     = instr[7:0];
      uut.memory.bytes[addr+1]   = instr[15:8];
      uut.memory.bytes[addr+2]   = instr[23:16];
      uut.memory.bytes[addr+3]   = instr[31:24];
    end
  endtask

  //--------------------------------------------------------------------------
  // Task: load_program
  // Loads an array of 32-bit instruction words (a program) into memory,
  // starting at address 0. The array "prog" holds at most 32 instructions.
  //--------------------------------------------------------------------------
  task load_program;
    input [31:0] prog [0:31];
    input integer num_instr;
    integer i;
    begin
      for (i = 0; i < num_instr; i = i + 1)
        write_instruction(i * 4, prog[i]);
    end
  endtask

  //--------------------------------------------------------------------------
  // Force the processor's PC to start at 0 (instead of its default 0x2000).
  //--------------------------------------------------------------------------
  initial begin
    force uut.PC = 0;
  end

  //--------------------------------------------------------------------------
  // Test Case 1: Test "mov rd, L" (opcode 0x12)
  //  - Program: mov R1, 10   (expected: R1 = 10)
  //--------------------------------------------------------------------------
  initial begin
    #1;
    $display("==============================================");
    $display("Test Case 1: mov R1, 10 (mov rd, L)");
    reg [31:0] prog0 [0:1];
    prog0[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10};  // Format: {opcode, rd, rs, rt, L}
    prog0[1] = {5'hF, 27'd0};                       // Halt: privileged instruction with opcode 0xF
    load_program(prog0, 2);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R1 = %h (expected 10)", uut.reg_file.registers[1]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 2: Test "addi" (opcode 0x19)
  //  - Program: mov R2,20; then addi R2,5 (expected: R2 = 25)
  //--------------------------------------------------------------------------
  initial begin
    #100;
    $display("==============================================");
    $display("Test Case 2: addi R2, 5 (after mov R2,20)");
    reg [31:0] prog1 [0:2];
    prog1[0] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd20}; // mov R2,20
    prog1[1] = {5'h19, 5'd2, 5'd0, 5'd0, 12'd5};   // addi R2,5   --> 20+5=25
    prog1[2] = {5'hF, 27'd0};                      // Halt
    load_program(prog1, 3);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R2 = %h (expected 25)", uut.reg_file.registers[2]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 3: Test "add" (opcode 0x18)
  //  - Program: mov R1,10; mov R2,25; add R3,R1,R2 (expected: R3 = 35)
  //--------------------------------------------------------------------------
  initial begin
    #200;
    $display("==============================================");
    $display("Test Case 3: add R3, R1, R2 (10 + 25)");
    reg [31:0] prog2 [0:3];
    prog2[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10}; // R1 = 10
    prog2[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd25}; // R2 = 25
    prog2[2] = {5'h18, 5'd3, 5'd1, 5'd2, 12'd0};  // add R3, R1, R2
    prog2[3] = {5'hF, 27'd0};                     // Halt
    load_program(prog2, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R3 = %h (expected 35)", uut.reg_file.registers[3]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 4: Test "sub" (opcode 0x1A)
  //  - Program: mov R1,10; mov R2,25; sub R4,R2,R1 (expected: R4 = 15)
  //--------------------------------------------------------------------------
  initial begin
    #300;
    $display("==============================================");
    $display("Test Case 4: sub R4, R2, R1 (25 - 10)");
    reg [31:0] prog3 [0:3];
    prog3[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10}; // R1 = 10
    prog3[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd25}; // R2 = 25
    prog3[2] = {5'h1A, 5'd4, 5'd2, 5'd1, 12'd0};  // sub R4, R2, R1 --> 25-10=15
    prog3[3] = {5'hF, 27'd0};                     // Halt
    load_program(prog3, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R4 = %h (expected 15)", uut.reg_file.registers[4]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 5: Test "subi" (opcode 0x1B)
  //  - Program: mov R4,15; then subi R4,5 (expected: R4 = 10)
  //--------------------------------------------------------------------------
  initial begin
    #400;
    $display("==============================================");
    $display("Test Case 5: subi R4, 5 (15 - 5 = 10)");
    reg [31:0] prog4 [0:2];
    prog4[0] = {5'h12, 5'd4, 5'd0, 5'd0, 12'd15}; // R4 = 15
    prog4[1] = {5'h1B, 5'd4, 5'd0, 5'd0, 12'd5};   // subi R4,5   --> 15-5=10
    prog4[2] = {5'hF, 27'd0};                      // Halt
    load_program(prog4, 3);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R4 = %h (expected 10)", uut.reg_file.registers[4]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 6: Test "mul" (opcode 0x1C)
  //  - Program: mov R1,10; mov R2,25; mul R5,R1,R2 (expected: R5 = 250)
  //--------------------------------------------------------------------------
  initial begin
    #500;
    $display("==============================================");
    $display("Test Case 6: mul R5, R1, R2 (10 * 25)");
    reg [31:0] prog5 [0:3];
    prog5[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10}; // R1 = 10
    prog5[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd25}; // R2 = 25
    prog5[2] = {5'h1C, 5'd5, 5'd1, 5'd2, 12'd0};  // mul R5, R1, R2 --> 10*25=250
    prog5[3] = {5'hF, 27'd0};                     // Halt
    load_program(prog5, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R5 = %h (expected 250)", uut.reg_file.registers[5]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 7: Test "div" (opcode 0x1D)
  //  - Program: mov R1,10; mov R2,25; div R6,R2,R1 (expected: R6 = 2)
  //--------------------------------------------------------------------------
  initial begin
    #600;
    $display("==============================================");
    $display("Test Case 7: div R6, R2, R1 (25 / 10)");
    reg [31:0] prog6 [0:3];
    prog6[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10}; // R1 = 10
    prog6[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd25}; // R2 = 25
    prog6[2] = {5'h1D, 5'd6, 5'd2, 5'd1, 12'd0};  // div R6, R2, R1
    prog6[3] = {5'hF, 27'd0};                     // Halt
    load_program(prog6, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R6 = %h (expected 2)", uut.reg_file.registers[6]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 8a: Test "and" (opcode 0x0)
  //  - Program: mov R1,10; mov R2,25; and R7,R1,R2 (expected: R7 = 10 & 25 = 8)
  //--------------------------------------------------------------------------
  initial begin
    #700;
    $display("==============================================");
    $display("Test Case 8a: and R7, R1, R2 (10 & 25)");
    reg [31:0] prog7 [0:3];
    prog7[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10};
    prog7[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd25};
    prog7[2] = {5'h0, 5'd7, 5'd1, 5'd2, 12'd0};
    prog7[3] = {5'hF, 27'd0};
    load_program(prog7, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R7 = %h (expected 8)", uut.reg_file.registers[7]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 8b: Test "or" (opcode 0x1)
  //  - Program: mov R1,10; mov R2,25; or R8,R1,R2 (expected: R8 = 10 | 25 = 27)
  //--------------------------------------------------------------------------
  initial begin
    #800;
    $display("==============================================");
    $display("Test Case 8b: or R8, R1, R2 (10 | 25)");
    reg [31:0] prog8 [0:3];
    prog8[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10};
    prog8[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd25};
    prog8[2] = {5'h1, 5'd8, 5'd1, 5'd2, 12'd0};
    prog8[3] = {5'hF, 27'd0};
    load_program(prog8, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R8 = %h (expected 27)", uut.reg_file.registers[8]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 8c: Test "xor" (opcode 0x2)
  //  - Program: mov R1,10; mov R2,25; xor R9,R1,R2 (expected: R9 = 10 xor 25 = 19)
  //--------------------------------------------------------------------------
  initial begin
    #900;
    $display("==============================================");
    $display("Test Case 8c: xor R9, R1, R2 (10 xor 25)");
    reg [31:0] prog9 [0:3];
    prog9[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10};
    prog9[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd25};
    prog9[2] = {5'h2, 5'd9, 5'd1, 5'd2, 12'd0};
    prog9[3] = {5'hF, 27'd0};
    load_program(prog9, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R9 = %h (expected 19)", uut.reg_file.registers[9]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 8d: Test "not" (opcode 0x3)
  //  - Program: mov R1,10; not R10,R1 (expected: R10 = ~10)
  //--------------------------------------------------------------------------
  initial begin
    #1000;
    $display("==============================================");
    $display("Test Case 8d: not R10, R1 (bitwise NOT of 10)");
    reg [31:0] prog10 [0:2];
    prog10[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10};
    prog10[1] = {5'h3, 5'd10, 5'd1, 5'd0, 12'd0};
    prog10[2] = {5'hF, 27'd0};
    load_program(prog10, 3);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R10 = %h (expected ~10)", uut.reg_file.registers[10]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 9a: Test "shftr" (opcode 0x4)
  //  - Program: mov R12,255; mov R15,2; shftr R11,R12,R15 (expected: R11 = 255 >> 2 = 63)
  //--------------------------------------------------------------------------
  initial begin
    #1100;
    $display("==============================================");
    $display("Test Case 9a: shftr R11, R12, R15 (255 >> 2 = 63)");
    reg [31:0] prog11 [0:3];
    prog11[0] = {5'h12, 5'd12, 5'd0, 5'd0, 12'd255};
    prog11[1] = {5'h12, 5'd15, 5'd0, 5'd0, 12'd2};
    prog11[2] = {5'h4, 5'd11, 5'd12, 5'd15, 12'd0};
    prog11[3] = {5'hF, 27'd0};
    load_program(prog11, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R11 = %h (expected 63)", uut.reg_file.registers[11]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 9b: Test "shftri" (opcode 0x5)
  //  - Program: mov R16,240; shftri R16,L with L=4 (expected: R16 = 240 >> 4 = 15)
  //--------------------------------------------------------------------------
  initial begin
    #1200;
    $display("==============================================");
    $display("Test Case 9b: shftri R16, L (240 >> 4 = 15)");
    reg [31:0] prog12 [0:2];
    prog12[0] = {5'h12, 5'd16, 5'd0, 5'd0, 12'd240};
    prog12[1] = {5'h5, 5'd16, 5'd0, 5'd0, 12'd4};
    prog12[2] = {5'hF, 27'd0};
    load_program(prog12, 3);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R16 = %h (expected 15)", uut.reg_file.registers[16]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 9c: Test "shftl" (opcode 0x6)
  //  - Program: mov R12,255; mov R15,2; shftl R17,R12,R15 (expected: R17 = 255 << 2 = 1020)
  //--------------------------------------------------------------------------
  initial begin
    #1300;
    $display("==============================================");
    $display("Test Case 9c: shftl R17, R12, R15 (255 << 2 = 1020)");
    reg [31:0] prog13 [0:3];
    prog13[0] = {5'h12, 5'd12, 5'd0, 5'd0, 12'd255};
    prog13[1] = {5'h12, 5'd15, 5'd0, 5'd0, 12'd2};
    prog13[2] = {5'h6, 5'd17, 5'd12, 5'd15, 12'd0};
    prog13[3] = {5'hF, 27'd0};
    load_program(prog13, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R17 = %h (expected 1020)", uut.reg_file.registers[17]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 9d: Test "shftli" (opcode 0x7)
  //  - Program: mov R18,15; shftli R18,L with L=2 (expected: R18 = 15 << 2 = 60)
  //--------------------------------------------------------------------------
  initial begin
    #1400;
    $display("==============================================");
    $display("Test Case 9d: shftli R18, L (15 << 2 = 60)");
    reg [31:0] prog14 [0:2];
    prog14[0] = {5'h12, 5'd18, 5'd0, 5'd0, 12'd15};
    prog14[1] = {5'h7, 5'd18, 5'd0, 5'd0, 12'd2};
    prog14[2] = {5'hF, 27'd0};
    load_program(prog14, 3);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: R18 = %h (expected 60)", uut.reg_file.registers[18]);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 10a: Test "br" (opcode 0x8)
  //  - Program: mov R20,8; then br R20 (branch to address 8)
  //--------------------------------------------------------------------------
  initial begin
    #1500;
    $display("==============================================");
    $display("Test Case 10a: br rd (branch using register)");
    reg [31:0] prog15 [0:2];
    prog15[0] = {5'h12, 5'd20, 5'd0, 5'd0, 12'd8};
    prog15[1] = {5'h8, 5'd20, 5'd0, 5'd0, 12'd0};
    prog15[2] = {5'hF, 27'd0};
    load_program(prog15, 3);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: br rd executed (PC should be 8)");
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 10b: Test "brr rd" (opcode 0x9)
  //  - Program: mov R21,4; then brr R21 (expected: PC = current PC + 4)
  //--------------------------------------------------------------------------
  initial begin
    #1600;
    $display("==============================================");
    $display("Test Case 10b: brr rd (branch relative using register)");
    reg [31:0] prog16 [0:2];
    prog16[0] = {5'h12, 5'd21, 5'd0, 5'd0, 12'd4};
    prog16[1] = {5'h9, 5'd21, 5'd0, 5'd0, 12'd0};
    prog16[2] = {5'hF, 27'd0};
    load_program(prog16, 3);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: brr rd executed (PC should be 4)");
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 10c: Test "brr L" (opcode 0xA)
  //  - Program: brr L with L=4 (expected: PC = current PC + 4)
  //--------------------------------------------------------------------------
  initial begin
    #1700;
    $display("==============================================");
    $display("Test Case 10c: brr L (branch relative using literal 4)");
    reg [31:0] prog17 [0:1];
    prog17[0] = {5'hA, 5'd0, 5'd0, 5'd0, 12'd4};
    prog17[1] = {5'hF, 27'd0};
    load_program(prog17, 2);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: brr L executed (PC should become 4)");
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 10d: Test "brnz" (opcode 0xB)
  //  - Program: mov R1,10; mov R22,8; then brnz R22, R1 (expected: branch taken to address 8)
  //--------------------------------------------------------------------------
  initial begin
    #1800;
    $display("==============================================");
    $display("Test Case 10d: brnz rd, rs (branch if R1 != 0)");
    reg [31:0] prog18 [0:3];
    prog18[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd10};
    prog18[1] = {5'h12, 5'd22, 5'd0, 5'd0, 12'd8};
    prog18[2] = {5'hB, 5'd22, 5'd1, 5'd0, 12'd0};
    prog18[3] = {5'hF, 27'd0};
    load_program(prog18, 4);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: brnz executed (PC should become 8)");
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 10e: Test "call" (opcode 0xC) and "return" (opcode 0xD)
  //  - Program: mov R1,12; call R1; (skipped instruction); return; Halt.
  //    (The call instruction stores PC+4 in memory at (R31-8).)
  //--------------------------------------------------------------------------
  initial begin
    #1900;
    $display("==============================================");
    $display("Test Case 10e: call and return");
    reg [31:0] prog19 [0:4];
    prog19[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd12};
    prog19[1] = {5'hC, 5'd1, 5'd0, 5'd0, 12'd0};
    prog19[2] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd55}; // This instruction should be skipped.
    prog19[3] = {5'hD, 27'd0};
    prog19[4] = {5'hF, 27'd0};
    load_program(prog19, 5);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: call/return executed (check memory at R31-8 for saved PC)");
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 10f: Test "brgt" (opcode 0xE)
  //  - Program: mov R1,20; mov R2,10; brgt R3,R1,R2; mov R3,8 (expected: branch to address 8)
  //--------------------------------------------------------------------------
  initial begin
    #2100;
    $display("==============================================");
    $display("Test Case 10f: brgt (branch if R1 > R2)");
    reg [31:0] prog20 [0:4];
    prog20[0] = {5'h12, 5'd1, 5'd0, 5'd0, 12'd20};
    prog20[1] = {5'h12, 5'd2, 5'd0, 5'd0, 12'd10};
    prog20[2] = {5'hE, 5'd3, 5'd1, 5'd2, 12'd0};
    prog20[3] = {5'h12, 5'd3, 5'd0, 5'd0, 12'd8};
    prog20[4] = {5'hF, 27'd0};
    load_program(prog20, 5);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    $display("Result: brgt executed (PC should become 8)");
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 11a: Test "addf" (opcode 0x14, floating-point)
  //  - Floating point test: 1.5 + 2.5 = 4.0.
  //--------------------------------------------------------------------------
  initial begin
    #2300;
    $display("==============================================");
    $display("Test Case 11a: addf R3, R1, R2 (1.5 + 2.5)");
    reg [31:0] prog21 [0:1];
    prog21[0] = {5'h14, 5'd3, 5'd1, 5'd2, 12'd0};
    prog21[1] = {5'hF, 27'd0};
    load_program(prog21, 2);
    uut.reg_file.registers[1] = $realtobits(1.5);
    uut.reg_file.registers[2] = $realtobits(2.5);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    real fresult;
    fresult = $bitstoreal(uut.reg_file.registers[3]);
    $display("Result: R3 (floating-point) = %f (expected 4.0)", fresult);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 11b: Test "subf" (opcode 0x15, floating-point)
  //  - Floating point test: 5.5 - 2.5 = 3.0.
  //--------------------------------------------------------------------------
  initial begin
    #2400;
    $display("==============================================");
    $display("Test Case 11b: subf R3, R1, R2 (5.5 - 2.5)");
    reg [31:0] prog22 [0:1];
    prog22[0] = {5'h15, 5'd3, 5'd1, 5'd2, 12'd0};
    prog22[1] = {5'hF, 27'd0};
    load_program(prog22, 2);
    uut.reg_file.registers[1] = $realtobits(5.5);
    uut.reg_file.registers[2] = $realtobits(2.5);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    real fresult;
    fresult = $bitstoreal(uut.reg_file.registers[3]);
    $display("Result: R3 (floating-point) = %f (expected 3.0)", fresult);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 11c: Test "mulf" (opcode 0x16, floating-point)
  //  - Floating point test: 2.0 * 3.0 = 6.0.
  //--------------------------------------------------------------------------
  initial begin
    #2500;
    $display("==============================================");
    $display("Test Case 11c: mulf R3, R1, R2 (2.0 * 3.0)");
    reg [31:0] prog23 [0:1];
    prog23[0] = {5'h16, 5'd3, 5'd1, 5'd2, 12'd0};
    prog23[1] = {5'hF, 27'd0};
    load_program(prog23, 2);
    uut.reg_file.registers[1] = $realtobits(2.0);
    uut.reg_file.registers[2] = $realtobits(3.0);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    real fresult;
    fresult = $bitstoreal(uut.reg_file.registers[3]);
    $display("Result: R3 (floating-point) = %f (expected 6.0)", fresult);
    #20;
  end

  //--------------------------------------------------------------------------
  // Test Case 11d: Test "divf" (opcode 0x17, floating-point)
  //  - Floating point test: 7.0 / 2.0 = 3.5.
  //--------------------------------------------------------------------------
  initial begin
    #2600;
    $display("==============================================");
    $display("Test Case 11d: divf R3, R1, R2 (7.0 / 2.0)");
    reg [31:0] prog24 [0:1];
    prog24[0] = {5'h17, 5'd3, 5'd1, 5'd2, 12'd0};
    prog24[1] = {5'hF, 27'd0};
    load_program(prog24, 2);
    uut.reg_file.registers[1] = $realtobits(7.0);
    uut.reg_file.registers[2] = $realtobits(2.0);
    reset = 1;
    #10;
    reset = 0;
    wait(hlt);
    #10;
    real fresult;
    fresult = $bitstoreal(uut.reg_file.registers[3]);
    $display("Result: R3 (floating-point) = %f (expected 3.5)", fresult);
    #20;
  end

  //--------------------------------------------------------------------------
  // End simulation.
  //--------------------------------------------------------------------------
  initial begin
    #3000;
    $display("==============================================");
    $display("All test cases completed.");
    $finish;
  end

endmodule
