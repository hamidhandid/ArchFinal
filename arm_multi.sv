module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, Adr, 
           output logic        MemWrite);

  logic [31:0] PC, Instr, ReadData;
  
  // instantiate processor and shared memory
  arm arm(clk, reset, MemWrite, Adr, 
          WriteData, ReadData);
  mem mem(clk, MemWrite, Adr, WriteData, ReadData);
endmodule

module arm(input  logic        clk, reset,
           output logic        MemWrite,
           output logic [31:0] Adr, WriteData,
           input  logic [31:0] ReadData);

  logic [31:0] Instr;
  logic [3:0]  ALUFlags;
  logic        PCWrite, RegWrite, IRWrite, NoWrite;
  logic        AdrSrc;
  logic [1:0]  RegSrc, ALUSrcA, ALUSrcB, ImmSrc, ALUControl, ResultSrc;

  controller c(clk, reset, Instr[31:12], ALUFlags, 
               PCWrite, MemWrite, RegWrite, IRWrite, NoWrite,
               AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc,
               ImmSrc, ALUControl);
  datapath dp(clk, reset, Adr, WriteData, ReadData, Instr, ALUFlags,
              PCWrite, RegWrite, IRWrite,
              AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc,
              ImmSrc, ALUControl);
endmodule

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("memfile.dat",RAM);

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module controller(input  logic         clk,
                  input  logic         reset,
                  input  logic [31:12] Instr,
                  input  logic [3:0]   ALUFlags,
                  output logic         PCWrite,
                  output logic         MemWrite,
                  output logic         RegWrite,
                  output logic         IRWrite,
						output logic			NoWrite,
                  output logic         AdrSrc,
                  output logic [1:0]   RegSrc,
                  output logic [1:0]   ALUSrcA,
                  output logic [1:0]   ALUSrcB,
                  output logic [1:0]   ResultSrc,
                  output logic [1:0]   ImmSrc,
                  output logic [1:0]   ALUControl);
                  
  logic [1:0] FlagW;
  logic       PCS, NextPC, RegW, MemW;
  
  decode dec(clk, reset, Instr[27:26], Instr[25:20], Instr[15:12],
             FlagW, PCS, NextPC, RegW, MemW,
             IRWrite, NoWrite, AdrSrc, ResultSrc, 
             ALUSrcA, ALUSrcB, ImmSrc, RegSrc, ALUControl);
  condlogic cl(clk, reset, Instr[31:28], ALUFlags,
               FlagW, PCS, NextPC, RegW, MemW, NoWrite,
               PCWrite, RegWrite, MemWrite);
endmodule

module datapath(input  logic        clk, reset,
                output logic [31:0] Adr, WriteData,
                input  logic [31:0] ReadData,
                output logic [31:0] Instr,
                output logic [3:0]  ALUFlags,
                input  logic        PCWrite, RegWrite,
                input  logic        IRWrite,
                input  logic        AdrSrc, 
                input  logic [1:0]  RegSrc, 
                input  logic [1:0]  ALUSrcA, ALUSrcB, ResultSrc,
                input  logic [1:0]  ImmSrc, ALUControl);

	logic [31:0] PC;
	logic [31:0] ExtImm, SrcA, SrcB, Result;
	logic [31:0] Data, RD1, RD2, A, ALUResult, ALUOut;
	logic [3:0]  RA1, RA2;
	logic [31:0] shiftSignal;
	
	//Flip-Flop with enable
	flopenr #(32) pcreq (clk, reset, PCWrite, Result, PC);
	flopenr #(32) instrflp (clk, reset, IRWrite, ReadData, Instr);
	
	//Flip-Flop without enable
	flopr #(32) readdataflp (clk, reset,ReadData, Data);
	flopr #(32) rd1flp (clk, reset, RD1, A);
	flopr #(32) rd2flp (clk, reset, RD2, WriteData);
	flopr #(32) aluflp (clk, reset, ALUResult, ALUOut);
	
	//Multiplexers
	mux2 #(32) pcmux (PC, Result, AdrSrc, Adr);
	mux2 #(4) ra1mux (Instr[19:16], 4'b1111, RegSrc[0], RA1);
	mux2 #(4) ra2mux (Instr[3:0], Instr[15:12], RegSrc[1], RA2);
	mux2 #(32) aluSrcAmux (A, PC, ALUSrcA[0], SrcA);
	mux3 #(32) aluSrcBmux (shiftSignal, ExtImm, 32'd4, ALUSrcB, SrcB);
	mux3 #(32) resmux (ALUOut, Data, ALUResult, ResultSrc, Result);
	
	regfile rf (clk, RegWrite, RA1, RA2, Instr[15:12], Result, Result, RD1, RD2);
	
	Shifter shifter (WriteData, Instr[11:7], Instr[6:5], shiftSignal);
	
	alu alu (SrcA, SrcB, ALUControl, ALUResult, ALUFlags);
	
	extend ext (Instr[23:0], ImmSrc, ExtImm);
	
endmodule

module decode(input  logic       clk, reset,
              input  logic [1:0] Op,
              input  logic [5:0] Funct,
              input  logic [3:0] Rd,
              output logic [1:0] FlagW,
              output logic       PCS, NextPC, RegW, MemW,
              output logic       IRWrite, NoWrite, AdrSrc,
              output logic [1:0] ResultSrc, ALUSrcA, ALUSrcB, 
              output logic [1:0] ImmSrc, RegSrc, ALUControl);

  logic [4:0] Addr, romAddr, nextAddr;
  logic [17:0] controls;
  logic Branch, ALUOp;
  
  //Filp-flop for keeping current address
  flopr #(5) controlAddrReg(clk, reset, Addr, romAddr);
  
  // rom
  rom rom(romAddr, controls);
  // assigning control signals
  assign {NextPC, RegW, MemW, IRWrite, AdrSrc, ResultSrc, ALUSrcA, ALUSrcB, Branch, ALUOp, nextAddr} = controls;
    
  //SequencingLogic
  sequencingLogic SL (nextAddr, Op, Funct[5], Funct[0], Addr);
  
  //ALU Decoder
	logic AND, ADD, SUB;
	assign ADD = (Funct[4:1] == 4'b0100);
	assign SUB = (Funct[4:1] == 4'b0010) | (NoWrite);
	assign AND = (Funct[4:1] == 4'b0000);
	assign NoWrite = (Funct[4:1] == 4'b1010);
	assign ALUControl = (ALUOp) ? ( (ADD) ? (2'b00) : ( (SUB) ? (2'b01) : ( (AND) ? (2'b10) : (2'b11) ) ) ) : (2'b00);
		
	assign FlagW[1] = (Funct[0] & ALUOp);
	assign FlagW[0] = (Funct[0]) & (~ALUControl[1]) & (ALUOp);
	
  //Instr Decoder
  assign ImmSrc = Op;
  assign RegSrc[0] = (Op == 2'b10);
  assign RegSrc[1] = (Op == 2'b01);
  
  // PC Logic
  assign PCS = ((Rd == 4'b1111) & RegW) | (Branch);
  
endmodule

module sequencingLogic(input logic [4:0] addr, input logic [1:0] op, input logic Funct5, Funct0, output logic [4:0] nextAddr);
	logic br, mem, afterdecode, afterMemAdr;
	assign br = (op == 2'b10);
	assign mem = (op == 2'b01);
	assign afterdecode = (addr == 5'b10000);
	assign afterMemAdr = (addr == 5'b10001);
	
	//recognizing real next address
	assign nextAddr = (afterdecode) ? ( (mem) ? (5'b00010) : ( (br) ? (5'b01001) : ( (Funct5) ? (5'b00111) : (5'b00110) ) ) ) : 
												 ( (afterMemAdr) ? ( (Funct0) ? (5'b00011) : (5'b00101) ): (addr) );
endmodule

module rom(input logic [4:0] address,
				output logic [17:0] controls);
		always_comb
		// NextPC, RegW, MemW, IRWrite, AdrSrc, ResultSrc[1:0], ALUSrcA[1:0], ALUSrcB[1:0], Branch, ALUOp, NextAddr[4:0]
			case (address)
				// fetch
				5'b00000: controls = 18'b1_0_0_1_0_10_01_10_0_0_00001;
				// decode
				5'b00001: controls = 18'b0_0_0_0_0_10_01_10_0_0_10000; // 10000 for deciding after decode in SequencingLogic
				// MemAdr
				5'b00010: controls = 18'b0_0_0_0_0_00_00_01_0_0_10001; // 10001 for deciding after MemAdr in SequencingLogic
				// MemRead
				5'b00011: controls = 18'b0_0_0_0_1_00_00_00_0_0_00100;
				// MemWriteBack
				5'b00100: controls = 18'b0_1_0_0_0_01_00_00_0_0_00000;
				// MemWrite
				5'b00101: controls = 18'b0_0_1_0_1_00_00_00_0_0_00000;
				// ExecuteReg
				5'b00110: controls = 18'b0_0_0_0_0_00_00_00_0_1_01000;
				// ExecuteImm
				5'b00111: controls = 18'b0_0_0_0_0_00_00_01_0_1_01000;
				// AluWriteBack
				5'b01000: controls = 18'b0_1_0_0_0_00_00_00_0_0_00000;
				// Branch
				5'b01001: controls = 18'b0_0_0_0_0_10_00_01_1_0_00000;
				
				default: controls = 18'b1_0_0_1_0_10_01_10_0_0_00001; //fetch
			endcase
		
endmodule

module condlogic(input  logic       clk, reset,
                 input  logic [3:0] Cond,
                 input  logic [3:0] ALUFlags,
                 input  logic [1:0] FlagW,
                 input  logic       PCS, NextPC, RegW, MemW,
					  input  logic			NoWrite,
                 output logic       PCWrite, RegWrite, MemWrite);

  logic [1:0] FlagWrite;
  logic [3:0] Flags;
  logic       CondEx;
  logic CondExexcute;
  
  assign FlagWrite = (CondEx) ? (FlagW) : (2'b00);
  
  flopenr #(2)flagreg1(clk, reset, FlagWrite[1], ALUFlags[3:2], Flags[3:2]);
  flopenr #(2)flagreg0(clk, reset, FlagWrite[0], ALUFlags[1:0], Flags[1:0]);
  
  condcheck cc (Cond, Flags, CondEx);
  
  flopr #(1) condflp (clk, reset, CondEx, CondExexcute);
  
  assign PCWrite = (NextPC) | (PCS & CondExexcute);
  assign RegWrite = (RegW) & (CondExexcute) & (~NoWrite);
  assign MemWrite = MemW & CondExexcute;
  
endmodule

module condcheck (input logic [3:0] Cond,
						input logic [3:0] Flags,
						output logic CondEx);
						
logic neg, zero, carry, overflow, ge;
assign {neg, zero, carry, overflow} = Flags;
assign ge = (neg == overflow);

	always_comb
		case(Cond)
			4'b0000: CondEx = zero; // EQ
			4'b0001: CondEx = ~zero; // NE
			4'b0010: CondEx = carry; // CS
			4'b0011: CondEx = ~carry; // CC
			4'b0100: CondEx = neg; // MI
			4'b0101: CondEx = ~neg; // PL
			4'b0110: CondEx = overflow; // VS
			4'b0111: CondEx = ~overflow; // VC
			4'b1000: CondEx = carry & ~zero; // HI
			4'b1001: CondEx = ~(carry & ~zero); // LS
			4'b1010: CondEx = ge; // GE
			4'b1011: CondEx = ~ge; // LT
			4'b1100: CondEx = ~zero & ge; // GT
			4'b1101: CondEx = ~(~zero & ge); // LE
			4'b1110: CondEx = 1'b1; // Always
			default: CondEx = 1'bx; // undefined
		endcase
endmodule

module Shifter #(parameter N = 32)(input logic [N-1:0] in, input logic[4:0]Shamt, input logic[1:0]Type, output logic[N-1:0] out);
	always_comb
		case (Type)
			2'b00: out = in <<  Shamt; //LSL
			2'b01: out = in >>  Shamt; //LSR
			2'b10: out = in >>> Shamt; //ASR
			2'b11: out = (in >> Shamt) | (in << N - Shamt); //ROR
			default: out = in;
		endcase
endmodule

module flopr #(parameter WIDTH = 8) (input logic clk, reset,
													input logic [WIDTH-1:0] d,
													output logic [WIDTH-1:0] q);
	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else q <= d;
endmodule

module flopenr #(parameter WIDTH = 8) (input logic clk, reset, en,
													input logic [WIDTH-1:0] d,
													output logic [WIDTH-1:0] q);
													
	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else if (en) q <= d;
endmodule

module mux2 #(parameter WIDTH = 8) (input logic [WIDTH-1:0] d0, d1,
												input logic s,
												output logic [WIDTH-1:0] y);
	assign y = s ? d1 : d0;
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module extend (input logic [23:0] Instr,
					input logic [1:0] ImmSrc,
					output logic [31:0] ExtImm);
	always_comb
		case(ImmSrc)
			// 8-bit unsigned immediate
			2'b00: ExtImm = {24'b0, Instr[7:0]};
			// 12-bit unsigned immediate
			2'b01: ExtImm = {20'b0, Instr[11:0]};
			// 24-bit two's complement shifted branch
			2'b10: ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm = 32'bx; // undefined
		endcase
endmodule

module alu (input logic [31:0] a, b,
				input logic [1:0] ALUControl,
				output logic [31:0] Result,
				output logic [3:0] ALUFlags);
	logic neg, zero, Carry, overflow;
	logic [31:0] condinvb;
	logic [32:0] sum;
	
	assign condinvb = ALUControl[0] ? ~b : b;
	assign sum = a + condinvb + ALUControl[0];
	
	always_comb
		casex (ALUControl[1:0])
			2'b0?: Result = sum;
			2'b10: Result = a & b;
			2'b11: Result = a | b;
		endcase
	assign neg = Result[31];
	assign zero = (Result == 32'b0);
	assign Carry = (ALUControl[1] == 1'b0) & sum[32];
	assign overflow = (ALUControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);
	assign ALUFlags = {neg, zero, Carry, overflow};
endmodule

module regfile(input logic clk,
					input logic we3,
					input logic [3:0] ra1, ra2, wa3,
					input logic [31:0] wd3, r15,
					output logic [31:0] rd1, rd2);
					
	logic [31:0] rf[14:0];
	// three ported register file
	// read two ports combinationally
	// write third port on rising edge of clock
	// register 15 reads PC + 8 instead
	always_ff @(posedge clk)
		if (we3) rf[wa3] <= wd3;
	assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
	assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];
endmodule
