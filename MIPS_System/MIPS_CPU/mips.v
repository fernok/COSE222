`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg;
  wire 		  wire_memwrite;
  wire        alusrc, regdst, regwrite, jump, jal, jumpr, branch;
  wire [3:0]  alucontrol;
  wire [11:0] outinstr;

  // Instantiate Controller
  controller c(
    .op         (outinstr[11:6]), 
		.funct      (outinstr[5:0]), 
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (wire_memwrite),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
		.jal		(jal),
		.jumpr		(jumpr),
		.branch		(branch),
		.alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
	.memwrite	(wire_memwrite),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
	.jal		(jal),
	.jumpr		(jumpr),
	.branch		(branch),
    .alucontrol (alucontrol),
    .pc         (pc),
    .in_instr   (instr),
    .aluout     (memaddr), 
    .out_writedata  (memwritedata),
    .readdata   (memreaddata),
	.outinstr	(outinstr),
	.out_memwrite (memwrite));

endmodule

module controller(input  [5:0] op, funct,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       alusrc,
                  output       regdst, regwrite,
                  output       jump,
				  output 	   jal,
				  output	   jumpr,
				  output	   branch,
                  output [3:0] alucontrol);

  wire [1:0] aluop;

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
	.jal	  (jal),
    .aluop    (aluop));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol),
	.jumpr		(jumpr));


endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump, jal,
               output [1:0] aluop);

  reg [11:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, jal, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 12'b001100000011; // Rtype
      6'b100011: controls <= #`mydelay 12'b101010010000; // LW
      6'b101011: controls <= #`mydelay 12'b100010100000; // SW
	  6'b000101, 										 // BNE
      6'b000100: controls <= #`mydelay 12'b100001000001; // BEQ
      6'b001000, 
      6'b001001: controls <= #`mydelay 12'b101010000000; // ADDI, ADDIU: only difference is exception
      // ###### HyungMin Kim : Start
	  6'b001010: controls <= #`mydelay 12'b101010000000; // SLTI
	  6'b001011: controls <= #`mydelay 12'b101010000000; // SLTIU
	  // ###### HyungMin Kim : End
	  6'b001101: controls <= #`mydelay 12'b001010000010; // ORI
      6'b001111: controls <= #`mydelay 12'b011010000000; // LUI
      6'b000010: controls <= #`mydelay 12'b000000001000; // J
	  6'b000011: controls <= #`mydelay 12'b001000001100; // JAL
      default:   controls <= #`mydelay 12'bxxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output 	[3:0] alucontrol,
			  output 	jumpr);
			  
  reg [4:0] controls;
  
  assign {jumpr, alucontrol} = controls;

  always @(*)
    case(aluop)
      3'b00: controls <= #`mydelay 5'b00010;  // add
      3'b01: controls <= #`mydelay 5'b01010;  // sub
      3'b10: controls <= #`mydelay 5'b00001;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: controls <= #`mydelay 5'b00010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: controls <= #`mydelay 5'b01010; // SUB, SUBU: only difference is exception
          6'b100100: controls <= #`mydelay 5'b00000; // AND
          6'b100101: controls <= #`mydelay 5'b00001; // OR
		  // ###### HyungMin Kim : Start
		  6'b101011: controls <= #`mydelay 5'b01100; // SLTU
		  // ###### HyungMin Kim : End
          6'b101010: controls <= #`mydelay 5'b01011; // SLT
		  6'b001000: controls <= #`mydelay 5'b10010; // JR
          default:   controls <= #`mydelay 5'b0xxxx; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, memwrite,
                input         alusrc, regdst,
                input         regwrite, jump, jal, jumpr, branch,
                input  [3:0]  alucontrol,
                output [31:0] pc,
                input  [31:0] in_instr,
                output [31:0] aluout, out_writedata,
                input  [31:0] readdata,
				output [11:0] outinstr,
				output out_memwrite);

  wire [4:0]  	writereg, jalresultreg;
  wire [31:0] 	pcnext, pcnextbr, ifpcplus4, pcbranch, result_or_pc, jalresultdata;
  wire [31:0] 	signimm, signimmsh, shiftedimm;
  wire [31:0] 	result;
  wire        	zero, pcsrc;
  
  // wires of id stage
  wire [31:0] 	instr, idpcplus4, idsrca, idwritedata, forwardwd;
  wire [31:0]	outdataA, outdataB;
  // wires of ex stage
  wire 			stop;
  wire 			exregdst, exalusrc, exbranch, exmemwrite, exjump, exjal, exjumpr, exregwrite, exmemtoreg;
  wire [3:0] 	exalucontrol;
  wire [4:0] 	exrs, exrt, exrd;
  wire [5:0] 	instr_for_branch;
  wire [31:0]	exaluout, expcplus4, writedata, srca, real_srca, real_srcb, srcb_candidate, outdataC;
  wire [31:0]	exsignimmsh, exshiftedimm;
  wire [1:0]	srcacontrol, srcbcontrol;
  // wires of mem stage
  wire 			memjump, memjal, memjumpr, memregwrite, memmemtoreg;
  wire [4:0]	memregwriteaddr, memrt;
  wire [25:0]	memjumptarget;
  wire [31:0]	mempcplus4, temp_writedata;
  // wires of wb stage
  wire			wbjal, wbregwrite, wbmemtoreg;
  wire [4:0]	wbregwriteaddr;
  wire [31:0]	wbaluout, wbreaddata, wbpcplus4, wbresult;
  
  // ###### HyungMin Kim : Start
  wire [3:0]	new_alucontrol;
  
  assign new_alucontrol = (instr[31:26] == 6'b001010) ? 4'b1011 : (instr[31:26] == 6'b001011 ? 4'b1100 : alucontrol);
  // ###### HyungMin Kim : End
  

  // branch outcome is resolved in EX stage
  assign pcsrc = exbranch & (instr_for_branch == 6'b000100 ? zero : !zero);
  

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
	.stop  (stop),
    .d     (result_or_pc),		// jr address or next address?
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (ifpcplus4));

  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
	
  adder pcadd2(
    .a (expcplus4),
    .b (exsignimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (ifpcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));
// jump target is resolved at EX stage
  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({ifpcplus4[31:28], exrs, exrt, exshiftedimm[15:0], 2'b00}),
    .s    (exjump),
    .y    (pcnext));

  mux2 #(32) jumprmux(
	.d0   (pcnext),				// next pc address
	.d1   (srca),				// address fetched by jr instruction
	.s    (exjumpr),
	.y    (result_or_pc));

  mux2 #(32) jaldatamux(		// data input to register: write data or pc+4?
    .d0   (result),		
	.d1   (wbpcplus4),
	.s    (wbjal),
	.y    (jalresultdata));
	
  mux2 #(5) jaladdrmux(			// address of register: write destination or $ra?
    .d0   (writereg),
	.d1   (5'b11111),			// $ra
	.s    (exjal),
	.y    (jalresultreg));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (wbregwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (wbregwriteaddr),
    .wd      (jalresultdata),
    .rd1     (idsrca),
    .rd2     (idwritedata));

  mux2 #(5) wrmux(
    .d0  (exrt),
    .d1  (exrd),
    .s   (exregdst),
    .y   (writereg));

  mux2 #(32) resmux(
    .d0 (wbaluout),
    .d1 (wbreaddata),
    .s  (wbmemtoreg),
    .y  (result));

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic

  mux2 #(32) srcbmux(
    .d0 (srcb_candidate),
    .d1 (exshiftedimm[31:0]),
    .s  (exalusrc),
    .y  (real_srcb));

  alu alu(
    .a       (real_srca),
    .b       (real_srcb),
    .alucont (exalucontrol),
    .result  (exaluout),
    .zero    (zero));
    
	
  ifid_ff ifid(
	.in_inst	(in_instr),
	.in_pc		(ifpcplus4),
	.clk		(clk),
	.reset		(reset),
	.stop		(stop),
	.pcsrc		(pcsrc),
	.jump		(exjump),
	.jumpr		(exjumpr),
	.out_inst	(instr),
	.module_out_inst (outinstr),
	.out_pc		(idpcplus4));

  idex_ff idex(
	// ###### HyungMin Kim : Start
	.in_control		({regdst, alusrc, new_alucontrol, branch, memwrite, jump, jal, jumpr, regwrite, memtoreg}),
	// ###### HyungMin Kim : End
	.in_regone		(outdataA),
	.in_regtwo		(outdataB),
	.in_rs			(instr[25:21]),
	.in_rt			(instr[20:16]),
	.in_rd			(instr[15:11]),
	.in_opcode		(instr[31:26]),
	.in_pc			(idpcplus4),
	.in_sl2			(signimmsh),
	.in_shiftedimm	(shiftedimm),
	.clk			(clk),
	.reset			(reset),
	.stop			(stop),
	.out_control	({exregdst, exalusrc, exalucontrol, exbranch, exmemwrite, exjump, exjal, exjumpr, exregwrite, exmemtoreg}),
	.out_regone		(srca),
	.out_regtwo		(writedata),
	.out_rs			(exrs),
	.out_rt			(exrt),
	.out_rd			(exrd),
	.out_opcode		(instr_for_branch),
	.out_pc			(expcplus4),
	.out_sl2		(exsignimmsh),
	.out_shiftedimm	(exshiftedimm));

  exmem_ff exmem(
	.in_control		({exmemwrite, exjump, exjal, exjumpr, exregwrite, exmemtoreg}), 
	.in_alurst		(exaluout),
	.in_writedata	(outdataC),
	.in_regdst		(jalresultreg),
	.in_pc			(expcplus4),
	.in_exrt		(exrt),
	.clk			(clk),
	.reset			(reset),
	.out_control	({out_memwrite, memjump, memjal, memjumpr, memregwrite, memmemtoreg}),
	.out_alurst		(aluout),
	.out_writedata	(temp_writedata),
	.out_regdst		(memregwriteaddr),
	.out_pc			(mempcplus4),
	.out_memrt		(memrt));
	
  
  memwb_ff memwb(
	.in_control		({memjal, memregwrite, memmemtoreg}),
	.in_memdata		(readdata),
	.in_alurst		(aluout),
	.in_regdst		(memregwriteaddr),
	.in_pc			(mempcplus4),
	.clk			(clk),
	.reset			(reset),
	.out_control	({wbjal, wbregwrite, wbmemtoreg}),
	.out_memdata	(wbreaddata),
	.out_alurst		(wbaluout),
	.out_regdst		(wbregwriteaddr),
	.out_pc			(wbpcplus4));
  
  forwarding_unit fu(
	.exrs			(exrs),
	.exrt			(exrt),
	.idrs			(instr[25:21]),
	.idrt			(instr[20:16]),
	.memregaddr		(memregwriteaddr),
	.wbregaddr		(wbregwriteaddr),
	.memrt			(memrt),
	.memregwrite	(memregwrite),
	.wbregwrite		(wbregwrite),
	.exmemwrite		(exmemwrite),
	.memmemwrite	(out_memwrite),
	.regawritedata	(idsrca),
	.regbwritedata	(idwritedata),
	.wbwritedata	(jalresultdata),
	.exwritedata	(writedata),
	.temp_writedata	(temp_writedata),
	.srcacontrol	(srcacontrol),
	.srcbcontrol	(srcbcontrol),
	.outdataA		(outdataA),
	.outdataB		(outdataB),
	.outdataC		(outdataC),
	.out_writedata	(out_writedata));

  mux3 #(32) srcamux(
	.d0				(srca),
	.d1				(aluout),
	.d2				(jalresultdata),
	.s				(srcacontrol),
	.y				(real_srca));

  mux3 #(32) srcb_candidate_mux(
	.d0				(writedata),
	.d1				(aluout),
	.d2				(jalresultdata),
	.s				(srcbcontrol),
	.y				(srcb_candidate));
	
  hazard_detection_unit hdu(
	.memtoreg		(exmemtoreg),
	.exregaddr		(exrt),
	.rs				(instr[25:21]),
	.rt				(instr[20:16]),
	.stop			(stop));

endmodule









