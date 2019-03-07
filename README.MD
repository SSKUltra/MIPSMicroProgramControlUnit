#MicroProgram Instruction Fields (36 bits)
- ALU(3b) => aluCtrl(3b)
- ALUSrc1(2b) => aluSrcA(2b)
- ALUSrc2(3b) => aluSrcB(3b)
- RegCtrl(6b) => memtoReg(3b), regDst(2b), regWr(1b)
- mAdderCtrl(1b) => maluOp(1b)
- hiCtrl(3b) => hiSel(2b), hiWr(1b)
- loCtrl(3b) => loSel(2b), loWr(1b)
- MemCtrl(4b) => IorD(1b), memRd(1b), memWr(1b), IRWr(1b)
- PCCtrl(5b) => PCWr(1b), PCWrCondi(1b), PCSrc(3b)
- ExceptionCtrl(3b) => CauseSet(1b), CauseWr(1b), EPCWr(1b)
- AddrCtrl(3b) => AddrCtrl(3b)

#MicroProgram Instruction format
- Total length = 36bits
```
aluCtrl(3b)_aluSrcA(2b)_aluSrcB(3b)_RegCtrl(6b)_mAdderCtrl(1b)_hiCtrl(3b)_loCtrl(3b)_MemCtrl(4b)_PCCtrl(5b)_ExceptionCtrl(3b)_AddrCtrl(3b)
```

#ALU controls
- add = 000
- sub = 001
- mult = 010
- div = 011
- sll = 100
- or = 101

#mALU controls
- add = 0
- sub = 1

#memtoReg controls
- aluOut0 = 000
- MDR = 001
- PC = 010
- lui = 011
- Hi = 100
- Lo = 101

#regDst controls
- RegRd = 00
- RegRt = 01
- RedR31 = 10

#PCSrc controls
- aluOut0 = 000
- BTA = 001
- jump target address = 010
- RegRs = 011
- Exception Handling Routine = 100

#ExceptionCtrl bits
- CauseSet(1b) selects the input of the cause register. Given to a 2x1 MUX.
- CauseWr(1b) Write signal for cause register.
- EPCWr(1b) Write signal for EPC register.

#AddrCtrl values
- sequential = 000
- fetch = 001
- WriteBack(ADD) = 010
- Dispatch Table 1 = 011
- Dispatch Table 2 = 100
- Dispatch Table 3 = 101
- Dispatch Table 4 = 110 

#Sequence of Instructions in Testbench
note : For I-type and J-type Instruction "funcField" is set to don't care values.
- mfhi
- mflo
- mthi
- mtlo
- lui
- beq
- jump
- jal
- jr
- jalr
- add
- add with overflow
- sll
- sllv
- div
- mult
- madd
- msub
- addi
- lw
- sw
- ori
- Undefined Instruction

#MicroProgram Instruction entries fetched from "microinstructions.tv" file
```
000_00_000_xxxxx0_x_xx0_xx0_0101_10000_x00_000  //IF
000_00_010_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_011  //ID
xxx_xx_xxx_100001_x_xx0_xx0_xx00_00xxx_x00_001  //mfhi WB
xxx_xx_xxx_101001_x_xx0_xx0_xx00_00xxx_x00_001  //mflo WB  
xxx_xx_xxx_xxxxx0_x_011_xx0_xx00_00xxx_x00_001  //mthi WB
xxx_xx_xxx_xxxxx0_x_xx0_011_xx00_00xxx_x00_001  //mtlo WB
xxx_xx_xxx_011011_x_xx0_xx0_xx00_00xxx_x00_001  //lui WB
001_01_001_xxxxx0_x_xx0_xx0_xx00_01001_x00_001  //beq WB
xxx_xx_xxx_xxxxx0_x_xx0_xx0_xx00_10010_x00_001  //jump WB
xxx_xx_xxx_010101_x_xx0_xx0_xx00_10010_x00_001  //jal WB
xxx_xx_xxx_xxxxx0_x_xx0_xx0_xx00_10011_x00_001  //jr WB
xxx_xx_xxx_010001_x_xx0_xx0_xx00_10011_x00_001  //jalr WB
000_01_001_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_000  //add EX
xxx_xx_xxx_000001_x_xx0_xx0_xx00_00xxx_x00_110  //add WB
100_10_001_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_010  //sll EX
100_01_001_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_010  //sllv EX
011_01_001_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_000  //div EX
xxx_xx_xxx_xxxxx0_x_001_001_xx00_00xxx_x00_001  //div WB
010_01_001_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_100  //mult,madd,msub EX
000_01_011_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_101  //addi,sw,lw EX
101_01_100_xxxxx0_x_xx0_xx0_xx00_00xxx_x00_000  //ori EX
xxx_xx_xxx_000011_x_xx0_xx0_xx00_00xxx_x00_110  //ori WB
xxx_xx_xxx_xxxxx0_0_101_101_xx00_00xxx_x00_001  //madd WB
xxx_xx_xxx_xxxxx0_1_101_101_xx00_00xxx_x00_001  //msub WB
xxx_xx_xxx_xxxxx0_x_xx0_xx0_1100_00xxx_x00_000  //lw MEM
xxx_xx_xxx_001011_x_xx0_xx0_xx00_00xxx_x00_001  //lw WB
xxx_xx_xxx_xxxxx0_x_xx0_xx0_1x10_00xxx_x00_001  //sw MEM
xxx_xx_xxx_xxxxx0_x_xx0_xx0_xx00_00xxx_010_000  //Undefined Instruction Update cause register
001_00_000_xxxxx0_x_xx0_xx0_xx00_00xxx_x01_000  //Update EPC register
xxx_xx_xxx_xxxxx0_x_xx0_xx0_xx00_10100_x00_001  //Update PC to exception handling routine address ponited by PCSrcMux input 4
xxx_xx_xxx_xxxxx0_x_xx0_xx0_xx00_00xxx_110_111  //Overflow detected Update cause register
```

#Dispatch Table 1 entries fetched from "dispatchtable1.tv" file
```
000000_000000_00000000000000000000000000010001  //number of entries
000000_010000_00000000000000000000000000000010  //mfhi EX/WB
000000_010010_00000000000000000000000000000011  //mflo EX/WB
000000_010001_00000000000000000000000000000100  //mthi WB
000000_010011_00000000000000000000000000000101  //mtlo WB
001111_xxxxxx_00000000000000000000000000000110  //lui WB
000100_xxxxxx_00000000000000000000000000000111  //beq WB
000010_xxxxxx_00000000000000000000000000001000  //jump WB
000011_xxxxxx_00000000000000000000000000001001  //jal WB
000000_001000_00000000000000000000000000001010  //jr WB
000000_001001_00000000000000000000000000001011  //jalr WB
000000_100000_00000000000000000000000000001100  //add EX
000000_000000_00000000000000000000000000001110  //sll EX
000000_000100_00000000000000000000000000001111  //sllv EX
000000_011010_00000000000000000000000000010000  //div EX
000000_011000_00000000000000000000000000010010  //mult EX
011100_000000_00000000000000000000000000010010  //madd EX
011100_000100_00000000000000000000000000010010  //msub EX
001000_xxxxxx_00000000000000000000000000010011  //addi EX
100011_xxxxxx_00000000000000000000000000010011  //lw EX
101011_xxxxxx_00000000000000000000000000010011  //sw EX
001101_xxxxxx_00000000000000000000000000010100  //ori EX
```

#Dispatch Table 2 entries fetched from "dispatchtable2.tv" file
```
000000_000000_00000000000000000000000000000011  //number of entries
000000_011000_00000000000000000000000000010001  //mult WB
011100_000000_00000000000000000000000000010110  //madd WB
011100_000100_00000000000000000000000000010111  //msub WB
```

#Dispatch Table 3 entries fetched from "dispatchtable3.tv" file
```
000000_000000_00000000000000000000000000000011  //number of entries
001000_xxxxxx_00000000000000000000000000010101  //addi WB
100011_xxxxxx_00000000000000000000000000011000  //lw WB
101011_xxxxxx_00000000000000000000000000011010  //sw WB
```

#Dispatch Table 4 entries fetched from "dispatchtable4.tv" file
```
000000_000000_00000000000000000000000000000010    //number of entries
000000_100000_00000000000000000000000000011110    //add overflow
001000_xxxxxx_00000000000000000000000000011110    //addi overflow
```