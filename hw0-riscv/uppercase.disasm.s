
uppercase.bin:     file format elf32-littleriscv


Disassembly of section .text:

00010074 <_start>:
   10074:	ffff2517          	auipc	a0,0xffff2
   10078:	f8c50513          	addi	a0,a0,-116 # 2000 <__DATA_BEGIN__>
   1007c:	00000393          	li	t2,0
   10080:	06100e93          	li	t4,97
   10084:	07b00f13          	li	t5,123

00010088 <loop_start>:
   10088:	00054383          	lbu	t2,0(a0)
   1008c:	02038263          	beqz	t2,100b0 <end_program>
   10090:	01e3fc63          	bgeu	t2,t5,100a8 <skip>
   10094:	01d3ea63          	bltu	t2,t4,100a8 <skip>
   10098:	fe038393          	addi	t2,t2,-32
   1009c:	00750023          	sb	t2,0(a0)
   100a0:	00150513          	addi	a0,a0,1
   100a4:	fe5ff06f          	j	10088 <loop_start>

000100a8 <skip>:
   100a8:	00150513          	addi	a0,a0,1
   100ac:	fddff06f          	j	10088 <loop_start>

000100b0 <end_program>:
   100b0:	0000006f          	j	100b0 <end_program>
