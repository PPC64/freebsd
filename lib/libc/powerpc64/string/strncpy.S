/*-
 * Copyright (c) 2018 Instituto de Pesquisas Eldorado
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <machine/asm.h>
__FBSDID("$FreeBSD$");

#if 0
        RCSID("$NetBSD: strncpy.S,v 1.0 2018/04/10 15:18:16 lbianc Exp $")
#endif
	.text

ENTRY(strncpy)
	std     3, -8(1)
	std     4, -16(1)
	std     5, -24(1)

	cmpdi   5,0
	beq     .exit              // Just leave if n==0
	andi.   7,5,7              // Save the rest when divided by 8
	srdi    5,5,3
	cmpdi   cr7,5,0            // Check if n < 8
	beq     cr7,.cpy_byte      // If true, copy byte per byte (using r7 as n)
	mtctr   5
// Optimize copy reading and saving double word.
// Also check if zero is in the current double word.
.copy_dw:
 //1
	lbz     0,0(3)
	cmpdi   cr7,0,0
	beq     cr7,.label8
 //2
	lbz     6,1(3)
	cmpdi   cr7,6,0
	sldi    0,0,8
	or      0,0,6
	beq     cr7,.label16
 //3
	lbz     6,2(3)
	cmpdi   cr7,6,0
	sldi    0,0,8
	or      0,0,6
	beq     cr7,.label24
 //4
	lbz     6,3(3)
	cmpdi   cr7,6,0
	sldi    0,0,8
	or      0,0,6
	beq     cr7,.label32
 //5
	lbz     6,4(3)
	cmpdi   cr7,6,0
	sldi    0,0,8
	or      0,0,6
	beq     cr7,.label40
 //6
	lbz     6,5(3)
	cmpdi   cr7,6,0
	sldi    0,0,8
	or      0,0,6
	beq     cr7,.label48
 //7
	lbz     6,6(3)
	cmpdi   cr7,6,0
	sldi    0,0,8
	or      0,0, 6
	beq     cr7,.label56
 //8
	lbz     6,7(3)
	addi    3,3,8
	cmpdi   cr7,6,0
	sldi    0,0,8
	or      0,0,6
	std     0,0(4)
	addi    4,4,8
	beq     cr7,.store_z_dw
	addi    5,5,-1
	bdnz    .copy_dw
	b       .cpy_byte           // If ended double word, check missing bytes
 // Zero found on 7th byte, store 56 bytes from double word.
.label56:
	li      6,0
	srdi    6,0,24
	stw     6,0(4)
	srdi    6,0,8
	sth     6,4(4)
	stb     0,6(4)
	li      0,0
	stb     0,7(4)
	addi    4,4,8
	b       .store_z_dw
 // Zero found on 6th byte, store 48 bytes from double word.
.label48:
	srdi    6,0,16
	stw     6,0(4)
	sth     0,4(4)
	li      0,0
	sth     0,6(4)
	addi    4,4,8
	b       .store_z_dw
 // Zero found on 5th byte, store 40 bytes from double word.
.label40:
	srdi    6,0,8
	stw     6,0(4)
	stb     0,4(4)
	li      0,0
	stb     0,5(4)
	sth     0,6(4)
	addi    4,4,8
	b       .store_z_dw
 // Zero found on 4th byte, store 32 bytes from double word.
.label32:
	stw     0,0(4)
	li      0,0
	stw     0,4(4)
	addi    4,4,8
	b       .store_z_dw
 // Zero found on 3rd byte, store 24 bytes from double word.
.label24:
	srdi    6,0,8
	sth     6,0(4)
	stb     0,2(4)
	li      0,0
	stb     0,3(4)
	stw     0,4(4)
	addi    4,4,8
	b       .store_z_dw
 // Zero found on 2nd byte, store 16 bytes from double word.
.label16:
	sth     0,0(4)
	li      0,0
	sth     0,2(4)
	stw     0,4(4)
	addi    4,4,8
	b       .store_z_dw
 // Zero found on 1st byte, store 8 bytes from double word.
.label8:
	stb     0,0(4)
	li      0,0
	stb     0,1(4)
	sth     0,2(4)
	stw     0,4(4)
	addi    4,4,8
.store_z_dw:                    // Store zeros as double word.
	bdz     .cpy_byte_z
	li      0,0
	std     0,0(4)
	addi    4,4,8
	b       .store_z_dw
 // Copy byte per byte
 // Consider r7 with size (n)
.cpy_byte:
	cmpdi   cr7,7,0
	beq     cr7,.exit
	mr      5,7
	mtctr   7
.cpy_n_byte:
	lbz     0,0(3)
	cmpdi   cr7,0,0
	beq     cr7,.cpy_byte_z_1
	stb     0,0(4)
	addi    3,3,1
	addi    4,4,1
	bdnz    .cpy_n_byte
	b       .exit
.cpy_byte_z:
	cmpdi   cr7,7,0
	beq     cr7,.exit
	mr      5,7
	mtctr   7
.cpy_byte_z_1:
	li      0,0
.cpy_n_byte_z:
	stb     0,0(4)
	addi    3,3,1
	bdnz    .cpy_n_byte_z
.exit:
	ld      3,-8(1)
	ld      4,-16(1)
	ld      5,-24(1)
	blr

END(strncpy)

      .section .note.GNU-stack,"",%progbits

