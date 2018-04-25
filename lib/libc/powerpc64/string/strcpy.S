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
        RCSID("$NetBSD: strcpy.S,v 1.0 2018/04/04 13:02:11 lbianc Exp $")
#endif

	.text

ENTRY(strcpy)
// r3 = destination (%1)
// r4 = source (%0)
	std     3,-8(1)
.copy_dw:
// check 1st byte
	lbz     0,0(4)
	cmpdi   cr7,0,0
	beq     cr7, .found_on_byte_8
// check 2nd byte
	sldi    6,0,8
	lbz     0,1(4)
	cmpdi   cr7,0,0
	or      0,6,0
	beq     cr7, .found_on_byte_16
// check 3rd byte
	sldi    6,0,8
	lbz     0,2(4)
	cmpdi   cr7,0,0
	or      0,6,0
	beq     cr7, .found_on_byte_24
// check 4th byte
	sldi    6,0,8
	lbz     0,3(4)
	cmpdi   cr7,0,0
	or      0,6,0
	beq     cr7, .found_on_byte_32
// check 5th byte
	sldi    6,0,8
	lbz     0,4(4)
	cmpdi   cr7,0,0
	or      0,6,0
	beq     cr7, .found_on_byte_40
// check 6th byte
	sldi    6,0,8
	lbz     0,5(4)
	cmpdi   cr7,0,0
	or      0,6,0
	beq     cr7, .found_on_byte_48
// check 7th byte
	sldi    6,0,8
	lbz     0,6(4)
	cmpdi   cr7,0,0
	or      0,6,0
	beq     cr7, .found_on_byte_56
// check 8th byte
	sldi    6,0,8
	lbz     0,7(4)
	cmpdi   cr7,0,0
	or      0,6,0
	std     0,0(3)
	addi    3,3,8
	addi    4,4,8
	beq     cr7, .exit
	b       .copy_dw
// Zero found on 7th byte, store 56 bytes from double word.
.found_on_byte_56:
	srdi    6,0,24
	stw     6,0(3)
	srdi    6,0,8
	sth     6,4(3)
	stb     0,6(3)
	li      0,0
	stb     0,7(3)
	addi    3,3,8
	b       .exit
// Zero found on 6th byte, store 48 bytes from double word.
.found_on_byte_48:
	srdi    6,0,16
	stw     6,0(3)
	sth     0,4(3)
	li      0,0
	sth     0,6(3)
	addi    3,3,8
	b       .exit
// Zero found on 5th byte, store 40 bytes from double word.
.found_on_byte_40:
	srdi    6,0,8
	stw     6,0(3)
	stb     0,4(3)
	li      0,0
	stb     0,5(3)
	sth     0,6(3)
	addi    3,3,8
	b       .exit
// Zero found on 4th byte, store 32 bytes from double word.
.found_on_byte_32:
	stw     0,0(3)
	li      0,0
	stw     0,4(3)
	addi    3,3,8
	b       .exit
// Zero found on 3rd byte, store 24 bytes from double word.
.found_on_byte_24:
	srdi    6,0,8
	sth     6,0(3)
	stb     0,2(3)
	li      0,0
	stb     0,3(3)
	stw     0,4(3)
	addi    3,3,8
	b       .exit
// Zero found on 2nd byte, store 16 bytes from double word.
.found_on_byte_16:
	sth     0,0(3)
	li      0,0
	sth     0,2(3)
	stw     0,4(3)
	addi    3,3,8
	b       .exit
// Zero found on 1st byte, store 8 bytes from double word.
.found_on_byte_8:
	stb     0,0(3)
	li      0,0
	stb     0,1(3)
	sth     0,2(3)
	stw     0,4(3)
	addi    3,3,8
.exit:
	ld      3,-8(1)
	blr

END(strcpy)

	.section .note.GNU-stack,"",%progbits
