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
 *
 */

#include <machine/asm.h>
__FBSDID("$FreeBSD$");

#if 0
	RCSID("$NetBSD: bcopy.S,v 1.0 2019/03/06 13:37:42 lbianc Exp $")
#endif

#ifdef USE_VSX
#define MEMCPY_BUILD_FUNCTION_NAME(name) __ ## name ## _vsx
#define MEMCPY_ALIGNMENT_SIZE_BITS 6
#define MEMCPY_ALIGNMENT_BYTES (1 << MEMCPY_ALIGNMENT_SIZE_BITS)
#define MEMCPY_ALIGNMENT_MASK (MEMCPY_ALIGNMENT_BYTES - 1)
#else
#define MEMCPY_BUILD_FUNCTION_NAME(name) __ ## name
#define MEMCPY_ALIGNMENT_SIZE_BITS 4
#define MEMCPY_ALIGNMENT_BYTES (1 << MEMCPY_ALIGNMENT_SIZE_BITS)
#define MEMCPY_ALIGNMENT_MASK (MEMCPY_ALIGNMENT_BYTES - 1)
#endif

#define MEMCPY_FUNCTION_NAME MEMCPY_BUILD_FUNCTION_NAME(memcpy)

ENTRY(MEMCPY_FUNCTION_NAME)
	cmpdi	%r5, 0		/* len == 0? nothing to do : continue */
	beqlr-

	mr	%r8, %r3	/* save dst */

.Lalignment_loop:
	lbz	%r6, 0(%r4)
	stb	%r6, 0(%r3)
	addi	%r3, %r3, 1
	addi	%r4, %r4, 1
	addi	%r5, %r5, -1
	cmpdi	%r5, 0
	beq	.Lexit
	andi.	%r0,%r4,MEMCPY_ALIGNMENT_MASK
	bne	.Lalignment_loop

	/* Save the remaining of MEMCPY_ALIGNMENT_BYTES */
	andi.	%r7, %r5, MEMCPY_ALIGNMENT_MASK

	/* Check if there is blocks of MEMCPY_ALIGNMENT_BYTES size to be copied */
	xor.	%r5, %r5, %r7
	beq	.Lcopy_remaining_fix_index_byte

#ifdef USE_VSX
	/* Load CTR with quantity of blocks */
	srdi	%r5, %r5, MEMCPY_ALIGNMENT_SIZE_BITS
	mtctr	%r5
	/* Prepare indexes to load and store data */
	xor	%r6, %r6, %r6
	li	%r9,  16
	li	%r10, 32
	li	%r11, 48
.Lcopy_vsx_loop:
	lxvd2x	%vs6, %r6,  %r4
	lxvd2x	%vs7, %r9,  %r4
	lxvd2x	%vs8, %r10, %r4
	lxvd2x	%vs9, %r11, %r4
	stxvd2x	%vs6, %r6,  %r3
	stxvd2x	%vs7, %r9,  %r3
	stxvd2x	%vs8, %r10, %r3
	stxvd2x	%vs9, %r11, %r3

	addi	%r3, %r3, MEMCPY_ALIGNMENT_BYTES
	addi	%r4, %r4, MEMCPY_ALIGNMENT_BYTES
	bdnz	.Lcopy_vsx_loop

	/* Check if there is remaining bytes */
	cmpd    %r7, 0
	beq     .Lexit
#else
	/* Setup to copy word with ldu and stdu */
	ld	%r6, 0(%r4)
	ld	%r9, 8(%r4)
	std	%r6, 0(%r3)
	std	%r9, 8(%r3)
	addi	%r5, %r5, -MEMCPY_ALIGNMENT_BYTES
	cmpd	%r5, 0
	beq	.Lcopy_remaining_fix_index_word

	srdi    %r5, %r5, MEMCPY_ALIGNMENT_SIZE_BITS
	mtctr	%r5
.Lcopy_word:
	ldu	%r6,16(%r4)
	ld	%r9, 8(%r4)
	stdu	%r6,16(%r3)
	std	%r9, 8(%r3)
	bdnz	.Lcopy_word

.Lcopy_remaining_fix_index_word:
	/* Check if there is remaining bytes */
	cmpd	%r7, 0
	beq	.Lexit
	addi	%r3, %r3, MEMCPY_ALIGNMENT_MASK
	addi	%r4, %r4, MEMCPY_ALIGNMENT_MASK
	b	.Lcopy_remaining
#endif

.Lcopy_remaining_fix_index_byte:
	addi	%r4, %r4, -1
	addi	%r3, %r3, -1

.Lcopy_remaining:
	/* Load remaining bytes in CTR */
	mtctr	%r7

.Lcopy_remaining_loop:
	/* Copy remaining bytes */
	lbzu	%r6, 1(%r4)
	stbu	%r6, 1(%r3)
	bdnz	.Lcopy_remaining_loop

.Lexit:
	/* Restore dst */
	mr	%r3, %r8
	blr

END(MEMCPY_FUNCTION_NAME)

	.section .note.GNU-stack,"",%progbits

