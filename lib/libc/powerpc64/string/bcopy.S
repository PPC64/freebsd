/*-
 * Copyright (c) 2018 Instituto de Pesquisas Eldorado
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *	be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.	IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
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
	RCSID("$NetBSD: bcopy.S,v 1.0 2018/03/22 13:37:42 lffpires Exp $")
#endif

#ifdef MEMCOPY
ENTRY(memcpy)
#else
#ifdef MEMMOVE
ENTRY(memmove)
#else
ENTRY(bcopy)
#endif
#endif
	// src == dst? if so, nothing to do
	cmpld	%r3, %r4
	beqlr-	%cr0

#if defined(MEMCOPY) || defined(MEMMOVE)
	// save dst
	std	%r3, -8(%r1)
#else
	// bcopy has src and dest in a different order than memcpy/memmove
	mr	%r7, %r3
	mr	%r3, %r4
	mr	%r4, %r7
#endif

	// src > dst? if so, copy data in backward.
	bt	gt, .Lmemcpy_256_128_64_32_8_rev

	// if data size >= 32, copy by 256 block.
	cmpdi %r5,32
	bt	lt, .Lmemcpy_128_64_32_8

.Lmemcpy_256:
	// copy by 256 block.
	srdi	%r10, %r5, 5
	clrldi	%r5, %r5, 59
	mtctr	%r10

.Lmemcpy_256_loop:
	// while counter > 0, copy 256 block.
	ld	%r7, 0(%r4)
	std	%r7, 0(%r3)
	ld	%r6, 8(%r4)
	std	%r6, 8(%r3)
	ld	%r10, 16(%r4)
	std	%r10, 16(%r3)
	ld	%r11, 24(%r4)
	std	%r11, 24(%r3)

	// increment data addresses.
	addi	%r4, %r4,32
	addi	%r3, %r3,32

	bdnz	.Lmemcpy_256_loop

.Lmemcpy_128_64_32_8:
	// if data size >= 16, copy by quadword.
	cmpdi	%r5, 16
	bt	lt, .Lmemcpy_64_32_8

.Lmemcpy_128:
	// copy by quadword.
	srdi	%r10, %r5, 4
	clrldi	%r5, %r5, 60
	mtctr	%r10

.Lmemcpy_128_loop:
	// while counter > 0, copy quadword.
	ld	%r6, 0(%r4)
	std	%r6, 0(%r3)
	ld	%r7, 8(%r4)
	std	%r7, 8(%r3)

	// increment data addresses.
	addi	%r4, %r4, 16
	addi	%r3, %r3, 16

	bdnz	.Lmemcpy_128_loop

.Lmemcpy_64_32_8:
	// if data size >= 8, copy by double word.
	cmpdi	%r5, 8
	bt	lt, .Lmemcpy_32_8

.Lmemcpy_64:
	// copy by double word.
	srdi	%r10, %r5, 3
	clrldi	%r5, %r5, 61
	mtctr	%r10

.Lmemcpy_64_loop:
	// while counter > 0, copy double word.
	ld	%r6, 0(%r4)
	std	%r6, 0(%r3)

	// increment data addresses.
	addi	%r4, %r4, 8
	addi	%r3, %r3, 8

	bdnz	.Lmemcpy_64_loop

.Lmemcpy_32_8:
	// if data size >= 4, copy by word.
	cmpdi	%r5, 4
	bt	lt, .Lmemcpy_8

.Lmemcpy_32:
	// copy by word.
	srdi	%r10, %r5, 2
	clrldi	%r5, %r5, 62
	mtctr	%r10

.Lmemcpy_32_loop:
	// while counter > 0, copy word.
	lwz	%r6, 0(%r4)
	stw	%r6, 0(%r3)

	// increment data addresses.
	addi	%r4, %r4, 4
	addi	%r3, %r3, 4

	bdnz	.Lmemcpy_32_loop

.Lmemcpy_8:
	// if data size <= 0, finished copying.
	cmpdi	%r5, 1
	bt	lt, .Lmemcpy_end
	mtctr	%r5

.Lmemcpy_8_loop:
	// while counter > 0, copy byte.
	lbz	%r6, 0(%r4)
	stb	%r6, 0(%r3)

	// increment data addresses.
	addi	%r4, %r4, 1
	addi	%r3, %r3, 1

	bdnz	.Lmemcpy_8_loop
	b	.Lmemcpy_end

.Lmemcpy_256_128_64_32_8_rev:
	// src address > dst.
	// set addresses to end of input variables for backward copy.
	add	%r4, %r4, %r5
	add	%r3, %r3, %r5

	// if data size >= 32, copy by 256 block.
	cmpdi	%r5, 32
	bt	lt, .Lmemcpy_128_64_32_8_rev

.Lmemcpy_256_rev:
	// copy by 256 block.
	srdi	%r10, %r5, 5
	clrldi	%r5, %r5, 59
	mtctr	%r10

.Lmemcpy_256_rev_loop:
	// while counter > 0, copy 256 block.
	ld	%r6, -8(%r4)
	std	%r6, -8(%r3)
	ld	%r7, -16(%r4)
	std	%r7, -16(%r3)
	ld	%r10, -24(%r4)
	std	%r10, -24(%r3)
	ld	%r11, -32(%r4)
	std	%r11, -32(%r3)

	// decrement data addresses.
	subi	%r4, %r4, 32
	subi	%r3, %r3, 32

	bdnz	.Lmemcpy_256_rev_loop

.Lmemcpy_128_64_32_8_rev:
	// if data size >= 16, copy by quadword.
	cmpdi	%r5, 16
	bt	lt, .Lmemcpy_64_32_8_rev

.Lmemcpy_128_rev:
	// copy by quadword.
	srdi	%r10, %r5, 4
	clrldi	%r5, %r5, 60
	mtctr	%r10

.Lmemcpy_128_rev_loop:
	// while counter > 0, copy quadword.
	ld	%r6, -8(%r4)
	std	%r6, -8(%r3)
	ld	%r7, -16(%r4)
	std	%r7, -16(%r3)

	// decrement data addresses.
	subi	%r4, %r4, 16
	subi	%r3, %r3, 16

	bdnz	.Lmemcpy_128_rev_loop

.Lmemcpy_64_32_8_rev:
	// if data size >= 8, copy by double word.
	cmpdi	%r5, 8
	bt	lt, .Lmemcpy_32_8_rev

.Lmemcpy_64_rev:
	// copy by double word.
	srdi	%r10, %r5, 3
	clrldi	%r5, %r5, 61
	mtctr	%r10

.Lmemcpy_64_rev_loop:
	// while counter > 0, copy word.
	ld	%r6, -8(%r4)
	std	%r6, -8(%r3)

	// decrement data addresses.
	subi	%r4, %r4, 8
	subi	%r3, %r3, 8

	bdnz	.Lmemcpy_64_rev_loop

.Lmemcpy_32_8_rev:
	// if data size >= 4, copy by word.
	cmpdi	%r5, 4
	bt	lt, .Lmemcpy_8_rev

.Lmemcpy_32_rev:
	// copy word.
	srdi	%r10, %r5, 2
	clrldi	%r5, %r5, 62
	mtctr	%r10

.Lmemcpy_32_rev_loop:
	// while counter > 0, copy word.
	lwz	%r6, -4(%r4)
	stw	%r6, -4(%r3)

	// decrement data addresses.
	subi	%r4, %r4, 4
	subi	%r3, %r3, 4

	bdnz	.Lmemcpy_32_rev_loop

.Lmemcpy_8_rev:
	// if data size <= 0, finished copying.
	cmpdi	%r5, 1
	bt	lt, .Lmemcpy_end
	mtctr	%r5

.Lmemcpy_8_rev_loop:
	// while counter > 0, copy byte.
	// copy byte.
	lbz	%r6, -1(%r4)
	stb	%r6, -1(%r3)

	// decrement data addresses.
	subi	%r4, %r4, 1
	subi	%r3, %r3, 1

	bdnz	.Lmemcpy_8_rev_loop

.Lmemcpy_end:
	// finished copying. 

#if defined(MEMCOPY) || defined(MEMMOVE)
	// restore dst
	ld	%r3, -8(%r1)
#endif
	blr

#ifdef MEMCOPY
END(memcpy)
#else
#ifdef MEMMOVE
END(memmove)
#else
END(bcopy)
#endif
#endif

	.section .note.GNU-stack,"",%progbits

