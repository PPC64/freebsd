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
	RCSID("$NetBSD: bcopy.S,v 1.0 2018/03/22 13:37:42 lffpires Exp $")
#endif

#define BCOPY_ALIGNMENT_BYTES 16
#define BCOPY_ALIGNMENT_MASK (BCOPY_ALIGNMENT_BYTES - 1)

#define BCOPY_BLOCK_SIZE_BITS 6
#define BCOPY_BLOCK_SIZE (1 << BCOPY_BLOCK_SIZE_BITS)
#define BCOPY_BLOCK_SIZE_MASK (BCOPY_BLOCK_SIZE - 1)

#define BCOPY_BLOCK_COPY_THRESHOLD 512

#ifndef FN_NAME
#ifdef MEMMOVE
#define FN_NAME __memmove
#else
#define FN_NAME __bcopy
#endif


ENTRY(FN_NAME)
	cmpld	%r3, %r4					/* src == dst? if so, nothing to do */
	beqlr-
	cmpdi	%r5, 0						/* len == 0? nothing to do : continue */
	beqlr-

	std	%r3, -8(%r1)					/* save dst */
	cmpldi	%r5, BCOPY_BLOCK_COPY_THRESHOLD			/* len >= BCOPY_BLOCK_COPY_THRESHOLD? */
	bge	.Lmulti_phase					/* if so, go to multi-phase */

	/* set up single-phase copy parameters */
.Lsingle_phase_setup:
	cmpd	%r4, %r3					/* forward or backward copy? */
	blt	.Lbackward_single_copy

	/* forward copy */
	li	%r0, 1						/* increment for single phase 1-byte */
	li	%r8, 16						/* increment for single phase 16-byte */
	li	%r9, 0						/* pre-adjustment for single phase 16-byte */

	b	.Lsingle_phase

.Lbackward_single_copy:
	/* backward copy */
	li	%r0, -1						/* increment for single phase 1-byte */
	li	%r8, -16					/* increment for single phase 16-byte */
	li	%r9, -15					/* pre/post adjustment for single phase 16-byte */
	add	%r6, %r5, %r0					/* %r6 = len - 1 */
	add	%r3, %r3, %r6					/* advance to the last position in dst */
	add	%r4, %r4, %r6					/* advance to the last position in src */

.Lsingle_phase:
	srdi.	%r6, %r5, 4					/* number of 16-bytes */
	beq	.Lsingle_1

	add	%r3, %r3, %r9					/* pre-adjustment */
	add	%r4, %r4, %r9					/* pre-adjustment */

	mtctr	%r6
	.align 5
.Lsingle_16_loop:
	ld	%r6, 0(%r4)
	ld	%r7, 8(%r4)
	add	%r4, %r4, %r8
	std	%r6, 0(%r3)
	std	%r7, 8(%r3)
	add	%r3, %r3, %r8
	bdnz	.Lsingle_16_loop

	sub	%r3, %r3, %r9					/* post-adjustment */
	sub	%r4, %r4, %r9					/* post-adjustment */

.Lsingle_1:
	andi.	%r6, %r5, 0x0f					/* number of 1-bytes */
	beq	.Ldone						/* 1-bytes == 0? if so, nothing to do */

	mtctr	%r6
	.align 5
.Lsingle_1_loop:
	lbz	%r6, 0(%r4)
	add	%r4, %r4, %r0					/* increment */
	stb	%r6, 0(%r3)
	add	%r3, %r3, %r0					/* increment */

	bdnz	.Lsingle_1_loop

.Ldone:
	/* done copying */

	ld	%r3, -8(%r1)					/* restore dst */
	blr


.Lmulti_phase:
	/* set up multi-phase copy parameters */
	andi.	%r6, %r4, BCOPY_ALIGNMENT_MASK

	subfic	%r7, %r6, BCOPY_ALIGNMENT_BYTES
	andi.	%r7, %r7, BCOPY_ALIGNMENT_MASK			/* %r7 = bytes before the aligned section of the buffer */
	sub	%r8, %r5, %r7					/* %r8 = number of bytes in and after the aligned section of the buffer */
	andi.	%r9, %r8, BCOPY_BLOCK_SIZE_MASK			/* %r9 = number of bytes after the aligned section of the buffer */
	srdi	%r10, %r8, BCOPY_BLOCK_SIZE_BITS		/* %r10 = number of BLOCKS in the aligned section of the buffer */

	cmpd	%r4, %r3					/* forward or backward copy? */
	blt	.Lbackward_multi_copy

	/* set up forward copy parameters */
	std	%r7, -32(%r1)					/* number of bytes to copy in phase 1 */
	std	%r9, -48(%r1)					/* number of bytes to copy in phase 3 */
	std	%r10, -40(%r1)					/* number of BLOCKS to copy in phase 2 */

	li	%r0, 1						/* increment for phases 1 and 3 */
	li	%r5, BCOPY_BLOCK_SIZE				/* increment for phase 2 */

	li	%r7, 0						/* offset for op 1 of phase 2 */
	li	%r8, 16						/* offset for op 2 of phase 2 */
	li	%r9, 32						/* offset for op 3 of phase 2 */
	li	%r10, 48					/* offset for op 4 of phase 2 */

	std	%r8, -16(%r1)					/* increment for single phase 16-byte (16) */
	std	%r7, -24(%r1)					/* pre/post adjustment for single phase 16-byte (0) */

	b	.Lphase1

.Lbackward_multi_copy:
	/* set up backward copy parameters */
	std	%r7, -48(%r1)					/* number of bytes to copy in phase 3 */
	std	%r9, -32(%r1)					/* number of bytes to copy in phase 1 */
	std	%r10, -40(%r1)					/* number of BLOCKS to copy in phase 2 */

	li	%r0, -1						/* increment for phases 1 and 3 */
	add	%r6, %r5, %r0					/* %r6 = len - 1 */
	add	%r3, %r3, %r6					/* advance to the last position in dst */
	add	%r4, %r4, %r6					/* advance to the last position in src */
	li	%r5, -BCOPY_BLOCK_SIZE				/* increment for phase 2 */

	li	%r7, -15					/* offset for op 1 of phase 2 */
	li	%r8, -31					/* offset for op 2 of phase 2 */
	li	%r9, -47					/* offset for op 3 of phase 2 */
	li	%r10, -63					/* offset for op 4 of phase 2 */

	add	%r6, %r7, %r0					/* %r6 = -16 */
	std	%r6, -16(%r1)					/* increment for single phase 16-byte (-16) */
	std	%r7, -24(%r1)					/* pre/post adjustment for single phase 16-byte (-15) */

.Lphase1:
	ld	%r6, -32(%r1)					/* number of bytes to copy in phase 1 */
	cmpldi	%r6, 0						/* %r6 == 0? (if so, nothing to copy in phase 1) */
	beq+	.Lphase2

	mtctr	%r6
	.align 5
.Lphase1_loop:
	lbz	%r6, 0(%r4)
	add	%r4, %r4, %r0					/* phase 1 increment */
	stb	%r6, 0(%r3)
	add	%r3, %r3, %r0					/* phase 1 increment */

	bdnz	.Lphase1_loop

.Lphase2:
	ld	%r6, -40(%r1)					/* number of BLOCKS to copy in phase 2 */
	cmpldi	%r6, 0						/* %r6 == 0? (if so, nothing to copy in phase 2) */
	beq	.Lphase3

#ifdef FN_PHASE2
FN_PHASE2
#else
	/* save registers */
	std	%r14, -56(%r1)
	std	%r15, -64(%r1)
	std	%r16, -72(%r1)
	std	%r17, -80(%r1)
	std	%r18, -88(%r1)
	std	%r19, -96(%r1)
	std	%r20, -104(%r1)
	std	%r21, -112(%r1)

	addi	%r18, %r7, 8
	addi	%r19, %r8, 8
	addi	%r20, %r9, 8
	addi	%r21, %r10, 8

	mtctr	%r6
	.align 5
.Lphase2_no_vsx_loop:
	ldx	%r14, %r7, %r4
	ldx	%r15, %r18, %r4
	ldx	%r16, %r8, %r4
	ldx	%r17, %r19, %r4
	stdx	%r14, %r7, %r3
	stdx	%r15, %r18, %r3
	stdx	%r16, %r8, %r3
	stdx	%r17, %r19, %r3

	ldx	%r14, %r9, %r4
	ldx	%r15, %r20, %r4
	ldx	%r16, %r10, %r4
	ldx	%r17, %r21, %r4
	stdx	%r14, %r9, %r3
	stdx	%r15, %r20, %r3
	stdx	%r16, %r10, %r3
	stdx	%r17, %r21, %r3

	add	%r4, %r4, %r5					/* phase 2 increment */
	add	%r3, %r3, %r5					/* phase 2 increment */

	bdnz	.Lphase2_no_vsx_loop

	/* restore registers */
	ld	%r14, -56(%r1)
	ld	%r15, -64(%r1)
	ld	%r16, -72(%r1)
	ld	%r17, -80(%r1)
	ld	%r18, -88(%r1)
	ld	%r19, -96(%r1)
	ld	%r20, -104(%r1)
	ld	%r21, -112(%r1)
#endif

.Lphase3:
	/* load registers for transitioning into the single-phase logic */
	ld	%r5, -48(%r1)					/* number of bytes to copy in phase 3 */
	ld	%r8, -16(%r1)					/* increment for single phase 16-byte */
	ld	%r9, -24(%r1)					/* pre/post adjustment for single phase 16-byte */
	b	.Lsingle_phase

END(FN_NAME)

	.section .note.GNU-stack,"",%progbits

