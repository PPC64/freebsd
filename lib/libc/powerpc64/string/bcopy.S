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
        RCSID("$NetBSD: bcopy.S,v 1.0 2018/03/22 13:37:42 lffpires Exp $")
#endif

#define BCOPY_MULTI_ALIGNMENT_BYTES 4
#define BCOPY_MULTI_ALIGNMENT_MASK (BCOPY_MULTI_ALIGNMENT_BYTES - 1)
#define BCOPY_MULTI_BLOCK_SIZE_BITS 6
#define BCOPY_MULTI_BLOCK_SIZE (1 << BCOPY_MULTI_BLOCK_SIZE_BITS)
#define BCOPY_MULTI_BLOCK_SIZE_MASK (BCOPY_MULTI_BLOCK_SIZE - 1)

        .text

#ifdef MEMCOPY
ENTRY(memcpy)
#else
#ifdef MEMMOVE
ENTRY(memmove)
#else
ENTRY(bcopy)
#endif
#endif
        or.     %r5, %r5, %r5                                   // len == 0? if so, nothing to do
        beqlr-  %cr0

        cmpld   %r3, %r4                                        // src == dst? if so, nothing to do
        beqlr-  %cr0

        // check if dst and src are aligned to each other
        andi.   %r6, %r3, BCOPY_MULTI_ALIGNMENT_MASK
        andi.   %r7, %r4, BCOPY_MULTI_ALIGNMENT_MASK
        cmpd    %cr0, %r6, %r7
        bne     %cr0, .Lmisaligned_128_64_32_8

#if !defined(MEMCOPY) && !defined(MEMMOVE)
        std     %r3, -136(%r1)                                  // save dst
#endif

        // set up copy parameters
        subfic  %r7, %r6, BCOPY_MULTI_ALIGNMENT_BYTES
        andi.   %r7, %r7, BCOPY_MULTI_ALIGNMENT_MASK            // %r7 = bytes before the aligned section of the buffer
        sub     %r8, %r5, %r7                                   // %r8 = number of bytes in and after the aligned section of the buffer
        andi.   %r9, %r8, BCOPY_MULTI_BLOCK_SIZE_MASK           // %r9 = number of bytes after the aligned section of the buffer
        srdi    %r10, %r8, BCOPY_MULTI_BLOCK_SIZE_BITS          // %r10 = number of BLOCKS in the aligned section of the buffer

        cmpd    %cr0, %r4, %r3                                  // forward or backward copy?
        blt     .Laligned_backward_copy

        // set up forward copy parameters
        std     %r7, -144(%r1)                                  // number of bytes to copy in phase 1
        std     %r9, -160(%r1)                                  // number of bytes to copy in phase 3
        std     %r10, -152(%r1)                                 // number of BLOCKS to copy in phase 2

        li      %r8, 1                                          // incrementx for phases 1 and 3
        li      %r9, 0                                          // pre increment for phase 2
        li      %r10, BCOPY_MULTI_BLOCK_SIZE                    // post increment for phase 2

        b       .Laligned_phase1

.Laligned_backward_copy:
        // set up backward copy parameters
        std     %r7, -160(%r1)                                  // number of bytes to copy in phase 3
        std     %r9, -144(%r1)                                  // number of bytes to copy in phase 1
        std     %r10, -152(%r1)                                 // number of BLOCKS to copy in phase 2

        li      %r8, -1                                         // incrementx for phases 1 and 3
        li      %r9, (1 - BCOPY_MULTI_BLOCK_SIZE)               // pre increment for phase 2
        li      %r10, -1                                        // post increment for phase 2

        add     %r6, %r5, %r8                                   // %r6 = len - 1
        add     %r3, %r3, %r6                                   // advance to the last position in dst
        add     %r4, %r4, %r6                                   // advance to the last position in src

.Laligned_phase1:
        ld      %r7, -144(%r1)                                  // number of bytes to copy in phase 1
        cmpldi  %r7, 0                                          // %r7 == 0? (if so, nothing to copy in phase 1)
        beq+    %cr0, .Laligned_phase2

        mtctr   %r7
.Laligned_phase1_loop:
        lbz     %r6, 0(%r4)
        add     %r4, %r4, %r8                                   // phase 1 increment
        stb     %r6, 0(%r3)
        add     %r3, %r3, %r8                                   // phase 1 increment

        bdnz    .Laligned_phase1_loop

.Laligned_phase2:
        ld      %r7, -152(%r1)                                  // number of BLOCKS to copy in phase 2
        cmpldi  %r7, 0                                          // %r7 == 0? (if so, nothing to copy in phase 2)
        beq     %cr0, .Laligned_phase3

        // save %r16-%r31 - is there a better way to do this?
        std     %r16, -8(%r1)
        std     %r17, -16(%r1)
        std     %r18, -24(%r1)
        std     %r19, -32(%r1)
        std     %r20, -40(%r1)
        std     %r21, -48(%r1)
        std     %r22, -56(%r1)
        std     %r23, -64(%r1)
        std     %r24, -72(%r1)
        std     %r25, -80(%r1)
        std     %r26, -88(%r1)
        std     %r27, -96(%r1)
        std     %r28, -104(%r1)
        std     %r29, -112(%r1)
        std     %r30, -120(%r1)
        std     %r31, -128(%r1)

        mtctr   %r7
.Laligned_phase2_loop:
        add     %r4, %r4, %r9                                   // phase 2 pre increment
        lmw     %r16, 0(%r4)
        add     %r4, %r4, %r10                                  // phase 2 post increment

        add     %r3, %r3, %r9                                   // phase 2 pre increment
        stmw    %r16, 0(%r3)
        add     %r3, %r3, %r10                                  // phase 2 post increment

        bdnz    .Laligned_phase2_loop

        // restore %r16-%r31 - is there a better way to do this?
        ld      %r16, -8(%r1)
        ld      %r17, -16(%r1)
        ld      %r18, -24(%r1)
        ld      %r19, -32(%r1)
        ld      %r20, -40(%r1)
        ld      %r21, -48(%r1)
        ld      %r22, -56(%r1)
        ld      %r23, -64(%r1)
        ld      %r24, -72(%r1)
        ld      %r25, -80(%r1)
        ld      %r26, -88(%r1)
        ld      %r27, -96(%r1)
        ld      %r28, -104(%r1)
        ld      %r29, -112(%r1)
        ld      %r30, -120(%r1)
        ld      %r31, -128(%r1)

.Laligned_phase3:
        ld      %r7, -160(%r1)                                  // number of bytes to copy in phase 3
        cmpldi  %r7, 0                                          // %r7 == 0? (if so, nothing to copy in phase 3)
        beq     %cr0, .Laligned_done

        mtctr   %r7
.Laligned_phase3_loop:
        lbz     %r6, 0(%r4)
        add     %r4, %r4, %r8                                   // phase 3 increment
        stb     %r6, 0(%r3)
        add     %r3, %r3, %r8                                   // phase 3 increment

        bdnz    .Laligned_phase3_loop

.Laligned_done:
        // done copying

#if !defined(MEMCOPY) && !defined(MEMMOVE)
        ld      %r3, -136(%r1)                                  // restore dst
#endif
        blr

.Lmisaligned_128_64_32_8:
        // if src address >= dst address copy forward,
        // else copy backward.
        cmpd  %r3,%r4
        bt  gt,.Lmisaligned_128_64_32_8_rev

        mr  %r8,%r4 // Src pointer in R8.
        mr  %r9,%r3 // Dst pointer in R9.

        // if variable size >= 16, copy by quadword.
        cmpdi %r5,16
        bt  lt, .Lmisaligned_64_32_8

.Lmisaligned_128:
        // copy by quadword.
        srdi %r10,%r5,4
        clrldi %r5,%r5,60
        mtctr %r10

.Lmisaligned_128_loop:
        // while counter > 0, copy quadword.
        ld %r6,0(%r8)
        std %r6,0(%r9)
        ld %r7,8(%r8)
        std %r7,8(%r9)

        // increment variable addresses.
        addi %r8,%r8,16
        addi %r9,%r9,16

        bdnz .Lmisaligned_128_loop

.Lmisaligned_64_32_8:
        // if variable size >= 8, copy by double word.
        cmpdi %r5,8
        bt  lt, .Lmisaligned_32_8

.Lmisaligned_64:
        // copy by double word.
        srdi %r10,%r5,3
        clrldi %r5,%r5,61
        mtctr %r10

.Lmisaligned_64_loop:
        // while counter > 0, copy double word.
        ld %r6,0(%r8)
        std %r6,0(%r9)

        // increment variable addresses.
        addi %r8,%r8,8
        addi %r9,%r9,8

        bdnz .Lmisaligned_64_loop

.Lmisaligned_32_8:
        // if variable size >= 4, copy by word.
        cmpdi %r5,4
        bt  lt, .Lmisaligned_8

.Lmisaligned_32:
        // copy by word.
        srdi %r10,%r5,2
        clrldi %r5,%r5,62
        mtctr %r10

.Lmisaligned_32_loop:
        // while counter > 0, copy word.
        lwz %r6,0(%r8)
        stw %r6,0(%r9)

        // increment variable addresses.
        addi  %r8,%r8,4
        addi  %r9,%r9,4

        bdnz .Lmisaligned_32_loop

.Lmisaligned_8:
        // if variable size <= 0, finished copying.
        cmpdi %r5,1
        bt  lt, .Lmisaligned_end
        mtctr %r5

.Lmisaligned_8_loop:
        // while counter > 0, copy byte.
        lbz %r6,0(%r8)
        stb %r6,0(%r9)

        // increment variable addresses.
        addi  %r8,%r8,1
        addi  %r9,%r9,1

        bdnz .Lmisaligned_8_loop
        b .Lmisaligned_end

.Lmisaligned_128_64_32_8_rev:
        // src address > dst.
        // set addresses to end of input variables for backward copy.
        add %r8,%r4,%r5
        add %r9,%r3,%r5

        // if variable size >= 16, copy by quadword.
        cmpdi %r5,16
        bt  lt, .Lmisaligned_64_32_8_rev

.Lmisaligned_128_rev:
        // copy by quadword.
        srdi %r10,%r5,4
        clrldi %r5,%r5,60
        mtctr %r10

.Lmisaligned_128_rev_loop:
        // while counter > 0, copy quadword.
        ld %r7,-16(%r8)
        std %r7,-16(%r9)
        ld %r6,-8(%r8)
        std %r6,-8(%r9)

        // decrement variable addresses.
        subi %r8,%r8,16
        subi %r9,%r9,16

        bdnz .Lmisaligned_128_rev_loop

.Lmisaligned_64_32_8_rev:
        // if variable size >= 8, copy by double word.
        cmpdi %r5,8
        bt  lt,.Lmisaligned_32_8_rev

.Lmisaligned_64_rev:
        // copy by double word.
        srdi %r10,%r5,3
        clrldi %r5,%r5,61
        mtctr %r10

.Lmisaligned_64_rev_loop:
        // while counter > 0, copy word.
        ld %r6,-8(%r8)
        std %r6,-8(%r9)

        // decrement variable addresses.
        subi %r8,%r8,8
        subi %r9,%r9,8

        bdnz .Lmisaligned_64_rev_loop

.Lmisaligned_32_8_rev:
        // if variable size >= 4, copy by word.
        cmpdi %r5,4
        bt  lt, .Lmisaligned_8_rev

.Lmisaligned_32_rev:
        // copy word.
        srdi %r10,%r5,2
        clrldi %r5,%r5,62
        mtctr %r10

.Lmisaligned_32_rev_loop:
        // while counter > 0, copy word.
        lwz %r6,-4(%r8)
        stw %r6,-4(%r9)

        // decrement variable addresses.
        subi  %r8,%r8,4
        subi  %r9,%r9,4

        bdnz .Lmisaligned_32_rev_loop

.Lmisaligned_8_rev:
        // if variable size <= 0, finished copying.
        cmpdi %r5,1
        bt  lt, .Lmisaligned_end
        mtctr %r5

.Lmisaligned_8_rev_loop:
        // while counter > 0, copy byte.
        // copy byte.
        lbz %r6,-1(%r8)
        stb %r6,-1(%r9)

        // decrement variable addresses.
        subi  %r8,%r8,1
        subi  %r9,%r9,1

        bdnz  .Lmisaligned_8_rev_loop

.Lmisaligned_end:
        // finished copying.
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

