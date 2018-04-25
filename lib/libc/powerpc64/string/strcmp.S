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
        RCSID("$NetBSD: strcmp.S,v 1.0 2018/04/16 08:34:02 alexandre Exp $")
#endif

ENTRY(strcmp)

.Lstrcmp_loop:
        /* Load double words. */
        ld %r5,0(%r3)
        ld %r6,0(%r4)

        /* Check if double words are different. */
        xor %r0,%r5,%r6

        /* Check if double words contain zero. */
        xor %r8,%r8,%r8
        cmpb %r9,%r5,%r8

        /*
         * If double words are different or contain zero,
         * find what byte is different or contains zero,
         * else load next double words.
         */
        or. %r9,%r9,%r0
        bne .Lstrcmp_check_zeros_differences

        /* Increment data addresses. */
        addi  %r3,%r3,8
        addi  %r4,%r4,8
        b .Lstrcmp_loop

.Lstrcmp_check_zeros_differences:
        /* Find what byte is different or contains zero. */

        /* Load masks. */
        lis %r7,0xff00
        sldi %r5,%r7,8
        sldi %r6,%r5,8
        sldi %r8,%r6,8
        sldi %r10,%r8,8

        /* Check 1st byte. */
        and. %r0,%r9,%r10
        bne .Lstrcmp_found_zero_difference_in_1

        /* Check 2nd byte. */
        and. %r0,%r9,%r8
        bne .Lstrcmp_found_zero_difference_in_2

        /* Check 3rd byte. */
        and. %r0,%r9,%r6
        bne .Lstrcmp_found_zero_difference_in_3

        /* Check 4th byte. */
        and. %r0,%r9,%r5
        bne .Lstrcmp_found_zero_difference_in_4

        /* Check 5th byte. */
        and. %r0,%r9,%r7
        bne .Lstrcmp_found_zero_difference_in_5

        /* Check 6th byte. */
        andis. %r0,%r9,0xff
        bne .Lstrcmp_found_zero_difference_in_6

        /* Check 7th byte. */
        andi. %r0,%r9,0xff00
        bne .Lstrcmp_found_zero_difference_in_7

        /* 8th byte is different or contains zero. */
        lbz %r5,7(%r3)
        lbz %r6,7(%r4)
        sub %r3,%r5,%r6
        blr

.Lstrcmp_found_zero_difference_in_1:
        /* 1st byte is different or contains zero. */
        lbz %r5,0(%r3)
        lbz %r6,0(%r4)
        sub %r3,%r5,%r6
        blr

.Lstrcmp_found_zero_difference_in_2:
        /* 2nd byte is different or contains zero. */
        lbz %r5,1(%r3)
        lbz %r6,1(%r4)
        sub %r3,%r5,%r6
        blr

.Lstrcmp_found_zero_difference_in_3:
        /* 3rd byte is different or contains zero. */
        lbz %r5,2(%r3)
        lbz %r6,2(%r4)
        sub %r3,%r5,%r6
        blr

.Lstrcmp_found_zero_difference_in_4:
        /* 4th byte is different or contains zero. */
        lbz %r5,3(%r3)
        lbz %r6,3(%r4)
        sub %r3,%r5,%r6
        blr

.Lstrcmp_found_zero_difference_in_5:
        /* 5th byte is different or contains zero. */
        lbz %r5,4(%r3)
        lbz %r6,4(%r4)
        sub %r3,%r5,%r6
        blr

.Lstrcmp_found_zero_difference_in_6:
        /* 6th byte is different or contains zero. */
        lbz %r5,5(%r3)
        lbz %r6,5(%r4)
        sub %r3,%r5,%r6
        blr

.Lstrcmp_found_zero_difference_in_7:
        /* 7th byte is different or contains zero. */
        lbz %r5,6(%r3)
        lbz %r6,6(%r4)
        sub %r3,%r5,%r6
        blr

END(strcmp)

        .section .note.GNU-stack,"",%progbits
