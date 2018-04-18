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
        // while %r5 != %r6 or %r5 != 0, load chars.

        // 1st byte.
        lbz %r5,0(%r3)
        lbz %r6,0(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // 2st byte.
        lbz %r5,1(%r3)
        lbz %r6,1(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // 3rd byte.
        lbz %r5,2(%r3)
        lbz %r6,2(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // 4th byte.
        lbz %r5,3(%r3)
        lbz %r6,3(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // 5th byte.
        lbz %r5,4(%r3)
        lbz %r6,4(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // 6th byte.
        lbz %r5,5(%r3)
        lbz %r6,5(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // 7th byte.
        lbz %r5,6(%r3)
        lbz %r6,6(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // 8th byte.
        lbz %r5,7(%r3)
        lbz %r6,7(%r4)
        cmpdi %r5,0
        beq .Lstrcmp_end
        cmpd  %r5,%r6
        bne .Lstrcmp_end

        // increment data addresses.
        addi  %r3,%r3,8
        addi  %r4,%r4,8

        b .Lstrcmp_loop

.Lstrcmp_end:
        // finished checking chars.
        sub %r3,%r5,%r6
        blr

END(strcmp)

        .section .note.GNU-stack,"",%progbits
