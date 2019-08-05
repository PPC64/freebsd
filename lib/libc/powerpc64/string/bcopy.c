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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <machine/cpu.h>
#include <machine/ifunc.h>

#ifdef MEMCOPY
#define FN_NAME     memcpy
#define FN_RET      void *
#define FN_PARAMS   (void *dst, const void *src, size_t len)

#elif defined(MEMMOVE)
#define FN_NAME     memmove
#define FN_RET      void *
#define FN_PARAMS   (void *dst, const void *src, size_t len)

#else
#define FN_NAME     bcopy
#define FN_RET      void
#define FN_PARAMS   (const void *src, void *dst, size_t len)
#endif

#define CAT(a,b)    a##b
#define CAT3(a,b,c) a##b##c

FN_RET CAT(__, FN_NAME) FN_PARAMS;
FN_RET CAT3(__, FN_NAME, _vsx) FN_PARAMS;


DEFINE_UIFUNC(, FN_RET, FN_NAME, FN_PARAMS)
{
        if (hwcap & PPC_FEATURE_HAS_VSX)
                return (CAT3(__, FN_NAME, _vsx));
        else
                return (CAT(__, FN_NAME));
}
