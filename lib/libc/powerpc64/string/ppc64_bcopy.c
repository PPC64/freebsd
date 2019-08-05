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

#include <sys/types.h>
#include <sys/auxv.h>
#include <machine/cpu.h>
#include <machine/atomic.h>
#include <stdlib.h>

#ifdef MEMCOPY
extern int bcopy_has_vsx;
extern void* memcpy_plain(void *dst, const void *src, size_t len);
extern void* memcpy_vsx(void *dst, const void *src, size_t len);

void* memcpy(void *dst, const void *src, size_t len)
#elif defined(MEMMOVE)
extern int bcopy_has_vsx;
extern void* bcopy_plain(void *dst, const void *src, size_t len);
extern void* bcopy_vsx(void *dst, const void *src, size_t len);

void* memmove(void *dst, const void *src, size_t len)
#else
int bcopy_has_vsx = -1;
extern void* bcopy_plain(void *dst, const void *src, size_t len);
extern void* bcopy_vsx(void *dst, const void *src, size_t len);

void bcopy(const void *src, void *dst, size_t len)
#endif
{
	/* XXX: all of this should be replaced with ifunc code once it's available */
	if (bcopy_has_vsx < 0) {
		/*
		 * Initialize bcopy_has_vsx to 0, at least until elf_aux_info() returns.
		 * Otherwise, if elf_aux_info() calls bcopy/memcpy/memmove, we would enter an infinite loop.
		 */
		if (atomic_cmpset_int(&bcopy_has_vsx, -1, 0) != 0) {
			u_long hwcap;

			if (elf_aux_info(AT_HWCAP, &hwcap, sizeof(hwcap)) == 0 &&
			    (hwcap & PPC_FEATURE_HAS_VSX) != 0) {
				atomic_set_int(&bcopy_has_vsx, 1);
			}
		}
	}

	if (bcopy_has_vsx > 0) {
		/* VSX is supported */
#if defined(MEMCOPY)
		return memcpy_vsx(dst, src, len);
#elif defined(MEMMOVE)
		return bcopy_vsx(dst, src, len);
#else
		bcopy_vsx(dst, src, len);
#endif
	} else {
		/* VSX is not supported */
#if defined(MEMCOPY)
		return memcpy_plain(dst, src, len);
#elif defined(MEMMOVE)
		return bcopy_plain(dst, src, len);
#else
		bcopy_plain(dst, src, len);
#endif
	}
}
