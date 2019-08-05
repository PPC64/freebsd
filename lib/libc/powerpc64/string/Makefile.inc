# $FreeBSD$

SRCS+= \
	ppc64_bcopy.c \
	ppc64_memcpy.c \
	memmove.c

MDSRCS+= \
	bcopy.S \
	bcopy_vsx.S \
	memcpy.S \
	memcpy_vsx.S
	strcpy_arch_2_05.S \
	strcpy.c \
	strcpy_resolver.c \
	strncpy_arch_2_05.S \
	strncpy.c \
	strncpy_resolver.c
