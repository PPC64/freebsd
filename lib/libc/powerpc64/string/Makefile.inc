# $FreeBSD$

SRCS+= \
	ppc64_bcopy.c \
	ppc64_memcpy.c \
	ppc64_memmove.c

MDSRCS+= \
	bcopy.S \
	bcopy_vsx.S \
	memcpy.S \
	memcpy_vsx.S \
	memmove.S \
	memmove_vsx.S \
	strncpy.S \
	strcmp.S \
	strcpy.S