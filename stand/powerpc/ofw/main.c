/*-
 * Copyright (c) 2000 Benno Rice <benno@jeamland.net>
 * Copyright (c) 2000 Stephane Potvin <sepotvin@videotron.ca>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <stand.h>
#include "openfirm.h"
#include "libofw.h"
#include "bootstrap.h"

#include <machine/psl.h>

struct arch_switch	archsw;		/* MI/MD interface boundary */

extern char end[];

uint32_t	acells, scells;

static char bootargs[128];

#define	HEAP_SIZE	0x800000
static char heap[HEAP_SIZE]; // In BSS, so uses no space

#define OF_puts(fd, text) OF_write(fd, text, strlen(text))

static __inline register_t
mfmsr(void)
{
	register_t value;

	__asm __volatile ("mfmsr %0" : "=r"(value));

	return (value);
}

void
init_heap(void)
{
	bzero(heap, HEAP_SIZE);

	setheap(heap, (void *)((int)heap + HEAP_SIZE));
}

uint64_t
memsize(void)
{
	phandle_t	memoryp;
	cell_t		reg[24];
	int		i, sz;
	uint64_t	memsz;

	memsz = 0;
	memoryp = OF_instance_to_package(memory);

	sz = OF_getprop(memoryp, "reg", &reg, sizeof(reg));
	sz /= sizeof(reg[0]);

	for (i = 0; i < sz; i += (acells + scells)) {
		if (scells > 1)
			memsz += (uint64_t)reg[i + acells] << 32;
		memsz += reg[i + acells + scells - 1];
	}

	return (memsz);
}

int
main(int (*openfirm)(void *))
{
	phandle_t	root;
	int		i;
	char		bootpath[64];
	char		*ch;
	int		bargc;
	char		**bargv;

	/*
	 * Initialise the Open Firmware routines by giving them the entry point.
	 */
	OF_init(openfirm);

	root = OF_finddevice("/");

	scells = acells = 1;
	OF_getprop(root, "#address-cells", &acells, sizeof(acells));
	OF_getprop(root, "#size-cells", &scells, sizeof(scells));

	/*
	 * Initialise the heap as early as possible.  Once this is done,
	 * alloc() is usable. The stack is buried inside us, so this is
	 * safe.
	 */
	init_heap();

	/*
         * Set up console.
         */
	cons_probe();

	/*
	 * March through the device switch probing for things.
	 */
	for (i = 0; devsw[i] != NULL; i++)
		if (devsw[i]->dv_init != NULL)
			(devsw[i]->dv_init)();

	printf("\n%s", bootprog_info);
	printf("Memory: %lldKB\n", memsize() / 1024);

	OF_getprop(chosen, "bootpath", bootpath, 64);
	ch = strchr(bootpath, ':');
	*ch = '\0';
	printf("Booted from: %s\n", bootpath);

	printf("\n");

	/*
	 * Only parse the first bootarg if present. It should
	 * be simple to handle extra arguments
	 */
	OF_getprop(chosen, "bootargs", bootargs, sizeof(bootargs));
	bargc = 0;
	parse(&bargc, &bargv, bootargs);
	if (bargc == 1)
		env_setenv("currdev", EV_VOLATILE, bargv[0], ofw_setcurrdev,
		    env_nounset);
	else
		env_setenv("currdev", EV_VOLATILE, bootpath,
			   ofw_setcurrdev, env_nounset);
	env_setenv("loaddev", EV_VOLATILE, bootpath, env_noset,
	    env_nounset);
	setenv("LINES", "24", 1);		/* optional */

	/*
	 * On non-Apple hardware, where it works reliably, pass flattened
	 * device trees to the kernel by default instead of OF CI pointers.
	 * Apple hardware is the only virtual-mode OF implementation in
	 * existence, so far as I am aware, so use that as a flag.
	 */
	if (!(mfmsr() & PSL_DR))
		setenv("usefdt", "1", 1);

	archsw.arch_getdev = ofw_getdev;
	archsw.arch_copyin = ofw_copyin;
	archsw.arch_copyout = ofw_copyout;
	archsw.arch_readin = ofw_readin;
	archsw.arch_autoload = ofw_autoload;

	interact();				/* doesn't return */

	OF_exit();

	return 0;
}

COMMAND_SET(halt, "halt", "halt the system", command_halt);

static int
command_halt(int argc, char *argv[])
{

	OF_exit();
	return (CMD_OK);
}

COMMAND_SET(memmap, "memmap", "print memory map", command_memmap);

int
command_memmap(int argc, char **argv)
{

	ofw_memmap(acells);
	return (CMD_OK);
}

COMMAND_SET(setov5, "setov5", "Set LoPAPR Option Vector 5", command_setov5);

struct pvr {
	uint32_t	mask;
	uint32_t	val;
};

struct opt_vec_ignore {
	char	data[2];
} __packed;

struct opt_vec4 {
	char data[3];
} __packed;

/* byte 2 */
#define OV5_LPAR	0x80
#define OV5_SPLPAR	0x40
#define OV5_DRMEM	0x20
#define OV5_LP		0x10
#define OV5_ALPHA_PART	0x08
#define OV5_DMA_DELAY	0x04
#define OV5_DONATE_CPU	0x02
#define OV5_MSI		0x01

/* byte 5 */
#define OV5_ASSOC	0x80
#define OV5_PRRN	0x40

/* byte 17 */
#define OV5_RNG		0x80
#define OV5_COMP_ENG	0x40
#define OV5_ENC_ENG	0x20

#define PVR_VER_P8E	0x004b0000
#define PVR_VER_P8NVL	0x004c0000
#define PVR_VER_P8	0x004d0000
#define PVR_VER_P9	0x004e0000
#define PVR_VER_MASK	0xffff0000

struct opt_vec5 {
	char data[27];
} __packed;

static struct ibm_arch_vec {
	struct pvr		pvr_list[5];
	uint8_t			num_opts;
	struct opt_vec_ignore	vec1;
	struct opt_vec_ignore	vec2;
	struct opt_vec_ignore	vec3;
	struct opt_vec4		vec4;
	struct opt_vec5		vec5;
} __packed ibm_arch_vec;

static int
ppc64_set_arch_options(void)
{
	int i, rc;
	char *vec;
	struct pvr *pvr;
	ihandle_t ihandle;
	cell_t err;

	/* Match POWER8/POWER8E/POWER8NVL/POWER9 */
	pvr = ibm_arch_vec.pvr_list;
	pvr->mask = PVR_VER_MASK;
	pvr->val = PVR_VER_P8;
	pvr++;
	pvr->mask = PVR_VER_MASK;
	pvr->val = PVR_VER_P8E;
	pvr++;
	pvr->mask = PVR_VER_MASK;
	pvr->val = PVR_VER_P8NVL;
	pvr++;
	pvr->mask = PVR_VER_MASK;
	pvr->val = PVR_VER_P9;

	/* Insert a match-any-PVR terminator */
	pvr++;
	pvr->mask = 0;
	pvr->val = 0xffffffffu;

	/* Note: 4 actually means 5 option vectors */
	ibm_arch_vec.num_opts = 4;

	/* Set ignored vectors */
	vec = ibm_arch_vec.vec1.data;
	for (i = 0; i < 3; i++, vec += sizeof(struct opt_vec_ignore)) {
		vec[0] = 0;	/* length (n - 2) */
		vec[1] = 0x80;	/* ignore */
	}

	/* Set Option Vector 4 (can't be ignored) */
	vec = ibm_arch_vec.vec4.data;
	vec[0] = sizeof(struct opt_vec4) - 2;	/* length */
	vec[1] = 0;
	vec[2] = 10;	/* default: 10% */

	/* Set Option Vector 5 */
	vec = ibm_arch_vec.vec5.data;
	vec[0] = sizeof(struct opt_vec5) - 2;	/* length */
	vec[1] = 0;	/* don't ignore */
	vec[2] = OV5_LPAR | OV5_SPLPAR | OV5_LP | OV5_MSI;
	/* Max processors: 256 (hardcoded for now) */
	vec[9] = 0;
	vec[10] = 0;
	vec[11] = 1;
	vec[12] = 0;
	/* LoPAPR Level */
	vec[13] = 1;
	vec[14] = 1;
	/* Platform Facilities */
	vec[17] = OV5_RNG | OV5_COMP_ENG | OV5_ENC_ENG;
	/* 21 = 0 - subprocessors not supported */
	/* 22: DRMEM_V2 */
	/* 23 = 0 - XICS*/
	/* 24 = 0 - HPT */
	/* 25 = 0 - Segment Tables == no, GTSE == no */
	/* 26 = 0 - Radix: GTSE == no */

	ihandle = OF_open("/");
	if (ihandle == -1) {
		printf("OF_open() error\n");
		return (-1);
	}

	if (rc = OF_call_method("ibm,client-architecture-support",
	    ihandle, 1, 1, &ibm_arch_vec, &err))
		printf("Failed to call ibm,client-architecture-support method\n");
	else
		printf("err=0x%08lX\n", err);

	OF_close(ihandle);
	return (rc);
}


static int
command_setov5(int argc __unused, char *argv[] __unused)
{
	ppc64_set_arch_options();
	return (CMD_OK);
}
