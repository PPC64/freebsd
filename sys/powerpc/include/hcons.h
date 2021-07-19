#ifndef POWERPC_INCLUDE_HCONS
#define POWERPC_INCLUDE_HCONS

#define	HACKED		1
#define	CATCH_PANIC	0
#define	LATE_CNINIT	0

#ifndef __ASSEMBLER__

#include <sys/systm.h>

#include <machine/cpu.h>
#include <machine/spr.h>
#include <machine/vmparam.h>

#if HACKED
void hacked_puts(const char *str);
void hacked_printf(const char *fmt, ...);
int hacked_cngetc(void);
void hacked_cnprobe(void);

#define HPUTS(str)				\
	do {					\
		hacked_puts(str);		\
		DELAY(500 * 1000);		\
	} while (0)

#define HPRINTF(fmt, ...)	hacked_printf(fmt, ## __VA_ARGS__)

inline static void
reboot_on_keypress(void)
{
	HPUTS("press any key to reboot");
	for (;;) {
		DELAY(100 * 1000);
		if (hacked_cngetc() != -1)
			break;
	}

	cpu_reset();
}

extern vm_offset_t	__startkernel;

#define REL(x)		((x) - (__startkernel - KERNBASE))

inline static void
catch_watchpoint(struct trapframe *frame)
{
	HPRINTF("__startkernel 0x%lx rel 0x%lx\n",
		__startkernel, __startkernel - KERNBASE);
	HPRINTF("srr0 0x%lx 0x%lx lr 0x%lx 0x%lx\n",
		frame->srr0, REL(frame->srr0),
		frame->lr, REL(frame->lr));
	HPRINTF("sp 0x%lx dar 0x%lx\n",
		frame->fixreg[1], frame->dar);
	/* backtrace */
	{
		uintptr_t sp = frame->fixreg[1];
		uint64_t *p;
		int i;

		for (i = 1; i <= 4; i++) {
			sp = *(uintptr_t *)sp;
			p = (uint64_t *)sp;

			HPRINTF("%d 0x%lx 0x%lx\n", i, p[2], REL(p[2]));
		}
	}

	reboot_on_keypress();
}


#else				/* !HACKED */
#define HPUTS(str)
#define HPRINTF(fmt, ...)
#endif


/* Data Address Watchpoint */
inline static void
daddr_watch(void *p)
{
#define DAWR0	180
#define DAWRX0	188

	unsigned long addr = (unsigned long)p;
	unsigned long val;

	val =
		0x40	|	// DW
		0x8	|	// WTI
		0x4	|	// PRIVM(61:63) = HYP
		0;

	mtspr(DAWR0,  addr & ~7);
	mtspr(DAWRX0, val);
}

#endif	/* !defined(__ASSEMBLER__) */

#if HACKED
#define	KDB_ON_DISITRAP	0
#else
#define	KDB_ON_DISITRAP	1
#endif

#endif
