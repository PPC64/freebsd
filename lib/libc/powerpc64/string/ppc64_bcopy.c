#include <sys/types.h>
#include <sys/sysctl.h>
#include <machine/cpu.h>
#include <stdlib.h>

#ifdef MEMCOPY
extern int bcopy_has_vsx;
extern void* memcpy_plain(void *dst, const void *src, size_t len);
extern void* memcpy_vsx(void *dst, const void *src, size_t len);

void* memcpy(void *dst, const void *src, size_t len)
#else
#ifdef MEMMOVE
extern int bcopy_has_vsx;
extern void* memmove_plain(void *dst, const void *src, size_t len);
extern void* memmove_vsx(void *dst, const void *src, size_t len);

void* memmove(void *dst, const void *src, size_t len)
#else
int bcopy_has_vsx = -1;
extern void bcopy_plain(const void *src, void *dst, size_t len);
extern void bcopy_vsx(const void *src, void *dst, size_t len);

void bcopy(const void *src, void *dst, size_t len)
#endif
#endif
{
	/* XXX: all of this should be replaced with ifunc code once it's available */
	if (bcopy_has_vsx < 0) {
	        unsigned int cpu_features;
        	size_t cpu_features_len = sizeof(cpu_features);

	        if (sysctlbyname("hw.cpu_features", &cpu_features, &cpu_features_len, NULL, 0) == 0 &&
		    (cpu_features & PPC_FEATURE_HAS_VSX) != 0) {
			bcopy_has_vsx = 1;
		} else {
			bcopy_has_vsx = 0;
		}
	}

	if (bcopy_has_vsx > 0) {
		/* VSX is supported */
#ifdef MEMCOPY
		return memcpy_vsx(dst, src, len);
#else
#ifdef MEMMOVE
		return memmove_vsx(dst, src, len);
#else
		bcopy_vsx(src, dst, len);
#endif
#endif
	} else {
		/* VSX is not supported */
#ifdef MEMCOPY
		return memcpy_plain(dst, src, len);
#else
#ifdef MEMMOVE
		return memmove_plain(dst, src, len);
#else
		bcopy_plain(src, dst, len);
#endif
#endif
	}
}
