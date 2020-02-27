/*-
 * Copyright (c) 2017 Enji Cooper <ngie@freebsd.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/sysctl.h>
#include <errno.h>
#include <fcntl.h>
#include <kvm.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <atf-c.h>

#include "kvm_private.h"

#include "kvm_test_common.h"

/* 1100090 was where kvm_open2(3) was introduced. */
#if __FreeBSD_version >= 1100091
ATF_TC(kvm_read_positive_test_no_error);
ATF_TC_HEAD(kvm_read_positive_test_no_error, tc)
{

	atf_tc_set_md_var(tc, "descr",
	    "test that kvm_geterr(kd) when kd contains an error returns an error message");
	atf_tc_set_md_var(tc, "require.user", "root");
}

ATF_TC_BODY(kvm_read_positive_test_no_error, tc)
{
#define	ALL_IS_WELL	"that ends well"
	kvm_t *kd;
	char *error_msg;
	struct nlist nl[] = {
#define	SYMNAME	"_mp_maxcpus"
#define	X_MAXCPUS	0
		{ SYMNAME, 0, 0, 0, 0 },
		{ NULL, 0, 0, 0, 0 },
	};
	ssize_t rc;
	int mp_maxcpus, retcode;

	errbuf_clear();
	kd = kvm_open2(NULL, NULL, O_RDONLY, errbuf, NULL);
	ATF_CHECK(!errbuf_has_error(errbuf));
	ATF_REQUIRE_MSG(kd != NULL, "kvm_open2 failed: %s", errbuf);
	retcode = kvm_nlist(kd, nl);
	ATF_REQUIRE_MSG(retcode != -1,
	    "kvm_nlist failed (returned %d): %s", retcode, kvm_geterr(kd));
	if (nl[X_MAXCPUS].n_type == 0)
		atf_tc_skip("symbol (\"%s\") couldn't be found", SYMNAME);
	_kvm_err(kd, NULL, "%s", ALL_IS_WELL); /* XXX: internal API */
	rc = kvm_read(kd, nl[X_MAXCPUS].n_value, &mp_maxcpus,
	    sizeof(mp_maxcpus));

	ATF_REQUIRE_MSG(rc != -1, "kvm_read failed: %s", kvm_geterr(kd));
	error_msg = kvm_geterr(kd);
	ATF_REQUIRE_MSG(strcmp(error_msg, ALL_IS_WELL) == 0,
	    "error message changed: %s", error_msg);
	ATF_REQUIRE_MSG(kvm_close(kd) == 0, "kvm_close failed: %s",
	    strerror(errno));

	/* Check if value read from kvm_read is sane */
	int sysctl_maxcpus = 0;
	size_t len = sizeof(sysctl_maxcpus);
        retcode = sysctlbyname("kern.smp.maxcpus", &sysctl_maxcpus, &len, NULL, 0);
	ATF_REQUIRE_MSG(retcode == 0, "sysctl read failed : %d", retcode);
	ATF_REQUIRE_EQ_MSG(mp_maxcpus, sysctl_maxcpus,
	    "failed: kvm_read of mp_maxcpus returned %d but sysctl maxcpus returned %d",
	    mp_maxcpus, sysctl_maxcpus);
}
#endif

ATF_TP_ADD_TCS(tp)
{

#if __FreeBSD_version >= 1100091
	ATF_TP_ADD_TC(tp, kvm_read_positive_test_no_error);
#endif

	return (atf_no_error());
}
