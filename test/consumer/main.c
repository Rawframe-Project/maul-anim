// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The CPU check as a host makes it, through the installed public header
// alone: the backend the package names is the one the library reports,
// and this CPU runs it.

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    const char* backend = NULL;
    bool runs = maniCheckCpu(&backend);
    maniVersion version = maniGetVersion();
    printf("maul-anim %u.%u.%u: backend %s, this CPU %s it\n", (unsigned)version.major,
           (unsigned)version.minor, (unsigned)version.patch, backend != NULL ? backend : "(none)",
           runs ? "runs" : "does not run");
    bool named = backend != NULL && strcmp(backend, maniGetSimdBackend()) == 0 &&
                 strcmp(backend, CONSUMER_BACKEND) == 0;
    return runs && maniIsCpuBackendSupported() && named ? 0 : 1;
}
