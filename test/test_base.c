// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Versions, result names, the SIMD backend and the result attribute.

#include "test_harness.h"

#include "maul-anim/base.h"

#include <string.h>

// The suites build as C23, where every compiler the family takes has
// [[nodiscard]], so MANI_NODISCARD expands to it and a dropped status
// warns (GCC 14 reports its C23 as 202000L).
#define TEXT(x)     #x
#define EXPANDED(x) TEXT(x)
static_assert(sizeof(EXPANDED(MANI_NODISCARD)) > 1, "MANI_NODISCARD is the attribute in C23");

static void TestVersionMatchesHeader(void)
{
    maniVersion version = maniGetVersion();
    CHECK(version.major == MANI_VERSION_MAJOR, "major version");
    CHECK(version.minor == MANI_VERSION_MINOR, "minor version");
    CHECK(version.patch == MANI_VERSION_PATCH, "patch version");
}

static void TestResultNames(void)
{
    CHECK(strcmp(maniGetResultName(mani_success), "mani_success") == 0, "success name");
    CHECK(strcmp(maniGetResultName(mani_errorInvalid), "mani_errorInvalid") == 0, "invalid name");
    CHECK(strcmp(maniGetResultName(mani_errorCapacity), "mani_errorCapacity") == 0,
          "capacity name");
    CHECK(strcmp(maniGetResultName(mani_errorFormat), "mani_errorFormat") == 0, "format name");
    CHECK(strcmp(maniGetResultName(mani_errorVersion), "mani_errorVersion") == 0, "version name");
    CHECK(strcmp(maniGetResultName(mani_errorLimit), "mani_errorLimit") == 0, "limit name");
    CHECK(strcmp(maniGetResultName(mani_errorBusy), "mani_errorBusy") == 0, "busy name");
    // The values every library of the family gives these results, so a
    // program that links several reads a raw value one way.
    CHECK(mani_errorInvalid == -1 && mani_errorCapacity == -2 && mani_errorVersion == -7 &&
              mani_errorFormat == -8 && mani_errorLimit == -9 && mani_errorBusy == -10,
          "the family's values");
    CHECK(strcmp(maniGetResultName(12345), "unknown result") == 0, "unknown name");
}

// The backend is the one the build chose, the three reports agree, and
// every machine that runs the suites runs the backend it was built for.
static void TestSimdBackend(void)
{
    const char* name = maniGetSimdBackend();
#if defined(MAUL_ANIM_BUILT_FOR_AVX2)
    CHECK(strcmp(name, "avx2") == 0, "an AVX2 build names avx2");
#elif defined(MAUL_ANIM_SIMD_FORCE_SCALAR)
    CHECK(strcmp(name, "scalar") == 0, "a scalar build names scalar");
#elif defined(__aarch64__) || defined(_M_ARM64)
    CHECK(strcmp(name, "neon") == 0, "an arm64 build names neon");
#else
    CHECK(strcmp(name, "scalar") == 0, "another target names scalar");
#endif
    CHECK(maniIsCpuBackendSupported(), "this CPU runs the built backend");
    const char* checked = NULL;
    CHECK(maniCheckCpu(&checked), "the check passes");
    CHECK(checked == name, "the check names the same static string");
    CHECK(maniCheckCpu(NULL), "the check takes no name");
}

int main(void)
{
    TestVersionMatchesHeader();
    TestResultNames();
    TestSimdBackend();
    return s_failures == 0 ? 0 : 1;
}
