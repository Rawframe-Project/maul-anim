// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Versions and result names.

#include "test_harness.h"

#include "maul-anim/base.h"

#include <string.h>

static void TestVersionMatchesHeader(void)
{
    maniVersion version = maniGetVersion();
    CHECK(version.major == MANI_VERSION_MAJOR, "major version");
    CHECK(version.minor == MANI_VERSION_MINOR, "minor version");
    CHECK(version.patch == MANI_VERSION_PATCH, "patch version");
}

static void TestResultNames(void)
{
    CHECK(strcmp(maniResultName(mani_success), "mani_success") == 0, "success name");
    CHECK(strcmp(maniResultName(mani_errorInvalid), "mani_errorInvalid") == 0, "invalid name");
    CHECK(strcmp(maniResultName(mani_errorCapacity), "mani_errorCapacity") == 0, "capacity name");
    CHECK(strcmp(maniResultName(mani_errorFormat), "mani_errorFormat") == 0, "format name");
    CHECK(strcmp(maniResultName(mani_errorVersion), "mani_errorVersion") == 0, "version name");
    CHECK(strcmp(maniResultName(12345), "unknown result") == 0, "unknown name");
}

int main(void)
{
    TestVersionMatchesHeader();
    TestResultNames();
    return s_failures == 0 ? 0 : 1;
}
