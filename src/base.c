// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Versions and result names.

#include "maul-anim/base.h"

maniVersion maniGetVersion(void)
{
    return (maniVersion){MANI_VERSION_MAJOR, MANI_VERSION_MINOR, MANI_VERSION_PATCH};
}

const char* maniGetResultName(maniResult result)
{
    switch (result)
    {
    case mani_success:
        return "mani_success";
    case mani_errorInvalid:
        return "mani_errorInvalid";
    case mani_errorCapacity:
        return "mani_errorCapacity";
    case mani_errorVersion:
        return "mani_errorVersion";
    case mani_errorFormat:
        return "mani_errorFormat";
    case mani_errorLimit:
        return "mani_errorLimit";
    case mani_errorBusy:
        return "mani_errorBusy";
    default:
        return "unknown result";
    }
}
