// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening an FBX document (mani-0012) from any bytes, then reading
// every property of every node: numbers, integers, strings (decoded and
// compared), and arrays up to a million elements; binary or ASCII. Run as fuzz_fbx <corpus dir>;
// the seed comes from test_fbx --seed <file>.

#include "fbx_document.h"
#include "fbx_value.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void ReadAll(const maniFbx* fbx)
{
    for (uint32_t p = 0; p < fbx->propertyCount; ++p)
    {
        double d = 0.0;
        int64_t n = 0;
        const uint8_t* text = NULL;
        uint32_t length = 0;
        (void)maniFbxNumber(fbx, p, &d);
        (void)maniFbxInteger(fbx, p, INT64_MIN, INT64_MAX, &n);
        (void)maniFbxString(fbx, p, &text, &length);
        uint8_t decoded[64];
        (void)maniFbxStringDecode(fbx, p, decoded, sizeof decoded);
        (void)maniFbxStringEquals(fbx, p, "Model", 5);
        uint32_t count = maniFbxArrayCount(fbx, p);
        // Large arrays cost time, not coverage.
        if (count == 0 || count > (1u << 20))
        {
            continue;
        }
        void* out = malloc((size_t)count * sizeof(double));
        if (out != NULL)
        {
            (void)maniFbxReadDoubles(fbx, p, out);
            (void)maniFbxReadIntegers(fbx, p, out);
            free(out);
        }
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) != mani_success)
    {
        return 0;
    }
    maniFbx* fbx = NULL;
    if (maniOpenFbx(builder, data, size, &fbx).result == mani_success)
    {
        ReadAll(fbx);
        for (uint32_t v = 0; v < fbx->nodeCount; ++v)
        {
            (void)maniFbxFindChild(fbx, v, "P", 1);
        }
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
    return 0;
}
