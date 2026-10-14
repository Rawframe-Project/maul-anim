// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening an FBX document (mani-0012) from any bytes, then reading
// every property of every node: numbers, integers, strings (decoded and
// compared), and arrays up to a million elements; binary or ASCII; and
// every object's name, a property, its connections and its local
// transform; every model's children and bone name, the worlds, and
// every skeleton's models. Run as fuzz_fbx <corpus dir>;
// the seed comes from test_fbx --seed <file>.

#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_transform.h"
#include "fbx_units.h"
#include "fbx_value.h"
#include "fbx_world.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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
        for (uint32_t o = 0; o < fbx->objectCount; ++o)
        {
            uint8_t name[64];
            (void)maniFbxObjectName(fbx, o, name, sizeof name);
            uint32_t entry = maniFbxFindProperty(fbx, o, "Lcl Translation", 15);
            double d = 0.0;
            (void)maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, 0), &d);
            const uint32_t* list = NULL;
            (void)maniFbxChildConnections(fbx, o, &list);
            (void)maniFbxParentConnections(fbx, o, &list);
            (void)maniFbxFindObject(fbx, fbx->objects[o].id);
            maniFbxLocal local;
            (void)maniFbxModelLocal(fbx, o, &local);
        }
        for (uint32_t m = 0; m <= fbx->modelCount; ++m)
        {
            const uint32_t* children = NULL;
            uint32_t length = 0;
            (void)maniFbxModelChildren(fbx, m, &children);
            (void)maniFbxModelBoneName(fbx, m, &length);
        }
        maniFbxWorld* worlds = malloc(((size_t)fbx->modelCount + 1) * sizeof(maniFbxWorld));
        if (worlds != NULL)
        {
            (void)maniFbxEvaluateWorlds(fbx, worlds);
            free(worlds);
        }
        uint8_t* marked = malloc((size_t)fbx->modelCount + 1);
        for (uint32_t k = 0; marked != NULL && k < maniFbxSkeletonCount(fbx); ++k)
        {
            memset(marked, 0, (size_t)fbx->modelCount + 1);
            maniFbxMarkSkeleton(fbx, k, marked);
        }
        free(marked);
        for (uint32_t v = 0; v < fbx->nodeCount; ++v)
        {
            (void)maniFbxFindChild(fbx, v, "P", 1);
        }
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
    return 0;
}
