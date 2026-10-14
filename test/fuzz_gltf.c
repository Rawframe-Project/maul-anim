// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a glTF document (mani-0008) from any bytes, then reading every
// accessor, with each external buffer given the input's own bytes when
// they are long enough. Run as fuzz_gltf <corpus dir>; the seed comes
// from test_gltf --seed <file>.

#include "gltf_document.h"
#include "gltf_layout.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) != mani_success)
    {
        return 0;
    }
    maniGltf* gltf = NULL;
    if (maniOpenGltf(builder, data, size, &gltf).result == mani_success)
    {
        for (uint32_t b = 0; b < maniGltfBufferCount(gltf); ++b)
        {
            if (maniGltfBufferUri(gltf, b, NULL) != NULL)
            {
                (void)maniGltfSupplyBuffer(gltf, b, data, size);
            }
        }
        for (uint32_t i = 0; i < gltf->accessorCount; ++i)
        {
            const maniGltfAccessor* a = &gltf->accessors[i];
            size_t floats = (size_t)a->count * a->rows * a->columns;
            // Large implicit accessors cost time, not coverage.
            if (floats > ((size_t)1 << 20))
            {
                continue;
            }
            float* out = malloc(floats * sizeof(float));
            if (out != NULL)
            {
                (void)maniGltfReadFloats(gltf, i, out);
                free(out);
            }
        }
        maniCloseGltf(gltf);
    }
    maniDestroyBuilder(builder);
    return 0;
}
