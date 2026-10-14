// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a glTF document (mani-0008) from any bytes, then reading every
// accessor, with each external buffer given the input's own bytes when
// they are long enough, and building every skeleton, skin and clip. Run as fuzz_gltf <corpus dir>;
// the seed comes from test_gltf --seed <file>.

#include "gltf_document.h"
#include "gltf_layout.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

// Every skeleton, with and without folding and with a basis change, and
// each of its skins against it; the first node's subtree.
static void BuildAll(maniBuilder* builder, const maniGltf* gltf)
{
    static _Alignas(16) uint8_t file[1 << 20];
    const maniGltfOptions options[2] = {{{maniAxisX, maniAxisY, maniAxisZ}, 0},
                                        {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 1}};
    for (uint32_t k = 0; k < maniGltfSkeletonCount(gltf); ++k)
    {
        for (uint32_t o = 0; o < 2; ++o)
        {
            maniAffine placement;
            (void)maniGltfSkeletonPlacement(gltf, k, &options[o], &placement);
            size_t size = 0;
            maniSkeleton skeleton;
            if (maniGltfBuildSkeleton(builder, gltf, k, &options[o]).result != mani_success ||
                maniCopyBuilt(builder, file, sizeof file, &size) != mani_success ||
                maniOpenSkeleton(file, size, &skeleton).result != mani_success)
            {
                continue;
            }
            for (uint32_t s = 0; s < maniGltfSkinCount(gltf); ++s)
            {
                if (maniGltfSkinSkeleton(gltf, s) == k)
                {
                    (void)maniGltfBuildSkin(builder, gltf, s, &skeleton, &options[o]);
                }
            }
            for (uint32_t a = 0; a < maniGltfAnimationCount(gltf); ++a)
            {
                uint32_t bound = 0;
                (void)maniGltfBuildClip(builder, gltf, a, &skeleton, &options[o], 10.0f,
                                        maniLoopClamp, &bound);
            }
        }
    }
    if (maniGltfNodeCount(gltf) > 0)
    {
        (void)maniGltfBuildNodeSkeleton(builder, gltf, 0, &options[0]);
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
        BuildAll(builder, gltf);
        maniCloseGltf(gltf);
    }
    maniDestroyBuilder(builder);
    return 0;
}
