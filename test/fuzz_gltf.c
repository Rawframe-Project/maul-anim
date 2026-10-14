// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a glTF document (mani-0008) from any bytes, then reading every
// accessor, with each external buffer given the input's own bytes when
// they are long enough, and building every skeleton, skin and clip and
// reading the spring bones. Run as fuzz_gltf <corpus dir>;
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
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

// The builder's memory, at most 16 MiB live: a few bytes can ask for a
// clip of millions of samples, whose cost is time, not coverage, and
// past the budget the capacity refusals are exercised instead.
#define FUZZ_BUDGET ((size_t)16 << 20)

static size_t s_live;

// Aligned by hand, as aligned_alloc is not everywhere; the block's start
// sits just before the aligned memory.
static void* Allocate(size_t size, size_t alignment, void* context)
{
    (void)context;
    size_t extra = alignment + sizeof(void*);
    if (size > FUZZ_BUDGET - s_live || size > SIZE_MAX - extra)
    {
        return NULL;
    }
    uint8_t* block = malloc(size + extra);
    if (block == NULL)
    {
        return NULL;
    }
    uintptr_t at = ((uintptr_t)block + sizeof(void*) + alignment - 1) & ~(uintptr_t)(alignment - 1);
    memcpy((uint8_t*)at - sizeof(void*), &block, sizeof block);
    s_live += size;
    return (void*)at;
}

static void Release(void* memory, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    if (memory != NULL)
    {
        void* block = NULL;
        memcpy(&block, (uint8_t*)memory - sizeof(void*), sizeof block);
        s_live -= size;
        free(block);
    }
}

// Every skeleton, with and without folding and with a basis change, and
// each of its skins against it; the first node's subtree.
static void BuildAll(maniBuilder* builder, maniGltf* gltf)
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
            maniSpringSetSource springs;
            (void)maniGltfReadSprings(gltf, &skeleton, &options[o], &springs);
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
    def.allocator = (maniAllocator){Allocate, Release, NULL};
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
