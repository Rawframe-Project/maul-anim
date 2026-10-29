// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a glTF document from any bytes, then reading every
// accessor, with each external buffer given the input's own bytes when
// they are long enough, and building every skeleton, skin and clip,
// reading the spring bones and building the ragdoll read,
// every mesh primitive's influences against
// each skin and its morph targets. Run as fuzz_gltf <corpus dir>;
// the seed comes from test_gltf --seed <file>.

#include "gltf_document.h"
#include "gltf_layout.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

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

// Every primitive's influences against the skin, and morph targets.
static void BuildMeshes(maniBuilder* builder, maniGltf* gltf, const maniSkin* skin,
                        const maniGltfOptions* options)
{
    for (uint32_t m = 0; m < maniGltf_GetMeshCount(gltf) && m < 8; ++m)
    {
        for (uint32_t p = 0; p < maniGltf_GetPrimitiveCount(gltf, m) && p < 8; ++p)
        {
            if (skin != NULL)
            {
                (void)maniBuildGltfInfluences(builder, gltf, m, p, skin);
            }
            (void)maniBuildGltfMorphTargets(builder, gltf, m, p, options);
        }
    }
    for (uint32_t n = 0; n < maniGltf_GetNodeCount(gltf) && n < 8; ++n)
    {
        float weights[8];
        uint32_t mesh = maniGltf_GetNodeMesh(gltf, n);
        uint32_t count = mesh == UINT32_MAX ? 0 : maniGltf_GetPrimitiveTargetCount(gltf, mesh, 0);
        (void)maniGltf_GetNodeSkin(gltf, n);
        if (count > 0 && count <= 8)
        {
            (void)maniGltf_GetMorphWeights(gltf, n, weights, count, NULL);
        }
    }
}

// Every skeleton, with and without folding and with a basis change, and
// each of its skins against it; the first node's subtree.
static void BuildAll(maniBuilder* builder, maniGltf* gltf)
{
    static _Alignas(16) uint8_t file[1 << 20];
    const maniGltfOptions options[2] = {{{maniAxisX, maniAxisY, maniAxisZ}, 0},
                                        {{maniAxisX, maniAxisNegativeZ, maniAxisY}, 1}};
    for (uint32_t k = 0; k < maniGltf_GetSkeletonCount(gltf); ++k)
    {
        for (uint32_t o = 0; o < 2; ++o)
        {
            maniAffine placement;
            (void)maniGltf_GetSkeletonPlacement(gltf, k, &options[o], &placement);
            size_t size = 0;
            maniSkeleton skeleton;
            if (maniBuildGltfSkeleton(builder, gltf, k, &options[o]).result != mani_success ||
                maniBuilder_CopyBuilt(builder, file, sizeof file, &size) != mani_success ||
                maniOpenSkeleton(file, size, &skeleton).result != mani_success)
            {
                continue;
            }
            for (uint32_t s = 0; s < maniGltf_GetSkinCount(gltf); ++s)
            {
                static _Alignas(16) uint8_t skinFile[1 << 16];
                size_t skinSize = 0;
                maniSkin skin;
                if (maniGltf_GetSkinSkeleton(gltf, s) == k &&
                    maniBuildGltfSkin(builder, gltf, s, &skeleton, &options[o]).result ==
                        mani_success &&
                    maniBuilder_CopyBuilt(builder, skinFile, sizeof skinFile, &skinSize) ==
                        mani_success &&
                    maniOpenSkin(skinFile, skinSize, &skin).result == mani_success)
                {
                    BuildMeshes(builder, gltf, &skin, &options[o]);
                }
            }
            maniSpringSetSource springs;
            (void)maniGltf_ReadSprings(gltf, &skeleton, &options[o], &springs);
            // A ragdoll read is a source the builder takes or refuses.
            maniRagdollSource ragdoll;
            if (maniGltf_ReadRagdoll(gltf, &skeleton, &options[o], (maniVec3){0.0f, 0.0f, 1.0f},
                                     (maniVec3){0.0f, 1.0f, 0.0f}, &ragdoll)
                        .result == mani_success &&
                ragdoll.bodyCount > 0)
            {
                (void)maniBuildRagdoll(builder, &ragdoll);
            }
            for (uint32_t a = 0; a < maniGltf_GetAnimationCount(gltf); ++a)
            {
                // Every other animation with the root's travel and turn
                // taken out as root motion.
                uint32_t bound = 0;
                maniRootMotionSource root = {0, 7, maniRootRotationTwist, 1, 0, 1, a & 2u};
                maniImportClipOptions settings = maniDefaultImportClipOptions();
                settings.sampleRate = 10.0f;
                settings.rootMotion = (a & 1u) != 0 ? &root : NULL;
                (void)maniBuildGltfClip(builder, gltf, a, &skeleton, &options[o], &settings,
                                        &bound);
            }
        }
    }
    if (maniGltf_GetNodeCount(gltf) > 0)
    {
        (void)maniBuildGltfNodeSkeleton(builder, gltf, 0, &options[0]);
    }
    BuildMeshes(builder, gltf, NULL, &options[0]);
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
        for (uint32_t b = 0; b < maniGltf_GetBufferCount(gltf); ++b)
        {
            if (maniGltf_GetBufferUri(gltf, b, NULL) != NULL)
            {
                (void)maniGltf_SupplyBuffer(gltf, b, data, size);
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
