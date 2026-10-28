// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening an FBX document from any bytes, then reading
// every property of every node: numbers, integers, strings (decoded and
// compared), and arrays up to a million elements; binary or ASCII; and
// every object's name, a property, its connections and its local
// transform; every model's children and bone name, the worlds, every
// skeleton's models, and builds: skeletons and model subtrees, their
// skins and clips. Run as fuzz_fbx <corpus dir>; the seeds come from
// test_fbx --seed <file> and the fixtures of test/data/fbx.

#include "fbx_anim.h"
#include "fbx_blend.h"
#include "fbx_curve.h"
#include "fbx_document.h"
#include "fbx_keys.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_take.h"
#include "fbx_transform.h"
#include "fbx_units.h"
#include "fbx_value.h"
#include "fbx_world.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/morph.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

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

// Evaluates a curve inside, on and past its keys, and releases it.
static void Evaluate(const maniFbx* fbx, maniFbxCurve* curve)
{
    for (uint32_t k = 0; k < curve->count && k < 8; ++k)
    {
        double t = curve->keys[k].time;
        (void)maniFbxEvaluateCurve(curve, t, 0.0);
        (void)maniFbxEvaluateCurve(curve, t + 0.37, 0.0);
        (void)maniFbxEvaluateCurve(curve, t - 2.71, 0.0);
        (void)maniFbxEvaluateCurve(curve, t + 1e9, 0.0);
    }
    maniFbxReleaseCurve(fbx, curve);
}

// Reads a curve object and evaluates it.
static void Curve(const maniFbx* fbx, uint32_t object)
{
    maniFbxCurve curve;
    if (maniFbxNameIs(fbx, fbx->objects[object].node, "AnimationCurve", 14) &&
        maniFbxReadCurve(fbx, object, &curve).result == mani_success)
    {
        Evaluate(fbx, &curve);
    }
}

// Reads every node with a Key child as 6.x keys and evaluates them.
static void Keys(const maniFbx* fbx)
{
    for (uint32_t n = 0; n < fbx->nodeCount; ++n)
    {
        maniFbxCurve curve;
        if (maniFbxFindChild(fbx, n, "Key", 3) != MANI_FBX_NONE &&
            maniFbxReadKeys(fbx, n, 0, &curve).result == mani_success)
        {
            Evaluate(fbx, &curve);
        }
    }
}

// A blend shape channel's model, shapes and weights at a few points.
static void Blend(const maniFbx* fbx, uint32_t object)
{
    if (!maniFbxIsBlendChannel(fbx, object))
    {
        return;
    }
    (void)maniFbxBlendChannelModel(fbx, object);
    (void)maniFbxBlendRestPercent(fbx, object);
    uint32_t count = 0;
    (void)maniFbxReadBlendShapes(fbx, object, NULL, &count);
    if (count == 0 || count > 1024)
    {
        return;
    }
    maniFbxBlendShape* shapes = malloc(count * sizeof *shapes);
    double* effective = malloc(count * sizeof *effective);
    if (shapes != NULL && effective != NULL &&
        maniFbxReadBlendShapes(fbx, object, shapes, &count).result == mani_success)
    {
        static const double kWeights[5] = {-1.5, -0.25, 0.0, 0.4, 2.0};
        for (uint32_t k = 0; k < 5; ++k)
        {
            maniFbxBlendWeights(shapes, count, kWeights[k], effective);
        }
    }
    free(shapes);
    free(effective);
}

// Reads each stack and evaluates what it drives at its ends and between.
static void Stacks(const maniFbx* fbx)
{
    static const char* const kNames[4] = {"Lcl Translation", "Lcl Rotation", "Lcl Scaling",
                                          "Visibility"};
    for (uint32_t k = 0; k < maniFbxStackCount(fbx) && k < 4; ++k)
    {
        maniFbxAnimation a;
        if (maniFbxReadStack(fbx, k, &a).result != mani_success)
        {
            continue;
        }
        double tps = (double)fbx->settings.ticksPerSecond;
        double times[3] = {(double)a.start / tps, (double)a.stop / tps,
                           ((double)a.start + (double)a.stop) / (2.0 * tps)};
        for (uint32_t c = 0; c < a.channelCount && c < 16; ++c)
        {
            for (uint32_t n = 0; n < 4; ++n)
            {
                double v[3] = {1.0, 1.0, 1.0};
                (void)maniFbxAnimateProperty(&a, fbx, a.channels[c].target, kNames[n],
                                             strlen(kNames[n]), times[c % 3], c % 6, v);
            }
        }
        maniFbxReleaseAnimation(fbx, &a);
    }
}

// Builds the clips of short stacks (ten seconds at most, so the scene's
// rate keeps the samples few) for an opened skeleton.
static void Clips(maniBuilder* builder, const maniFbx* fbx, const maniSkeleton* skeleton,
                  const maniFbxOptions* options)
{
    for (uint32_t k = 0; k < maniFbxClipCount(fbx) && k < 3; ++k)
    {
        maniFbxAnimation a;
        if (maniFbxReadStack(fbx, k, &a).result != mani_success)
        {
            continue;
        }
        double seconds = ((double)a.stop - (double)a.start) / (double)fbx->settings.ticksPerSecond;
        maniFbxReleaseAnimation(fbx, &a);
        if (seconds <= 10.0)
        {
            // The second with the root's whole motion taken out.
            uint32_t bound = 0;
            maniRootMotionSource root = {0, 7, maniRootRotationWhole, 0, 1, 0, 1};
            maniImportClipOptions settings = maniDefaultImportClipOptions();
            settings.sampleRate = k == 1 ? 7.0f : 0.0f;
            settings.rootMotion = k == 1 ? &root : NULL;
            (void)maniFbxBuildClip(builder, fbx, k, skeleton, options, &settings, &bound);
        }
    }
}

// Builds the skins (for a skeleton of the file, not a model subtree) and
// the clips of a skeleton the builder holds, against it opened from a
// copy aligned by hand.
static void SkinsAndClips(maniBuilder* builder, const maniFbx* fbx, uint32_t skeleton, bool subtree,
                          const maniFbxOptions* options)
{
    size_t size = 0;
    (void)maniCopyBuilt(builder, NULL, 0, &size);
    uint8_t* block = malloc(size + 16);
    uint8_t* file = block != NULL ? block + (16 - (uintptr_t)block % 16) % 16 : NULL;
    maniSkeleton opened;
    if (file != NULL && maniCopyBuilt(builder, file, size, &size) == mani_success &&
        maniOpenSkeleton(file, size, &opened).result == mani_success)
    {
        for (uint32_t s = 0; !subtree && s < maniFbxSkinCount(fbx); ++s)
        {
            static _Alignas(16) uint8_t skinFile[1 << 16];
            size_t skinSize = 0;
            maniSkin skin;
            (void)maniFbxSkinMethod(fbx, s);
            if (maniFbxSkinSkeleton(fbx, s) == skeleton &&
                maniFbxBuildSkin(builder, fbx, s, &opened, options).result == mani_success &&
                maniCopyBuilt(builder, skinFile, sizeof skinFile, &skinSize) == mani_success &&
                maniOpenSkin(skinFile, skinSize, &skin).result == mani_success)
            {
                (void)maniFbxBuildInfluences(builder, fbx, s, &skin);
            }
        }
        Clips(builder, fbx, &opened, options);
    }
    free(block);
}

// The first sets of morph targets: their rest weights, built, opened and
// applied at those weights over their first vertices.
static void Morphs(maniBuilder* builder, const maniFbx* fbx, const maniFbxOptions* options)
{
    static _Alignas(16) uint8_t file[1 << 16];
    static float weights[256];
    static float positions[3 * 64];
    for (uint32_t k = 0; k < maniFbxMorphCount(fbx) && k < 4; ++k)
    {
        (void)maniFbxMorphModel(fbx, k);
        uint32_t count = maniFbxMorphTargetCount(fbx, k);
        size_t size = 0;
        maniMorphTargets m;
        if (count <= 256 && maniFbxMorphWeights(fbx, k, weights, count).result == mani_success &&
            maniFbxBuildMorphTargets(builder, fbx, k, options).result == mani_success &&
            maniCopyBuilt(builder, file, sizeof file, &size) == mani_success &&
            maniOpenMorphTargets(file, size, &m).result == mani_success)
        {
            uint32_t vertices = maniMorphVertexCount(&m);
            maniMorphStream stream = {positions, positions, 12, 12, maniMorphPosition, 0, 3};
            maniMorphJob job = {&m, weights, 0, vertices < 64 ? vertices : 64, 1, &stream};
            memset(positions, 0, sizeof positions);
            (void)maniApplyMorphTargets(&job);
        }
    }
}

// Builds the first skeletons and model subtrees, with the default options
// and with axes and units converted, helpers everywhere and the
// placement folded.
static void Build(maniBuilder* builder, const maniFbx* fbx)
{
    maniFbxOptions options[2] = {maniDefaultFbxOptions(), maniDefaultFbxOptions()};
    options[1].convertAxes = 1;
    options[1].axes[1] = maniAxisNegativeZ;
    options[1].axes[2] = maniAxisY;
    options[1].unitMeters = 1.0;
    options[1].inheritHelpers = 1;
    options[1].foldPlacement = 1;
    for (uint32_t o = 0; o < 2; ++o)
    {
        Morphs(builder, fbx, &options[o]);
        maniAffine placement;
        for (uint32_t k = 0; k < maniFbxSkeletonCount(fbx) && k < 4; ++k)
        {
            if (maniFbxBuildSkeleton(builder, fbx, k, &options[o]).result == mani_success)
            {
                SkinsAndClips(builder, fbx, k, false, &options[o]);
            }
            (void)maniFbxSkeletonPlacement(fbx, k, &options[o], &placement);
        }
        for (uint32_t m = 0; m < maniFbxModelCount(fbx) && m < 4; ++m)
        {
            if (maniFbxBuildModelSkeleton(builder, fbx, m, &options[o]).result == mani_success)
            {
                SkinsAndClips(builder, fbx, m, true, &options[o]);
            }
            (void)maniFbxModelPlacement(fbx, m, &options[o], &placement);
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
        Keys(fbx);
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
            (void)maniFbxModelLocal(fbx, o, NULL, &local);
            Curve(fbx, o);
            Blend(fbx, o);
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
            (void)maniFbxEvaluateWorlds(fbx, NULL, worlds);
            free(worlds);
        }
        uint8_t* marked = malloc((size_t)fbx->modelCount + 1);
        for (uint32_t k = 0; marked != NULL && k < maniFbxSkeletonCount(fbx); ++k)
        {
            memset(marked, 0, (size_t)fbx->modelCount + 1);
            maniFbxMarkSkeleton(fbx, k, marked);
        }
        free(marked);
        Stacks(fbx);
        Build(builder, fbx);
        for (uint32_t v = 0; v < fbx->nodeCount; ++v)
        {
            (void)maniFbxFindChild(fbx, v, "P", 1);
        }
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
    return 0;
}
