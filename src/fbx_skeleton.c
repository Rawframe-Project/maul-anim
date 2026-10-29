// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skeleton builds. Bones are the marked models in a
// depth-first walk from the root, in connection order. A bone's rest is
// its model's own transform wherever that is exact under the runtime's
// one composition rule: always for normal inheritance, and for the
// others when their parent has a helper bone, whose world is the
// parent's unscaled world. Without one, an RrSs or Rrs model's rest is
// its parent's world inverted times its own, decomposed; a parent gets a
// helper where that does not decompose exactly, or always on request.
// The options' axes and unit apply to every rest and to the placement.

#include "allocator.h"
#include "basis.h"
#include "builder.h"
#include "decompose.h"
#include "fbx_document.h"
#include "fbx_helpers.h"
#include "fbx_model.h"
#include "fbx_options.h"
#include "fbx_transform.h"
#include "fbx_units.h"
#include "fbx_world.h"
#include "fpenv.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <math.h>
#include <string.h>

// A helper bone's name: its parent's, then this.
static const char kHelperSuffix[] = "#unscaled";
#define MANI_HELPER_SUFFIX (sizeof kHelperSuffix - 1)

static maniImportResult Invalid(void)
{
    return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0};
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

static uint64_t ModelOffset(const maniFbx* fbx, uint32_t model)
{
    return fbx->nodes[fbx->objects[fbx->models[model]].node].nameOffset;
}

static maniImportResult Refuse(const maniFbx* fbx, maniImportReason reason, uint32_t model)
{
    return maniFbxFail(reason, ModelOffset(fbx, model), model);
}

static void* Block(const maniFbx* fbx, size_t count, size_t size)
{
    return maniAllocate(&fbx->allocator, count + 1, size, 8);
}

static void FreeBlock(const maniFbx* fbx, void* block, size_t count, size_t size)
{
    maniRelease(&fbx->allocator, block, count + 1, size, 8);
}

typedef struct Build
{
    const maniFbx* fbx;
    const maniFbxOptions* options;
    maniBasis basis;
    uint32_t root;
    // Per model: marked as a bone (or needed for the worlds), whether it
    // has a helper, its bone and its helper's bone.
    uint8_t* marked;
    uint8_t* helper;
    uint32_t* boneOf;
    uint32_t* helperOf;
    maniFbxWorld* worlds;
    // Per model: its compensated rest, where it has one.
    maniTransform* compensated;
    // Bone models in depth-first order.
    uint32_t* order;
    uint32_t count;
} Build;

// The marked models depth-first from the root, children in connection
// order (pushed last to first).
static void Order(Build* b)
{
    const maniFbx* fbx = b->fbx;
    uint32_t height = 0;
    uint32_t* stack = b->boneOf;
    stack[height++] = b->root;
    b->count = 0;
    while (height > 0)
    {
        uint32_t v = stack[--height];
        b->order[b->count++] = v;
        const uint32_t* children = NULL;
        uint32_t n = maniFbxModelChildren(fbx, v, &children);
        for (uint32_t k = n; k-- > 0;)
        {
            if (b->marked[children[k]] != 0)
            {
                stack[height++] = children[k];
            }
        }
    }
}

// Which parents get helpers: those of a non-normal child, on request;
// otherwise those of one whose compensation is not exact, at rest or at
// a sample of the file's animation. The children still exact are listed
// in boneOf, free until the bones are emitted.
static maniImportResult FindHelpers(Build* b)
{
    uint32_t exact = 0;
    for (uint32_t i = 1; i < b->count; ++i)
    {
        uint32_t c = b->order[i];
        uint32_t p = b->fbx->modelParents[c];
        if (b->worlds[c].inherit == maniFbxInheritRSrs)
        {
            continue;
        }
        bool kept = b->options->inheritHelpers == 0 &&
                    maniFbxCompensate(b->worlds, c, p, &b->compensated[c]);
        b->helper[p] = kept ? b->helper[p] : 1;
        if (kept)
        {
            b->boneOf[exact++] = c;
        }
    }
    return maniFbxAnimatedHelpers(b->fbx, b->marked, b->boneOf, exact, b->helper);
}

// The placement: the world the root's own transform is relative to.
static maniFbxMatrix Placement(const maniFbx* fbx, const maniFbxWorld* worlds, uint32_t root)
{
    maniFbxMatrix identity = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    uint32_t p = fbx->modelParents[root];
    if (p == MANI_FBX_NONE)
    {
        return identity;
    }
    return worlds[root].inherit == maniFbxInheritRSrs ? worlds[p].world : worlds[p].unscaled;
}

static maniImportResult RootRest(const Build* b, maniTransform* out)
{
    const maniFbxTrs* own = &b->worlds[b->root].transform;
    if (b->options->foldPlacement == 0)
    {
        *out = maniFbxTrsTransform(own);
        return maniFbxSuccess();
    }
    maniFbxMatrix placement = Placement(b->fbx, b->worlds, b->root);
    maniFbxMatrix local = maniFbxTrsMatrix(own);
    maniFbxMatrix folded = maniFbxMatrixMul(&placement, &local);
    maniAffine affine = maniFbxMatrixAffine(&folded);
    return maniDecomposeExact(&affine, out) ? maniFbxSuccess()
                                            : Refuse(b->fbx, maniImportReasonMatrix, b->root);
}

// A helper's rest: the inverse of its parent's inherited scale.
static maniImportResult HelperRest(const Build* b, uint32_t parent, maniTransform* out)
{
    const double* s = b->worlds[parent].inheritScale;
    double inverse[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        inverse[i] = 1.0 / s[i];
        // Not finite as a double, it is not as a float either.
        if (!isfinite((float)inverse[i]))
        {
            return Refuse(b->fbx, maniImportReasonMatrix, parent);
        }
    }
    *out = (maniTransform){
        {0, 0, 0}, {0, 0, 0, 1}, {(float)inverse[0], (float)inverse[1], (float)inverse[2]}};
    return maniFbxSuccess();
}

// One bone's source: its parent's bone and its rest.
static maniImportResult Bone(const Build* b, uint32_t model, maniBoneSource* bone)
{
    if (model == b->root)
    {
        bone->parent = -1;
        return RootRest(b, &bone->rest);
    }
    uint32_t p = b->fbx->modelParents[model];
    bool normal = b->worlds[model].inherit == maniFbxInheritRSrs;
    bool hangs = !normal && b->helper[p] != 0;
    bone->parent = (int32_t)(hangs ? b->helperOf[p] : b->boneOf[p]);
    bone->rest =
        normal || hangs ? maniFbxTrsTransform(&b->worlds[model].transform) : b->compensated[model];
    return maniFbxSuccess();
}

// Bones in order, each model's helper right after it; names point at the
// models' bone names and at text for the helpers'.
static maniImportResult Sources(Build* b, maniBoneSource* bones, char* text, uint32_t* countOut)
{
    uint32_t n = 0;
    size_t at = 0;
    maniImportResult r = maniFbxSuccess();
    for (uint32_t i = 0; r.result == mani_success && i < b->count; ++i)
    {
        uint32_t v = b->order[i];
        b->boneOf[v] = n;
        bones[n].name = maniFbxModelName(b->fbx, v, &bones[n].nameLength);
        r = Bone(b, v, &bones[n]);
        bones[n].rest = maniBasisTransform(&b->basis, bones[n].rest);
        ++n;
        if (r.result != mani_success || b->helper[v] == 0)
        {
            continue;
        }
        b->helperOf[v] = n;
        size_t length = bones[n - 1].nameLength;
        memcpy(text + at, bones[n - 1].name, length);
        memcpy(text + at + length, kHelperSuffix, MANI_HELPER_SUFFIX);
        bones[n] = (maniBoneSource){text + at,
                                    length + (uint32_t)MANI_HELPER_SUFFIX,
                                    (int32_t)(n - 1),
                                    {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}};
        at += length + MANI_HELPER_SUFFIX;
        r = HelperRest(b, v, &bones[n].rest);
        bones[n].rest = maniBasisTransform(&b->basis, bones[n].rest);
        ++n;
    }
    *countOut = n;
    return r;
}

// The builder's refusal as the import's: a name collision on the bone's
// model (a helper's, its parent's), anything else with the build's
// reason.
static maniImportResult FromBuild(const Build* b, maniBuildResult result, const uint32_t* modelOf)
{
    if (result.result == mani_success || result.result == mani_errorCapacity)
    {
        return (maniImportResult){result.result, maniImportReasonNone, 0, 0};
    }
    bool name =
        result.reason == maniBuildReasonDuplicateName || result.reason == maniBuildReasonNameHash;
    uint32_t model = modelOf[result.item];
    return maniFbxFail(name ? maniImportReasonName : maniImportReasonBuild,
                       ModelOffset(b->fbx, model), name ? model : (uint32_t)result.reason);
}

// Emits the bones and builds them.
static maniImportResult Emit(maniBuilder* builder, Build* b)
{
    const maniFbx* fbx = b->fbx;
    size_t text = 0;
    uint32_t helpers = 0;
    for (uint32_t i = 0; i < b->count; ++i)
    {
        size_t length = 0;
        (void)maniFbxModelName(fbx, b->order[i], &length);
        text += b->helper[b->order[i]] != 0 ? length + MANI_HELPER_SUFFIX : 0;
        helpers += b->helper[b->order[i]];
    }
    size_t n = (size_t)b->count + helpers;
    maniBoneSource* bones = Block(fbx, n, sizeof(maniBoneSource));
    char* names = Block(fbx, text, 1);
    uint32_t* modelOf = Block(fbx, n, sizeof(uint32_t));
    maniImportResult r = Capacity();
    if (bones != NULL && names != NULL && modelOf != NULL)
    {
        uint32_t count = 0;
        r = Sources(b, bones, names, &count);
        for (uint32_t i = 0, k = 0; r.result == mani_success && i < b->count; ++i)
        {
            modelOf[k++] = b->order[i];
            if (b->helper[b->order[i]] != 0)
            {
                modelOf[k++] = b->order[i];
            }
        }
        if (r.result == mani_success)
        {
            maniSkeletonSource source = {bones, count};
            r = FromBuild(b, maniBuildSkeleton(builder, &source), modelOf);
        }
    }
    FreeBlock(fbx, bones, n, sizeof(maniBoneSource));
    FreeBlock(fbx, names, text, 1);
    FreeBlock(fbx, modelOf, n, sizeof(uint32_t));
    return r;
}

// The worlds a build or a placement needs: the marked models and those
// above the root.
static maniImportResult Worlds(Build* b)
{
    for (uint32_t v = b->fbx->modelParents[b->root]; v != MANI_FBX_NONE;
         v = b->fbx->modelParents[v])
    {
        b->marked[v] = 2;
    }
    // The marks above the root stay: bones are walked down from it.
    return maniFbxEvaluateWorlds(b->fbx, b->marked, b->worlds);
}

static maniImportResult Start(Build* b)
{
    size_t n = b->fbx->modelCount;
    const maniFbx* fbx = b->fbx;
    b->helper = Block(fbx, n, 1);
    b->boneOf = Block(fbx, n, sizeof(uint32_t));
    b->helperOf = Block(fbx, n, sizeof(uint32_t));
    b->worlds = Block(fbx, n, sizeof(maniFbxWorld));
    b->compensated = Block(fbx, n, sizeof(maniTransform));
    b->order = Block(fbx, n, sizeof(uint32_t));
    if (b->marked == NULL || b->helper == NULL || b->boneOf == NULL || b->helperOf == NULL ||
        b->worlds == NULL || b->compensated == NULL || b->order == NULL)
    {
        return Capacity();
    }
    memset(b->helper, 0, n + 1);
    return Worlds(b);
}

static void Finish(Build* b)
{
    size_t n = b->fbx->modelCount;
    const maniFbx* fbx = b->fbx;
    FreeBlock(fbx, b->marked, n, 1);
    FreeBlock(fbx, b->helper, n, 1);
    FreeBlock(fbx, b->boneOf, n, sizeof(uint32_t));
    FreeBlock(fbx, b->helperOf, n, sizeof(uint32_t));
    FreeBlock(fbx, b->worlds, n, sizeof(maniFbxWorld));
    FreeBlock(fbx, b->compensated, n, sizeof(maniTransform));
    FreeBlock(fbx, b->order, n, sizeof(uint32_t));
}

// A build of the models marked in b->marked, rooted at b->root.
static maniImportResult BuildMarked(maniBuilder* builder, Build* b)
{
    maniImportResult r = Start(b);
    if (r.result == mani_success)
    {
        Order(b);
        r = FindHelpers(b);
    }
    if (r.result == mani_success)
    {
        r = Emit(builder, b);
    }
    Finish(b);
    return r;
}

static Build NewBuild(const maniFbx* fbx, const maniFbxOptions* options, uint32_t root)
{
    Build b = {fbx,  options, maniFbxBasis(fbx, options), root, NULL, NULL, NULL, NULL, NULL, NULL,
               NULL, 0};
    b.marked = Block(fbx, fbx->modelCount, 1);
    if (b.marked != NULL)
    {
        memset(b.marked, 0, (size_t)fbx->modelCount + 1);
    }
    return b;
}

static MANI_NOINLINE maniImportResult FbxBuildSkeletonWork(maniBuilder* builder, const maniFbx* fbx,
                                                           uint32_t skeleton,
                                                           const maniFbxOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorBusy, maniImportReasonNone, 0, 0};
    }
    bool usable =
        maniFbxUsable(builder, fbx, options, fbx != NULL && skeleton < fbx->skeletonCount);
    // fbx is not NULL when usable; said again for the analyzer.
    if (!usable || fbx == NULL)
    {
        return Invalid();
    }
    Build b = NewBuild(fbx, options, fbx->skeletonRoots[skeleton]);
    if (b.marked != NULL)
    {
        maniFbxMarkSkeleton(fbx, skeleton, b.marked);
    }
    return BuildMarked(builder, &b);
}

maniImportResult maniBuildFbxSkeleton(maniBuilder* builder, const maniFbx* fbx, uint32_t skeleton,
                                      const maniFbxOptions* options)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = FbxBuildSkeletonWork(builder, fbx, skeleton, options);
    maniFloatEnvLeave(env);
    return result;
}

// A model's subtree, marked by a walk of the children lists.
static void MarkSubtree(Build* b)
{
    uint32_t* stack = Block(b->fbx, b->fbx->modelCount, sizeof(uint32_t));
    if (stack == NULL || b->marked == NULL)
    {
        FreeBlock(b->fbx, stack, b->fbx->modelCount, sizeof(uint32_t));
        return;
    }
    uint32_t height = 0;
    stack[height++] = b->root;
    while (height > 0)
    {
        uint32_t v = stack[--height];
        b->marked[v] = 1;
        const uint32_t* children = NULL;
        uint32_t n = maniFbxModelChildren(b->fbx, v, &children);
        for (uint32_t k = 0; k < n; ++k)
        {
            stack[height++] = children[k];
        }
    }
    FreeBlock(b->fbx, stack, b->fbx->modelCount, sizeof(uint32_t));
}

static MANI_NOINLINE maniImportResult FbxBuildModelSkeletonWork(maniBuilder* builder,
                                                                const maniFbx* fbx, uint32_t model,
                                                                const maniFbxOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorBusy, maniImportReasonNone, 0, 0};
    }
    bool usable = maniFbxUsable(builder, fbx, options, fbx != NULL && model < fbx->modelCount);
    // fbx is not NULL when usable; said again for the analyzer.
    if (!usable || fbx == NULL)
    {
        return Invalid();
    }
    Build b = NewBuild(fbx, options, model);
    MarkSubtree(&b);
    if (b.marked != NULL && b.marked[model] == 0)
    {
        FreeBlock(fbx, b.marked, fbx->modelCount, 1);
        return Capacity();
    }
    return BuildMarked(builder, &b);
}

maniImportResult maniBuildFbxModelSkeleton(maniBuilder* builder, const maniFbx* fbx, uint32_t model,
                                           const maniFbxOptions* options)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = FbxBuildModelSkeletonWork(builder, fbx, model, options);
    maniFloatEnvLeave(env);
    return result;
}

// The placement of the skeleton rooted at root.
static maniImportResult PlacementOf(const maniFbx* fbx, uint32_t root,
                                    const maniFbxOptions* options, maniAffine* placementOut)
{
    *placementOut = maniIdentityAffine();
    if (options->foldPlacement != 0)
    {
        return maniFbxSuccess();
    }
    Build b = NewBuild(fbx, options, root);
    maniImportResult r = Capacity();
    if (b.marked != NULL)
    {
        b.marked[root] = 1;
        r = Start(&b);
    }
    if (r.result == mani_success)
    {
        maniFbxMatrix p = Placement(fbx, b.worlds, root);
        *placementOut = maniBasisAffine(&b.basis, maniFbxMatrixAffine(&p));
    }
    Finish(&b);
    return r;
}

static MANI_NOINLINE maniImportResult FbxSkeletonPlacementWork(const maniFbx* fbx,
                                                               uint32_t skeleton,
                                                               const maniFbxOptions* options,
                                                               maniAffine* placementOut)
{
    if (fbx == NULL || options == NULL || placementOut == NULL || !maniFbxOptionsValid(options) ||
        skeleton >= fbx->skeletonCount)
    {
        return Invalid();
    }
    return PlacementOf(fbx, fbx->skeletonRoots[skeleton], options, placementOut);
}

maniImportResult maniFbx_GetSkeletonPlacement(const maniFbx* fbx, uint32_t skeleton,
                                              const maniFbxOptions* options,
                                              maniAffine* placementOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = FbxSkeletonPlacementWork(fbx, skeleton, options, placementOut);
    maniFloatEnvLeave(env);
    return result;
}

static MANI_NOINLINE maniImportResult FbxModelPlacementWork(const maniFbx* fbx, uint32_t model,
                                                            const maniFbxOptions* options,
                                                            maniAffine* placementOut)
{
    if (fbx == NULL || options == NULL || placementOut == NULL || !maniFbxOptionsValid(options) ||
        model >= fbx->modelCount)
    {
        return Invalid();
    }
    return PlacementOf(fbx, model, options, placementOut);
}

maniImportResult maniFbx_GetModelPlacement(const maniFbx* fbx, uint32_t model,
                                           const maniFbxOptions* options, maniAffine* placementOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = FbxModelPlacementWork(fbx, model, options, placementOut);
    maniFloatEnvLeave(env);
    return result;
}
