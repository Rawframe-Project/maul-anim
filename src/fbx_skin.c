// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skins: a skin's clusters in connection order, each
// one's linked model a joint found by bone name, its inverse bind the
// cluster's Transform (the mesh node relative to the bone at bind, as
// ufbx reads it) times the mesh model's geometric transform. Without
// Transform, the bone's bind world is TransformLink, else its matrix in
// a bind pose, else its world at rest, and the mesh node's world at rest
// is taken relative to it.

#include "fbx_skin.h"

#include "allocator.h"
#include "basis.h"
#include "builder.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_options.h"
#include "fbx_transform.h"
#include "fbx_value.h"
#include "fbx_world.h"
#include "fpenv.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <string.h>

static maniImportResult Invalid(void)
{
    return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0};
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

static uint64_t ObjectOffset(const maniFbx* fbx, uint32_t object)
{
    return fbx->nodes[fbx->objects[object].node].nameOffset;
}

static void* Block(const maniFbx* fbx, size_t count, size_t size)
{
    return maniAllocate(&fbx->allocator, count + 1, size, 8);
}

static void FreeBlock(const maniFbx* fbx, void* block, size_t count, size_t size)
{
    maniRelease(&fbx->allocator, block, count + 1, size, 8);
}

static uint32_t MeshModel(const maniFbx* fbx, uint32_t skin)
{
    return maniFbxDeformerModel(fbx, fbx->skins[skin]);
}

uint32_t maniFbxSkinModel(const maniFbx* fbx, uint32_t skin)
{
    if (fbx == NULL || skin >= fbx->skinCount)
    {
        return UINT32_MAX;
    }
    uint32_t model = MeshModel(fbx, skin);
    return model != MANI_FBX_NONE ? model : UINT32_MAX;
}

// A skin's joints: each cluster with a linked model, in connection order.
typedef struct Joints
{
    uint32_t* clusters;
    uint32_t* models;
    uint32_t count;
} Joints;

// The first model a cluster is the parent of through an `OO` connection.
static uint32_t LinkedModel(const maniFbx* fbx, uint32_t cluster)
{
    const uint32_t* list = NULL;
    uint32_t n = maniFbxChildConnections(fbx, cluster, &list);
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniFbxConnection* c = &fbx->connections[list[k]];
        uint32_t model = maniFbxModelOf(fbx, c->child);
        if (c->childProperty == MANI_FBX_NONE && c->parentProperty == MANI_FBX_NONE &&
            model != MANI_FBX_NONE)
        {
            return model;
        }
    }
    return MANI_FBX_NONE;
}

static void VisitCluster(const maniFbx* fbx, uint32_t child, void* context)
{
    Joints* joints = context;
    if (!maniFbxDeformerIs(fbx, child, "Cluster", 7))
    {
        return;
    }
    uint32_t model = LinkedModel(fbx, child);
    if (model == MANI_FBX_NONE)
    {
        return;
    }
    if (joints->clusters != NULL)
    {
        joints->clusters[joints->count] = child;
        joints->models[joints->count] = model;
    }
    ++joints->count;
}

// The clusters linking a model, written where out is given.
typedef struct Collect
{
    uint32_t* out;
    uint32_t count;
} Collect;

static void VisitCollect(const maniFbx* fbx, uint32_t child, void* context)
{
    Collect* c = context;
    if (maniFbxDeformerIs(fbx, child, "Cluster", 7) && LinkedModel(fbx, child) != MANI_FBX_NONE)
    {
        if (c->out != NULL)
        {
            c->out[c->count] = child;
        }
        ++c->count;
    }
}

uint32_t maniFbxSkinClusters(const maniFbx* fbx, uint32_t skin, uint32_t* clusters)
{
    Collect c = {clusters, 0};
    maniFbxEachChild(fbx, fbx->skins[skin], VisitCollect, &c);
    return c.count;
}

// Whether every cluster linking a model links a bone of a skeleton.
typedef struct Within
{
    const maniSkeleton* skeleton;
    uint32_t count;
    bool all;
} Within;

static void VisitWithin(const maniFbx* fbx, uint32_t child, void* context)
{
    Within* w = context;
    uint32_t model =
        maniFbxDeformerIs(fbx, child, "Cluster", 7) ? LinkedModel(fbx, child) : MANI_FBX_NONE;
    if (model == MANI_FBX_NONE)
    {
        return;
    }
    size_t length = 0;
    const char* name = maniFbxModelName(fbx, model, &length);
    w->all = w->all && maniSkeletonFindBone(w->skeleton, name, length) != MANI_NO_BONE;
    ++w->count;
}

bool maniFbxSkinWithin(const maniFbx* fbx, uint32_t skin, const maniSkeleton* skeleton)
{
    Within w = {skeleton, 0, true};
    maniFbxEachChild(fbx, fbx->skins[skin], VisitWithin, &w);
    return w.count > 0 && w.all;
}

// Sixteen numbers, an array's or a node's own, column-major; refused
// (Property, item) when there are fewer or one is no finite number.
static maniImportResult ReadSixteen(const maniFbx* fbx, uint32_t node, uint32_t item, double m[16])
{
    uint32_t first = maniFbxNodeProperty(fbx, node, 0);
    uint64_t at =
        first != MANI_FBX_NONE ? fbx->properties[first].offset : fbx->nodes[node].nameOffset;
    uint32_t count = maniFbxArrayCount(fbx, first);
    if (count == 0)
    {
        for (uint32_t k = 0; k < 16; ++k)
        {
            if (!maniFbxNumber(fbx, maniFbxNodeProperty(fbx, node, k), &m[k]))
            {
                return maniFbxFail(maniImportReasonProperty, at, item);
            }
        }
        return maniFbxSuccess();
    }
    if (count < 16)
    {
        return maniFbxFail(maniImportReasonProperty, at, item);
    }
    double* all = Block(fbx, count, sizeof(double));
    if (all == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniFbxReadDoubles(fbx, first, all);
    r.item = r.result == mani_errorFormat ? item : r.item;
    if (r.result == mani_success)
    {
        memcpy(m, all, 16 * sizeof(double));
    }
    FreeBlock(fbx, all, count, sizeof(double));
    return r;
}

// A matrix child of a node (Transform, TransformLink, Matrix): absent
// (false), or read, all zero when not given; refused when not affine
// (Matrix, item).
static maniImportResult ReadMatrix(const maniFbx* fbx, uint32_t node, const char* name,
                                   uint32_t item, bool* present, maniFbxMatrix* out)
{
    uint32_t child = maniFbxFindChild(fbx, node, name, strlen(name));
    *present = child != MANI_FBX_NONE;
    if (!*present)
    {
        return maniFbxSuccess();
    }
    double m[16];
    maniImportResult r = ReadSixteen(fbx, child, item, m);
    if (r.result != mani_success)
    {
        return r;
    }
    // All zero is a matrix not given (ufbx's reading), not a projective one.
    bool zero = true;
    for (uint32_t k = 0; k < 16; ++k)
    {
        zero = zero && m[k] == 0.0;
    }
    if (!zero && !(fabs(m[3]) <= 1e-5 && fabs(m[7]) <= 1e-5 && fabs(m[11]) <= 1e-5 &&
                   fabs(m[15] - 1.0) <= 1e-5))
    {
        return maniFbxFail(maniImportReasonMatrix, fbx->nodes[child].nameOffset, item);
    }
    for (uint32_t c = 0; c < 4; ++c)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            out->m[3 * c + i] = m[4 * c + i];
        }
    }
    return maniFbxSuccess();
}

static bool AllZero(const maniFbxMatrix* m)
{
    for (uint32_t i = 0; i < 12; ++i)
    {
        if (m->m[i] != 0.0)
        {
            return false;
        }
    }
    return true;
}

typedef struct Skin
{
    const maniFbx* fbx;
    uint32_t skin;
    uint32_t mesh;
    Joints joints;
    // Per model: needed for the worlds, and its bind pose's PoseNode.
    uint8_t* needed;
    uint32_t* poseOf;
    maniFbxWorld* worlds;
    maniAffine* binds;
    uint32_t* bones;
} Skin;

// Each model's first bind pose entry (ufbx keeps the first pose a node
// is in): PoseNode children of BindPose objects, naming a model.
static void ReadPoses(Skin* s)
{
    const maniFbx* fbx = s->fbx;
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        s->poseOf[m] = MANI_FBX_NONE;
    }
    for (uint32_t o = 0; o < fbx->objectCount; ++o)
    {
        if (!maniFbxObjectKindIs(fbx, o, "Pose", 4) ||
            !maniFbxObjectSubclassIs(fbx, o, "BindPose", 8))
        {
            continue;
        }
        for (uint32_t e = fbx->nodes[fbx->objects[o].node].firstChild; e != MANI_FBX_NONE;
             e = fbx->nodes[e].next)
        {
            uint32_t node = maniFbxFindChild(fbx, e, "Node", 4);
            uint32_t object = node != MANI_FBX_NONE
                                  ? maniFbxReferencedObject(fbx, maniFbxNodeProperty(fbx, node, 0))
                                  : MANI_FBX_NONE;
            uint32_t model = maniFbxModelOf(fbx, object);
            if (maniFbxNameIs(fbx, e, "PoseNode", 8) && model != MANI_FBX_NONE &&
                s->poseOf[model] == MANI_FBX_NONE)
            {
                s->poseOf[model] = e;
            }
        }
    }
}

// The worlds the fallbacks need: the mesh's and each joint's, and those
// above them.
static maniImportResult Worlds(Skin* s)
{
    const maniFbx* fbx = s->fbx;
    memset(s->needed, 0, (size_t)fbx->modelCount + 1);
    for (uint32_t j = 0; j <= s->joints.count; ++j)
    {
        uint32_t v = j < s->joints.count ? s->joints.models[j] : s->mesh;
        for (; v != MANI_FBX_NONE && s->needed[v] == 0; v = fbx->modelParents[v])
        {
            s->needed[v] = 1;
        }
    }
    return maniFbxEvaluateWorlds(fbx, s->needed, s->worlds);
}

// A bone's world at bind: TransformLink (read only with Transform, as
// ufbx reads it), its bind pose matrix, or its world at rest.
static maniImportResult BindWorld(const Skin* s, uint32_t cluster, uint32_t model, bool paired,
                                  maniFbxMatrix* out)
{
    const maniFbx* fbx = s->fbx;
    bool present = false;
    maniImportResult r =
        ReadMatrix(fbx, fbx->objects[cluster].node, "TransformLink", s->skin, &present, out);
    bool linked = paired && present && !AllZero(out);
    bool posed = false;
    if (r.result == mani_success && !linked && s->poseOf[model] != MANI_FBX_NONE)
    {
        r = ReadMatrix(fbx, s->poseOf[model], "Matrix", s->skin, &present, out);
        posed = present && !AllZero(out);
    }
    if (r.result == mani_success && !linked && !posed)
    {
        *out = s->worlds[model].world;
    }
    return r;
}

// The mesh node relative to the bone at bind, from the bone's bind world
// and the mesh node's world at rest.
static maniImportResult Relative(const Skin* s, uint32_t j, bool paired, maniFbxMatrix* out)
{
    const maniFbx* fbx = s->fbx;
    uint32_t cluster = s->joints.clusters[j];
    maniFbxMatrix bind;
    maniFbxMatrix inverse;
    maniImportResult r = BindWorld(s, cluster, s->joints.models[j], paired, &bind);
    if (r.result != mani_success)
    {
        return r;
    }
    if (!maniFbxMatrixInverse(&bind, &inverse))
    {
        return maniFbxFail(maniImportReasonMatrix, ObjectOffset(fbx, cluster), s->skin);
    }
    maniFbxMatrix identity = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    const maniFbxMatrix* mesh = s->mesh != MANI_FBX_NONE ? &s->worlds[s->mesh].world : &identity;
    *out = maniFbxMatrixMul(&inverse, mesh);
    return maniFbxSuccess();
}

// A joint's inverse bind in the file's space: the mesh node relative to
// the bone at bind (the cluster's Transform, or Relative), times the
// mesh's geometric transform. Additive clusters are refused.
static maniImportResult Bind(const Skin* s, uint32_t j, const maniFbxMatrix* geometry,
                             maniFbxMatrix* out)
{
    const maniFbx* fbx = s->fbx;
    uint32_t node = fbx->objects[s->joints.clusters[j]].node;
    uint32_t mode = maniFbxFindChild(fbx, node, "Mode", 4);
    if (mode != MANI_FBX_NONE &&
        maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, mode, 0), "Additive", 8))
    {
        return maniFbxFail(maniImportReasonProperty, fbx->nodes[mode].nameOffset, s->skin);
    }
    bool present = false;
    bool linked = false;
    maniFbxMatrix meshToBone = {{0}};
    maniFbxMatrix link = {{0}};
    maniImportResult r = ReadMatrix(fbx, node, "Transform", s->skin, &present, &meshToBone);
    if (r.result == mani_success)
    {
        r = ReadMatrix(fbx, node, "TransformLink", s->skin, &linked, &link);
    }
    if (r.result == mani_success && !(present && linked && !AllZero(&meshToBone)))
    {
        r = Relative(s, j, present && linked, &meshToBone);
    }
    if (r.result == mani_success)
    {
        *out = maniFbxMatrixMul(&meshToBone, geometry);
    }
    return r;
}

// The joints' bones and inverse binds, the options' basis applied.
static maniImportResult ReadJoints(Skin* s, const maniSkeleton* skeleton, const maniBasis* basis)
{
    const maniFbx* fbx = s->fbx;
    maniFbxTrs geometry = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    maniImportResult r = s->mesh != MANI_FBX_NONE
                             ? maniFbxGeometricTransform(fbx, fbx->models[s->mesh], &geometry)
                             : maniFbxSuccess();
    maniFbxMatrix g = maniFbxTrsMatrix(&geometry);
    for (uint32_t j = 0; r.result == mani_success && j < s->joints.count; ++j)
    {
        size_t length = 0;
        const char* name = maniFbxModelName(fbx, s->joints.models[j], &length);
        s->bones[j] = maniSkeletonFindBone(skeleton, name, length);
        if (s->bones[j] == MANI_NO_BONE)
        {
            return maniFbxFail(maniImportReasonBone, ObjectOffset(fbx, s->joints.clusters[j]),
                               s->skin);
        }
        maniFbxMatrix bind;
        r = Bind(s, j, &g, &bind);
        s->binds[j] = maniBasisAffine(basis, maniFbxMatrixAffine(&bind));
    }
    return r;
}

static maniImportResult Build(maniBuilder* builder, Skin* s, const maniSkeleton* skeleton,
                              const maniFbxOptions* options)
{
    const maniFbx* fbx = s->fbx;
    maniFbxEachChild(fbx, fbx->skins[s->skin], VisitCluster, &s->joints);
    size_t n = s->joints.count;
    size_t models = fbx->modelCount;
    s->joints.clusters = Block(fbx, n, sizeof(uint32_t));
    s->joints.models = Block(fbx, n, sizeof(uint32_t));
    s->bones = Block(fbx, n, sizeof(uint32_t));
    s->binds = Block(fbx, n, sizeof(maniAffine));
    s->needed = Block(fbx, models, 1);
    s->poseOf = Block(fbx, models, sizeof(uint32_t));
    s->worlds = Block(fbx, models, sizeof(maniFbxWorld));
    if (s->joints.clusters == NULL || s->joints.models == NULL || s->bones == NULL ||
        s->binds == NULL || s->needed == NULL || s->poseOf == NULL || s->worlds == NULL)
    {
        return Capacity();
    }
    s->joints.count = 0;
    maniFbxEachChild(fbx, fbx->skins[s->skin], VisitCluster, &s->joints);
    ReadPoses(s);
    maniImportResult r = Worlds(s);
    maniBasis basis = maniFbxBasis(fbx, options);
    r = r.result == mani_success ? ReadJoints(s, skeleton, &basis) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    maniSkinSource source = {skeleton, s->bones, s->binds, s->joints.count};
    maniBuildResult b = maniBuildSkin(builder, &source);
    if (b.result == mani_success || b.result == mani_errorCapacity)
    {
        return (maniImportResult){b.result, maniImportReasonNone, 0, 0};
    }
    return maniFbxFail(maniImportReasonBuild, ObjectOffset(fbx, fbx->skins[s->skin]),
                       (uint32_t)b.reason);
}

static MANI_NOINLINE maniImportResult FbxBuildSkinWork(maniBuilder* builder, const maniFbx* fbx,
                                                       uint32_t skin, const maniSkeleton* skeleton,
                                                       const maniFbxOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorBusy, maniImportReasonNone, 0, 0};
    }
    bool usable = maniFbxUsable(builder, fbx, options,
                                fbx != NULL && skin < fbx->skinCount && skeleton != NULL);
    // fbx is not NULL when usable; said again for the analyzer.
    if (!usable || fbx == NULL)
    {
        return Invalid();
    }
    Skin s = {fbx, skin, MeshModel(fbx, skin), {NULL, NULL, 0}, NULL, NULL, NULL, NULL, NULL};
    maniImportResult r = Build(builder, &s, skeleton, options);
    size_t n = s.joints.count;
    size_t models = fbx->modelCount;
    FreeBlock(fbx, s.joints.clusters, n, sizeof(uint32_t));
    FreeBlock(fbx, s.joints.models, n, sizeof(uint32_t));
    FreeBlock(fbx, s.bones, n, sizeof(uint32_t));
    FreeBlock(fbx, s.binds, n, sizeof(maniAffine));
    FreeBlock(fbx, s.needed, models, 1);
    FreeBlock(fbx, s.poseOf, models, sizeof(uint32_t));
    FreeBlock(fbx, s.worlds, models, sizeof(maniFbxWorld));
    return r;
}

maniImportResult maniFbxBuildSkin(maniBuilder* builder, const maniFbx* fbx, uint32_t skin,
                                  const maniSkeleton* skeleton, const maniFbxOptions* options)
{
    uint64_t env = maniFloatEnvEnter();
    maniImportResult result = FbxBuildSkinWork(builder, fbx, skin, skeleton, options);
    maniFloatEnvLeave(env);
    return result;
}
