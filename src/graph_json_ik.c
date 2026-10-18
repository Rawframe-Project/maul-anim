// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_json_ik.h"

#include "arena.h"
#include "graph_json_read.h"
#include "graph_json_write.h"
#include "ik_chain.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/ik.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

#define ABSENT MANI_JSON_NONE
#define PI_F   3.14159265358979f

// A bone of the reader's skeleton by name.
static bool Bone(maniGraphReader* r, uint32_t value, uint32_t* out)
{
    const char* name = NULL;
    uint32_t length = 0;
    if (!maniGraphJsonName(r, value, &name, &length))
    {
        return false;
    }
    *out = maniSkeletonFindBone(r->skeleton, name, length);
    return *out != MANI_NO_BONE || maniGraphJsonFault(r, value, maniGraphFormName);
}

// [x, y, z], or v as it is when absent.
static bool Vector(maniGraphReader* r, uint32_t value, maniVec3* v)
{
    float f[3];
    if (value == ABSENT)
    {
        return true;
    }
    if (!maniGraphJsonFloats(r, value, f, 3))
    {
        return false;
    }
    *v = (maniVec3){f[0], f[1], f[2]};
    return true;
}

// The pole parameter, MANI_NO_PARAMETER when absent; then twist, a
// number and the weight, as given when absent.
static bool Optional(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* f,
                     uint32_t* pole, maniGraphScalar* scalars)
{
    *pole = MANI_NO_PARAMETER;
    if (f[0] != ABSENT && !maniGraphJsonRefer(r, &scope->parameters, f[0], pole))
    {
        return false;
    }
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (f[1 + k] != ABSENT && !maniGraphJsonScalar(r, scope, f[1 + k], &scalars[k]))
        {
            return false;
        }
    }
    return true;
}

bool maniGraphJsonTwoBone(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                          maniGraphTwoBoneNodeSource* s)
{
    const uint32_t* f = members;
    maniGraphScalar scalars[3] = {
        {MANI_NO_PARAMETER, 0.0f}, {MANI_NO_PARAMETER, 1.0f}, {MANI_NO_PARAMETER, 1.0f}};
    const uint32_t optional[4] = {f[6], f[8], f[9], f[10]};
    s->hinge = (maniVec3){0.0f, 0.0f, 0.0f};
    bool ok = Bone(r, f[2], &s->start) && Bone(r, f[3], &s->middle) && Bone(r, f[4], &s->end) &&
              maniGraphJsonRefer(r, &scope->parameters, f[5], &s->goal) &&
              Vector(r, f[7], &s->hinge) && Optional(r, scope, optional, &s->pole, scalars);
    s->twist = scalars[0];
    s->soften = scalars[1];
    s->weight = scalars[2];
    return ok;
}

// count bone names, 1 to MANI_AIM_MAX_BONES.
static bool Bones(maniGraphReader* r, uint32_t value, maniGraphAimNodeSource* s)
{
    if (!maniGraphJsonArray(r, value, &s->count))
    {
        return false;
    }
    if (s->count == 0 || s->count > MANI_AIM_MAX_BONES)
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    uint32_t* bones = maniArenaTake(&r->arena, s->count, sizeof(uint32_t));
    if (bones == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < s->count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!Bone(r, e, &bones[k]))
        {
            return false;
        }
    }
    s->bones = bones;
    return true;
}

// The shares, a number per bone; NULL when absent.
static bool Shares(maniGraphReader* r, uint32_t value, maniGraphAimNodeSource* s)
{
    s->shares = NULL;
    if (value == ABSENT)
    {
        return true;
    }
    float* shares = maniArenaTake(&r->arena, s->count, sizeof(float));
    if (shares == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    s->shares = shares;
    return maniGraphJsonFloats(r, value, shares, s->count);
}

bool maniGraphJsonAim(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                      maniGraphAimNodeSource* s)
{
    const uint32_t* f = members;
    maniGraphScalar scalars[3] = {
        {MANI_NO_PARAMETER, 0.0f}, {MANI_NO_PARAMETER, 0.0f}, {MANI_NO_PARAMETER, 1.0f}};
    const uint32_t optional[4] = {f[6], f[9], f[10], f[11]};
    s->offset = (maniVec3){0.0f, 0.0f, 0.0f};
    s->up = s->offset;
    bool ok = Bones(r, f[2], s) && maniGraphJsonRefer(r, &scope->parameters, f[3], &s->goal) &&
              Vector(r, f[4], &s->forward) && Shares(r, f[5], s) && Vector(r, f[7], &s->offset) &&
              Vector(r, f[8], &s->up) && Optional(r, scope, optional, &s->pole, scalars);
    s->twist = scalars[0];
    s->cone = scalars[1];
    s->weight = scalars[2];
    return ok;
}

static void PutBone(maniGraphWriter* w, uint32_t bone)
{
    uint32_t length = 0;
    const char* name = maniSkeletonBoneName(w->skeleton, bone, &length);
    maniGraphJsonPutString(w, name, length);
}

static void PutVector(maniGraphWriter* w, const char* key, maniVec3 v)
{
    const float f[3] = {v.x, v.y, v.z};
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    for (uint32_t k = 0; k < 3; ++k)
    {
        maniGraphJsonPutText(w, k == 0 ? "[" : ", ");
        maniGraphJsonPutFloat(w, f[k]);
    }
    maniGraphJsonPutText(w, "]");
}

// The goal, the pole when it has one.
static void PutGoal(maniGraphWriter* w, const maniGraphSource* source, uint32_t goal, uint32_t pole)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "goal");
    maniGraphJsonPutParameterName(w, source, goal);
    if (pole != MANI_NO_PARAMETER)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "pole");
        maniGraphJsonPutParameterName(w, source, pole);
    }
}

static void PutScalars(maniGraphWriter* w, const maniGraphSource* source, const char* const* keys,
                       const maniGraphScalar* scalars)
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, keys[k]);
        maniGraphJsonPutScalar(w, source, scalars[k]);
    }
}

static void PutTwoBone(maniGraphWriter* w, const maniGraphSource* source,
                       const maniGraphTwoBoneNodeSource* s)
{
    static const char* const kBones[3] = {"start", "middle", "end"};
    static const char* const kKeys[3] = {"twist", "soften", "weight"};
    const uint32_t bones[3] = {s->start, s->middle, s->end};
    const maniGraphScalar scalars[3] = {s->twist, s->soften, s->weight};
    for (uint32_t k = 0; k < 3; ++k)
    {
        maniGraphJsonPutText(w, k > 0 ? ", " : "");
        maniGraphJsonPutKey(w, kBones[k]);
        PutBone(w, bones[k]);
    }
    PutGoal(w, source, s->goal, s->pole);
    PutVector(w, "hinge", s->hinge);
    PutScalars(w, source, kKeys, scalars);
}

static void PutAim(maniGraphWriter* w, const maniGraphSource* source,
                   const maniGraphAimNodeSource* s)
{
    static const char* const kKeys[3] = {"twist", "cone", "weight"};
    const maniGraphScalar scalars[3] = {s->twist, s->cone, s->weight};
    maniGraphJsonPutKey(w, "bones");
    for (uint32_t k = 0; k < s->count; ++k)
    {
        maniGraphJsonPutText(w, k == 0 ? "[" : ", ");
        PutBone(w, s->bones[k]);
    }
    maniGraphJsonPutText(w, "]");
    if (s->shares != NULL)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "shares");
        for (uint32_t k = 0; k < s->count; ++k)
        {
            maniGraphJsonPutText(w, k == 0 ? "[" : ", ");
            maniGraphJsonPutFloat(w, s->shares[k]);
        }
        maniGraphJsonPutText(w, "]");
    }
    PutGoal(w, source, s->goal, s->pole);
    PutVector(w, "forward", s->forward);
    PutVector(w, "offset", s->offset);
    PutVector(w, "up", s->up);
    PutScalars(w, source, kKeys, scalars);
}

void maniGraphJsonPutIk(maniGraphWriter* w, const maniGraphSource* source,
                        const maniGraphNodeSource* n)
{
    if (n->kind == maniGraphNodeAim)
    {
        PutAim(w, source, &n->aim);
        return;
    }
    PutTwoBone(w, source, &n->twoBone);
}

// A limit: {"kind": "none"}, {"kind": "cone", "cone": half angle,
// "twist": [low, high]} (the twist free when left out) or {"kind":
// "hinge", "axis": [x, y, z], "range": [low, high]}.
static bool Limit(maniGraphReader* r, uint32_t object, maniJointLimit* l)
{
    static const char* const kKeys[] = {"kind", "cone", "twist", "axis", "range"};
    static const char* const kKinds[] = {"none", "cone", "hinge"};
    // The members each kind takes, by bit.
    static const uint32_t kTakes[3] = {1u, 7u, 25u};
    uint32_t f[5];
    if (!maniGraphJsonMembers(r, object, kKeys, 5, 1u, f))
    {
        return false;
    }
    uint32_t kind = maniGraphJsonWord(r, f[0], kKinds, 3);
    if (kind == 3)
    {
        return maniGraphJsonFault(r, f[0], maniGraphFormValue);
    }
    for (uint32_t k = 1; k < 5; ++k)
    {
        bool takes = (kTakes[kind] >> k & 1u) != 0;
        bool needed = takes && k != 2;
        if (f[k] != ABSENT && !takes)
        {
            return maniGraphJsonFault(r, f[k] - 1, maniGraphFormMember);
        }
        if (f[k] == ABSENT && needed)
        {
            return maniGraphJsonFault(r, object, maniGraphFormMissing);
        }
    }
    float twist[2] = {-PI_F, PI_F};
    float range[2] = {0.0f, 0.0f};
    *l = (maniJointLimit){.kind = kind};
    bool ok = (f[1] == ABSENT || maniGraphJsonFloat(r, f[1], &l->cone)) &&
              (f[2] == ABSENT || maniGraphJsonFloats(r, f[2], twist, 2)) &&
              Vector(r, f[3], &l->axis) &&
              (f[4] == ABSENT || maniGraphJsonFloats(r, f[4], range, 2));
    l->twistLow = kind == maniJointLimitCone ? twist[0] : 0.0f;
    l->twistHigh = kind == maniJointLimitCone ? twist[1] : 0.0f;
    l->low = range[0];
    l->high = range[1];
    return ok;
}

// The limits, one per turning bone of the chain from root to end.
static bool Limits(maniGraphReader* r, uint32_t value, const maniGraphChainNodeSource* s,
                   const maniJointLimit** out)
{
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    uint32_t count = 0;
    if (!maniGraphJsonArray(r, value, &count))
    {
        return false;
    }
    uint32_t n = maniSkeletonBoneCount(r->skeleton);
    uint32_t chain =
        s->root < n && s->end < n ? maniIkChainPath(r->skeleton, s->root, s->end, bones) : 0u;
    if (chain == 0 || count != chain - 1)
    {
        return maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    maniJointLimit* limits = maniArenaTake(&r->arena, count, sizeof(maniJointLimit));
    if (limits == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!Limit(r, e, &limits[k]))
        {
            return false;
        }
    }
    *out = limits;
    return true;
}

bool maniGraphJsonChain(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                        bool fabrik, maniGraphChainNodeSource* s)
{
    const uint32_t* f = members;
    *s = (maniGraphChainNodeSource){.pole = MANI_NO_PARAMETER,
                                    .rotation = MANI_NO_PARAMETER,
                                    .rotationWeight = {MANI_NO_PARAMETER, 1.0f},
                                    .weight = {MANI_NO_PARAMETER, 1.0f}};
    bool ok = Bone(r, f[2], &s->root) && Bone(r, f[3], &s->end) &&
              maniGraphJsonRefer(r, &scope->parameters, f[4], &s->goal) &&
              maniGraphJsonIndex(r, f[5], &s->iterations) &&
              (f[6] == ABSENT || (fabrik ? maniGraphJsonRefer(r, &scope->parameters, f[6], &s->pole)
                                         : maniGraphJsonFloat(r, f[6], &s->step))) &&
              (f[7] == ABSENT || maniGraphJsonRefer(r, &scope->parameters, f[7], &s->rotation)) &&
              (f[8] == ABSENT || maniGraphJsonFloat(r, f[8], &s->tolerance)) &&
              (f[9] == ABSENT || maniGraphJsonScalar(r, scope, f[9], &s->rotationWeight)) &&
              (f[10] == ABSENT || maniGraphJsonScalar(r, scope, f[10], &s->weight));
    return ok && (f[11] == ABSENT || Limits(r, f[11], s, &s->limits));
}

static void PutPair(maniGraphWriter* w, const char* key, float a, float b)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutText(w, "[");
    maniGraphJsonPutFloat(w, a);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutFloat(w, b);
    maniGraphJsonPutText(w, "]");
}

static void PutLimit(maniGraphWriter* w, const maniJointLimit* l)
{
    static const char* const kKinds[3] = {"\"none\"", "\"cone\"", "\"hinge\""};
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "kind");
    maniGraphJsonPutText(w, kKinds[l->kind]);
    if (l->kind == maniJointLimitCone)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "cone");
        maniGraphJsonPutFloat(w, l->cone);
        PutPair(w, "twist", l->twistLow, l->twistHigh);
    }
    else if (l->kind == maniJointLimitHinge)
    {
        PutVector(w, "axis", l->axis);
        PutPair(w, "range", l->low, l->high);
    }
    maniGraphJsonPutText(w, "}");
}

void maniGraphJsonPutChain(maniGraphWriter* w, const maniGraphSource* source,
                           const maniGraphNodeSource* n)
{
    const maniGraphChainNodeSource* s = &n->chain;
    maniGraphJsonPutKey(w, "root");
    PutBone(w, s->root);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "end");
    PutBone(w, s->end);
    PutGoal(w, source, s->goal, s->pole);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "iterations");
    maniGraphJsonPutUnsigned(w, s->iterations);
    if (n->kind == maniGraphNodeCcd)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "step");
        maniGraphJsonPutFloat(w, s->step);
    }
    if (s->rotation != MANI_NO_PARAMETER)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "rotation");
        maniGraphJsonPutParameterName(w, source, s->rotation);
    }
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "tolerance");
    maniGraphJsonPutFloat(w, s->tolerance);
    static const char* const kKeys[2] = {"rotationWeight", "weight"};
    const maniGraphScalar weights[2] = {s->rotationWeight, s->weight};
    for (uint32_t k = 0; k < 2; ++k)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, kKeys[k]);
        maniGraphJsonPutScalar(w, source, weights[k]);
    }
    if (s->limits == NULL)
    {
        return;
    }
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    uint32_t count = maniIkChainPath(w->skeleton, s->root, s->end, bones);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "limits");
    for (uint32_t k = 0; k + 1 < count; ++k)
    {
        maniGraphJsonPutText(w, k == 0 ? "[" : ", ");
        PutLimit(w, &s->limits[k]);
    }
    maniGraphJsonPutText(w, "]");
}
