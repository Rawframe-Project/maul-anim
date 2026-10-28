// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_json_body.h"

#include "arena.h"
#include "graph_json_ik.h"
#include "graph_json_read.h"
#include "graph_json_write.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/ik.h"

#include <stdbool.h>
#include <stdint.h>

#define ABSENT MANI_JSON_NONE

// The element count of a list of least to most objects.
static bool Count(maniGraphReader* r, uint32_t value, uint32_t least, uint32_t most,
                  uint32_t* count)
{
    if (!maniGraphJsonArray(r, value, count))
    {
        return false;
    }
    return (*count >= least && *count <= most) || maniGraphJsonFault(r, value, maniGraphFormValue);
}

static bool Effector(maniGraphReader* r, const maniGraphScope* scope, uint32_t object,
                     maniGraphEffectorSource* e)
{
    static const char* const kKeys[] = {"bone",           "position", "rotation", "positionWeight",
                                        "rotationWeight", "strength", "pullChain"};
    uint32_t f[7];
    if (!maniGraphJsonMembers(r, object, kKeys, 7, 3u, f))
    {
        return false;
    }
    if (f[4] != ABSENT && f[2] == ABSENT)
    {
        return maniGraphJsonFault(r, f[4] - 1, maniGraphFormMember);
    }
    *e = (maniGraphEffectorSource){
        .rotation = MANI_NO_PARAMETER,
        .positionWeight = {MANI_NO_PARAMETER, 1.0f},
        .rotationWeight = {MANI_NO_PARAMETER, f[2] != ABSENT ? 1.0f : 0.0f},
        .strength = 1.0f};
    const maniNameTable* parameters = &scope->parameters;
    return maniGraphJsonBone(r, f[0], &e->bone) &&
           maniGraphJsonRefer(r, parameters, f[1], &e->position) &&
           (f[2] == ABSENT || maniGraphJsonRefer(r, parameters, f[2], &e->rotation)) &&
           (f[3] == ABSENT || maniGraphJsonScalar(r, scope, f[3], &e->positionWeight)) &&
           (f[4] == ABSENT || maniGraphJsonScalar(r, scope, f[4], &e->rotationWeight)) &&
           (f[5] == ABSENT || maniGraphJsonFloat(r, f[5], &e->strength)) &&
           (f[6] == ABSENT || maniGraphJsonFloat(r, f[6], &e->pullChain));
}

static bool Setting(maniGraphReader* r, uint32_t object, maniBodyBone* b)
{
    static const char* const kKeys[] = {"bone", "limit", "preferred", "rotationStiffness",
                                        "positionStiffness"};
    uint32_t f[5];
    if (!maniGraphJsonMembers(r, object, kKeys, 5, 1u, f))
    {
        return false;
    }
    *b = (maniBodyBone){.limit = {.kind = maniJointLimitNone}};
    return maniGraphJsonBone(r, f[0], &b->bone) &&
           (f[1] == ABSENT || maniGraphJsonLimit(r, f[1], &b->limit)) &&
           maniGraphJsonVector(r, f[2], &b->preferred) &&
           (f[3] == ABSENT || maniGraphJsonFloat(r, f[3], &b->rotationStiffness)) &&
           (f[4] == ABSENT || maniGraphJsonFloat(r, f[4], &b->positionStiffness));
}

static bool Effectors(maniGraphReader* r, const maniGraphScope* scope, uint32_t value,
                      maniGraphBodyNodeSource* s)
{
    uint32_t count = 0;
    if (!Count(r, value, 1, MANI_MAX_BODY_IK_EFFECTORS, &count))
    {
        return false;
    }
    maniGraphEffectorSource* effectors =
        maniArenaTake(&r->arena, count, sizeof(maniGraphEffectorSource));
    if (effectors == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!Effector(r, scope, e, &effectors[k]))
        {
            return false;
        }
    }
    s->effectors = effectors;
    s->effectorCount = count;
    return true;
}

static bool Settings(maniGraphReader* r, uint32_t value, maniGraphBodyNodeSource* s)
{
    uint32_t count = 0;
    if (!Count(r, value, 0, MANI_MAX_BODY_IK_BONES, &count))
    {
        return false;
    }
    maniBodyBone* bones = maniArenaTake(&r->arena, count, sizeof(maniBodyBone));
    if (bones == NULL && count > 0)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!Setting(r, e, &bones[k]))
        {
            return false;
        }
    }
    s->bones = bones;
    s->boneCount = count;
    return true;
}

bool maniGraphJsonBody(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                       maniGraphBodyNodeSource* s)
{
    static const char* const kModes[3] = {"pinned", "free", "prePull"};
    const uint32_t* f = members;
    *s = (maniGraphBodyNodeSource){.weight = {MANI_NO_PARAMETER, 1.0f}};
    uint32_t mode = f[5] != ABSENT ? maniGraphJsonWord(r, f[5], kModes, 3) : 0u;
    if (mode == 3)
    {
        return maniGraphJsonFault(r, f[5], maniGraphFormValue);
    }
    s->topMode = (maniBodyTopMode)mode;
    return maniGraphJsonBone(r, f[2], &s->top) && Effectors(r, scope, f[3], s) &&
           maniGraphJsonIndex(r, f[4], &s->iterations) &&
           (f[6] == ABSENT || Settings(r, f[6], s)) &&
           (f[7] == ABSENT || maniGraphJsonBool(r, f[7], &s->stretch)) &&
           (f[8] == ABSENT || maniGraphJsonFloat(r, f[8], &s->tolerance)) &&
           (f[9] == ABSENT || maniGraphJsonScalar(r, scope, f[9], &s->weight));
}

// ", key: " and a number.
static void PutNumber(maniGraphWriter* w, const char* key, float value)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutFloat(w, value);
}

static void PutWeight(maniGraphWriter* w, const maniGraphSource* source, const char* key,
                      maniGraphScalar s)
{
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, key);
    maniGraphJsonPutScalar(w, source, s);
}

static void PutEffector(maniGraphWriter* w, const maniGraphSource* source,
                        const maniGraphEffectorSource* e)
{
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "bone");
    maniGraphJsonPutBone(w, e->bone);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "position");
    maniGraphJsonPutParameterName(w, source, e->position);
    bool rotation = e->rotation != MANI_NO_PARAMETER;
    if (rotation)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "rotation");
        maniGraphJsonPutParameterName(w, source, e->rotation);
    }
    PutWeight(w, source, "positionWeight", e->positionWeight);
    if (rotation)
    {
        PutWeight(w, source, "rotationWeight", e->rotationWeight);
    }
    PutNumber(w, "strength", e->strength);
    PutNumber(w, "pullChain", e->pullChain);
    maniGraphJsonPutText(w, "}");
}

static void PutSetting(maniGraphWriter* w, const maniBodyBone* b)
{
    maniGraphJsonPutText(w, "{");
    maniGraphJsonPutKey(w, "bone");
    maniGraphJsonPutBone(w, b->bone);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "limit");
    maniGraphJsonPutLimit(w, &b->limit);
    maniGraphJsonPutVector(w, "preferred", b->preferred);
    PutNumber(w, "rotationStiffness", b->rotationStiffness);
    PutNumber(w, "positionStiffness", b->positionStiffness);
    maniGraphJsonPutText(w, "}");
}

void maniGraphJsonPutBody(maniGraphWriter* w, const maniGraphSource* source,
                          const maniGraphNodeSource* n)
{
    static const char* const kModes[3] = {"\"pinned\"", "\"free\"", "\"prePull\""};
    const maniGraphBodyNodeSource* s = &n->body;
    maniGraphJsonPutKey(w, "top");
    maniGraphJsonPutBone(w, s->top);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "effectors");
    for (uint32_t e = 0; e < s->effectorCount; ++e)
    {
        maniGraphJsonPutText(w, e == 0 ? "[" : ", ");
        PutEffector(w, source, &s->effectors[e]);
    }
    maniGraphJsonPutText(w, "], ");
    maniGraphJsonPutKey(w, "iterations");
    maniGraphJsonPutUnsigned(w, s->iterations);
    maniGraphJsonPutText(w, ", ");
    maniGraphJsonPutKey(w, "topMode");
    maniGraphJsonPutText(w, kModes[s->topMode]);
    if (s->boneCount > 0)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "bones");
        for (uint32_t k = 0; k < s->boneCount; ++k)
        {
            maniGraphJsonPutText(w, k == 0 ? "[" : ", ");
            PutSetting(w, &s->bones[k]);
        }
        maniGraphJsonPutText(w, "]");
    }
    if (s->stretch)
    {
        maniGraphJsonPutText(w, ", ");
        maniGraphJsonPutKey(w, "stretch");
        maniGraphJsonPutBool(w, true);
    }
    PutNumber(w, "tolerance", s->tolerance);
    PutWeight(w, source, "weight", s->weight);
}
