// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_body_check.h"

#include "graph_file.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static bool ParameterIs(const maniGraphSource* source, uint32_t parameter,
                        maniGraphParameterType type)
{
    return parameter < source->parameterCount && source->parameters[parameter].type == type;
}

// A float parameter, or a number in [0, 1].
static bool WeightValid(const maniGraphSource* source, maniGraphScalar s)
{
    return s.parameter == MANI_NO_PARAMETER
               ? s.value >= 0.0f && s.value <= 1.0f
               : ParameterIs(source, s.parameter, maniGraphParameterFloat);
}

// Effector e's parameters and weights; its goals placeholders in out.
static maniBuildResult CheckEffector(const maniGraphSource* source,
                                     const maniGraphEffectorSource* f, uint32_t node, uint32_t e,
                                     maniBodyEffector* out)
{
    bool none = f->rotation == MANI_NO_PARAMETER;
    bool ok = ParameterIs(source, f->position, maniGraphParameterVec3) &&
              (none || ParameterIs(source, f->rotation, maniGraphParameterQuat));
    *out = (maniBodyEffector){.bone = f->bone,
                              .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                              .positionWeight = 1.0f,
                              .rotationWeight = 1.0f,
                              .strength = f->strength,
                              .pullChain = f->pullChain};
    if (!ok)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    if (!WeightValid(source, f->positionWeight))
    {
        return Refuse(maniBuildReasonScalar, node, 2 * e);
    }
    bool zero = f->rotationWeight.parameter == MANI_NO_PARAMETER && f->rotationWeight.value == 0.0f;
    return WeightValid(source, f->rotationWeight) && (!none || zero)
               ? kOk
               : Refuse(maniBuildReasonScalar, node, 2 * e + 1);
}

maniBuildResult maniCheckGraphBody(const maniGraphSource* source, const maniGraphNodeSource* n,
                                   uint32_t node)
{
    const maniGraphBodyNodeSource* s = &n->body;
    if (s->effectors == NULL || s->effectorCount < 1 ||
        s->effectorCount > MANI_BODY_IK_MAX_EFFECTORS ||
        (s->stretch && n->kind != maniGraphNodeBodyXpbd))
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    maniBodyEffector effectors[MANI_BODY_IK_MAX_EFFECTORS];
    for (uint32_t e = 0; e < s->effectorCount; ++e)
    {
        maniBuildResult r = CheckEffector(source, &s->effectors[e], node, e, &effectors[e]);
        if (r.result != mani_success)
        {
            return r;
        }
    }
    const maniBodySolve solve = {.top = s->top,
                                 .topMode = s->topMode,
                                 .effectors = effectors,
                                 .bones = s->bones,
                                 .effectorCount = s->effectorCount,
                                 .boneCount = s->boneCount,
                                 .iterations = s->iterations,
                                 .stretch = s->stretch,
                                 .tolerance = s->tolerance,
                                 .weight = 1.0f};
    if (maniBodyIkScratchSize(source->skeleton, &solve) == 0)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    return WeightValid(source, s->weight)
               ? kOk
               : Refuse(maniBuildReasonScalar, node, 2 * s->effectorCount);
}

uint32_t maniGraphBodySourceWords(const maniGraphNodeSource* n)
{
    return MANI_GRAPH_BODY_WORDS + MANI_GRAPH_EFFECTOR_WORDS * n->body.effectorCount +
           MANI_GRAPH_SETTING_WORDS * n->body.boneCount;
}
