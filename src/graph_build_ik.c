// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An IK or full-body node's data words (mani-0029 to mani-0031): bones,
// parameters, axes and scalars as graph_file.h lays them out, a chain's
// limits one per turning bone with the words a limit's kind leaves
// unused 0.

#include "graph_build_ik.h"

#include "bytes.h"
#include "graph_build_words.h"
#include "graph_file.h"
#include "ik_chain.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/ik.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// A two-bone node's words: its bones and parameters, hinge and scalars.
static uint32_t PutTwoBone(uint8_t* d, uint32_t at, const maniGraphTwoBoneNodeSource* s)
{
    const uint32_t head[5] = {s->start, s->middle, s->end, s->goal, s->pole};
    const float hinge[3] = {s->hinge.x, s->hinge.y, s->hinge.z};
    at = maniGraphPutFloats(d, maniGraphPutWords(d, at, head, 5), hinge, 3);
    return maniGraphPutScalar(
        d, maniGraphPutScalar(d, maniGraphPutScalar(d, at, s->twist), s->soften), s->weight);
}

// An aim node's words: its count and parameters, its bones and shares
// (1 each for equal ones) with zeros past the count, axes and scalars.
static uint32_t PutAim(uint8_t* d, uint32_t at, const maniGraphAimNodeSource* s)
{
    uint32_t bones[MANI_AIM_MAX_BONES] = {0};
    float shares[MANI_AIM_MAX_BONES] = {0};
    for (uint32_t k = 0; k < s->count; ++k)
    {
        bones[k] = s->bones[k];
        shares[k] = s->shares != NULL ? s->shares[k] : 1.0f;
    }
    const uint32_t head[3] = {s->count, s->goal, s->pole};
    const float axes[9] = {s->forward.x, s->forward.y, s->forward.z, s->offset.x, s->offset.y,
                           s->offset.z,  s->up.x,      s->up.y,      s->up.z};
    at = maniGraphPutWords(d, maniGraphPutWords(d, at, head, 3), bones, MANI_AIM_MAX_BONES);
    at = maniGraphPutFloats(d, maniGraphPutFloats(d, at, shares, MANI_AIM_MAX_BONES), axes, 9);
    return maniGraphPutScalar(
        d, maniGraphPutScalar(d, maniGraphPutScalar(d, at, s->twist), s->cone), s->weight);
}

// A limit's kind and numbers, the words its kind leaves unused 0.
static uint32_t PutLimit(uint8_t* d, uint32_t at, const maniJointLimit* l)
{
    bool cone = l->kind == maniJointLimitCone;
    bool hinge = l->kind == maniJointLimitHinge;
    const float words[8] = {hinge ? l->axis.x : 0.0f,  hinge ? l->axis.y : 0.0f,
                            hinge ? l->axis.z : 0.0f,  cone ? l->cone : 0.0f,
                            hinge ? l->low : 0.0f,     hinge ? l->high : 0.0f,
                            cone ? l->twistLow : 0.0f, cone ? l->twistHigh : 0.0f};
    return maniGraphPutFloats(d, maniGraphPutWords(d, at, &l->kind, 1), words, 8);
}

// A chain node's words: its bones, parameters, counts and numbers, its
// scalars, then its limits (one per turning bone, or none), each with
// the words its kind leaves unused 0.
static uint32_t PutChain(const maniGraphSource* source, uint8_t* d, uint32_t at,
                         const maniGraphChainNodeSource* s)
{
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    uint32_t count = maniIkChainPath(source->skeleton, s->root, s->end, bones);
    uint32_t limits = s->limits != NULL ? count - 1 : 0u;
    const uint32_t head[6] = {s->root, s->end, s->goal, s->pole, s->rotation, s->iterations};
    const float numbers[2] = {s->tolerance, s->step};
    at = maniGraphPutFloats(d, maniGraphPutWords(d, at, head, 6), numbers, 2);
    at = maniGraphPutScalar(d, maniGraphPutScalar(d, at, s->rotationWeight), s->weight);
    at = maniGraphPutWords(d, at, &limits, 1);
    for (uint32_t k = 0; k < limits; ++k)
    {
        at = PutLimit(d, at, &s->limits[k]);
    }
    return at;
}

// A full-body node's words: its head and counts, its effectors (the
// rotation weight the number 0 without a rotation), then its settings.
static uint32_t PutBody(uint8_t* d, uint32_t at, const maniGraphBodyNodeSource* s)
{
    const uint32_t head[4] = {s->top, (uint32_t)s->topMode, s->stretch ? 1u : 0u, s->iterations};
    const uint32_t counts[2] = {s->effectorCount, s->boneCount};
    at = maniGraphPutScalar(
        d, maniGraphPutFloats(d, maniGraphPutWords(d, at, head, 4), &s->tolerance, 1), s->weight);
    at = maniGraphPutWords(d, at, counts, 2);
    const maniGraphScalar zero = {.parameter = MANI_NO_PARAMETER, .value = 0.0f};
    for (uint32_t e = 0; e < s->effectorCount; ++e)
    {
        const maniGraphEffectorSource* f = &s->effectors[e];
        const uint32_t words[3] = {f->bone, f->position, f->rotation};
        const float numbers[2] = {f->strength, f->pullChain};
        at = maniGraphPutScalar(d, maniGraphPutWords(d, at, words, 3), f->positionWeight);
        at = maniGraphPutScalar(d, at, f->rotation == MANI_NO_PARAMETER ? zero : f->rotationWeight);
        at = maniGraphPutFloats(d, at, numbers, 2);
    }
    for (uint32_t k = 0; k < s->boneCount; ++k)
    {
        const maniBodyBone* b = &s->bones[k];
        const float rest[5] = {b->preferred.x, b->preferred.y, b->preferred.z, b->rotationStiffness,
                               b->positionStiffness};
        at = maniGraphPutFloats(d, PutLimit(d, maniGraphPutWords(d, at, &b->bone, 1), &b->limit),
                                rest, 5);
    }
    return at;
}

// An IK node's words (kinds two-bone to full-body SDLS), by its kind.
uint32_t maniGraphPutIk(const maniGraphSource* source, uint8_t* d, uint32_t at,
                        const maniGraphNodeSource* node)
{
    switch (node->kind)
    {
    case maniGraphNodeTwoBone:
        return PutTwoBone(d, at, &node->twoBone);
    case maniGraphNodeAim:
        return PutAim(d, at, &node->aim);
    case maniGraphNodeBodyXpbd:
    case maniGraphNodeBodySdls:
        return PutBody(d, at, &node->body);
    default:
        return PutChain(source, d, at, &node->chain);
    }
}
