// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a graph file (mani-0023): the checked source's nodes ordered
// depth first from the output (each after its inputs, taken in order),
// each marked as giving poses or differences, each given a slot reused
// once its last reader has run, and every section written.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "graph_check.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_inline.h"
#include "graph_learned.h"
#include "graph_machine_check.h"
#include "graph_match.h"
#include "graph_match_check.h"
#include "graph_spring.h"
#include "ik_chain.h"
#include "sort.h"
#include "triangulate.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/ik.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define WORK_ARRAYS 9u

// Per node, by source index unless said: its place in the order (and the
// order itself, by place), its search state, the search's stack and next
// inputs, its last reader's place, its slot, the free slots, and whether
// it gives differences.
typedef struct Work
{
    uint32_t* order;
    uint32_t* position;
    uint32_t* state;
    uint32_t* stack;
    uint32_t* next;
    uint32_t* lastReader;
    uint32_t* slot;
    uint32_t* freeSlots;
    uint32_t* difference;
    uint32_t slotCount;
} Work;

static maniBuildResult Outcome(maniResult result)
{
    return (maniBuildResult){result, maniBuildReasonNone, 0, 0};
}

// Depth first from the output: each node after its inputs. Refuses a
// cycle (the node whose input closes it) and a node not reached.
static maniBuildResult Order(const maniGraphSource* source, Work* w)
{
    uint32_t n = source->nodeCount;
    memset(w->state, 0, n * sizeof(uint32_t));
    uint32_t depth = 1;
    uint32_t placed = 0;
    w->stack[0] = source->output;
    w->next[0] = 0;
    w->state[source->output] = 1;
    while (depth > 0)
    {
        uint32_t v = w->stack[depth - 1];
        const maniGraphNodeSource* node = &source->nodes[v];
        uint32_t k = w->next[depth - 1];
        if (k == node->inputCount)
        {
            --depth;
            w->state[v] = 2;
            w->position[v] = placed;
            w->order[placed++] = v;
            continue;
        }
        w->next[depth - 1] = k + 1;
        uint32_t u = node->inputs[k];
        if (w->state[u] == 1)
        {
            return maniGraphRefuse(maniBuildReasonCycle, v, k);
        }
        if (w->state[u] == 0)
        {
            w->state[u] = 1;
            w->stack[depth] = u;
            w->next[depth++] = 0;
        }
    }
    for (uint32_t v = 0; v < n; ++v)
    {
        if (w->state[v] == 0)
        {
            return maniGraphRefuse(maniBuildReasonUnreachable, v, 0);
        }
    }
    return Outcome(mani_success);
}

// Whether a node gives differences, from its inputs'; refuses a mix.
static maniBuildResult Kind(const maniGraphSource* source, uint32_t v, Work* w)
{
    const maniGraphNodeSource* node = &source->nodes[v];
    if (node->kind == maniGraphNodeMatch || node->kind == maniGraphNodeLearned ||
        node->kind == maniGraphNodePose)
    {
        w->difference[v] = 0;
        return Outcome(mani_success);
    }
    if (node->kind == maniGraphNodeClip)
    {
        const maniClip* clip = source->clips[node->clip.clip].clip;
        w->difference[v] = maniClipAdditiveBasis(clip) != maniAdditiveNone ? 1u : 0u;
        return Outcome(mani_success);
    }
    bool additive = node->kind == maniGraphNodeAdditive;
    bool posesOnly = node->kind == maniGraphNodeSpring || node->kind == maniGraphNodeTwoBone ||
                     (node->kind >= maniGraphNodeAim && node->kind <= maniGraphNodeBodySdls);
    uint32_t first = w->difference[node->inputs[0]];
    if ((additive || posesOnly) && first != 0)
    {
        return maniGraphRefuse(maniBuildReasonDifference, v, 0);
    }
    for (uint32_t k = 1; k < node->inputCount; ++k)
    {
        uint32_t d = w->difference[node->inputs[k]];
        if (additive ? d == 0 : d != first)
        {
            return maniGraphRefuse(maniBuildReasonDifference, v, k);
        }
    }
    w->difference[v] = first;
    return Outcome(mani_success);
}

// Slots over the order: a node's slot is taken before its inputs' are
// freed, so it is never one of theirs.
static void Slots(const maniGraphSource* source, Work* w)
{
    uint32_t n = source->nodeCount;
    for (uint32_t p = 0; p < n; ++p)
    {
        const maniGraphNodeSource* node = &source->nodes[w->order[p]];
        w->lastReader[w->order[p]] = p == n - 1 ? n : 0;
        for (uint32_t k = 0; k < node->inputCount; ++k)
        {
            w->lastReader[node->inputs[k]] = p;
        }
    }
    uint32_t freeCount = 0;
    w->slotCount = 0;
    for (uint32_t p = 0; p < n; ++p)
    {
        uint32_t v = w->order[p];
        w->slot[v] = freeCount > 0 ? w->freeSlots[--freeCount] : w->slotCount++;
        const maniGraphNodeSource* node = &source->nodes[v];
        for (uint32_t k = 0; k < node->inputCount; ++k)
        {
            uint32_t u = node->inputs[k];
            if (w->lastReader[u] == p)
            {
                w->freeSlots[freeCount++] = w->slot[u];
                w->lastReader[u] = n + 1;
            }
        }
    }
}

// The sections' bytes, laid out in one block.
typedef struct Layout
{
    uint8_t* block;
    size_t blockSize;
    uint8_t* sections[MANI_GRAPH_SECTIONS];
    uint64_t sizes[MANI_GRAPH_SECTIONS];
    uint64_t namesAt;
    // One mask's weights before they are copied in.
    float* weights;
    // A triangulation's working memory, when a space needs one.
    maniTriangulation* mesh;
    // Sorting a machine's transitions and a state's clips: keys, order
    // and scratch, as many as the most of either; the search below its
    // states and its clips.
    uint64_t* keys;
    uint32_t* order;
    uint32_t sortCount;
    maniGraphReach reach;
    maniGraphMachineWork machine;
    // The next machine's first word in an instance's machine state.
    uint32_t blockAt;
    uint32_t dataAt;
    uint32_t inputAt;
    uint32_t maskAt;
} Layout;

static bool Lay(const maniGraphSource* source, const maniGraphTally* tally, uint32_t bones,
                const maniAllocator* allocator, Layout* lay)
{
    lay->sizes[0] = MANI_GRAPH_HEADER_SIZE;
    lay->sizes[1] = (uint64_t)source->parameterCount * MANI_GRAPH_PARAMETER_SIZE;
    lay->sizes[2] = (uint64_t)source->nodeCount * MANI_GRAPH_NODE_SIZE;
    lay->sizes[3] = 4 * (uint64_t)tally->inputCount;
    lay->sizes[4] = 4 * (uint64_t)tally->dataWords;
    lay->sizes[5] = (uint64_t)source->clipCount * MANI_GRAPH_CLIP_SIZE;
    lay->sizes[6] = (uint64_t)source->syncGroupCount * MANI_GRAPH_SYNC_GROUP_SIZE;
    lay->sizes[7] = 4 * (uint64_t)tally->maskCount * bones;
    lay->sizes[8] = (uint64_t)source->databaseCount * MANI_GRAPH_DATABASE_SIZE;
    lay->sizes[9] = 4 * tally->databaseClips;
    lay->sizes[10] = tally->namesSize;
    lay->sizes[11] = (uint64_t)source->modelCount * MANI_GRAPH_MODEL_SIZE;
    lay->sizes[12] = (uint64_t)source->springSetCount * MANI_GRAPH_SPRING_SET_SIZE;
    uint64_t total = 0;
    for (uint32_t s = 0; s < MANI_GRAPH_SECTIONS; ++s)
    {
        total += (lay->sizes[s] + 7u) & ~(uint64_t)7u;
    }
    if (total > SIZE_MAX)
    {
        return false;
    }
    lay->blockSize = (size_t)total;
    lay->block = maniAllocate(allocator, lay->blockSize, 1, 8);
    if (lay->block == NULL)
    {
        return false;
    }
    memset(lay->block, 0, lay->blockSize);
    uint64_t at = 0;
    for (uint32_t s = 0; s < MANI_GRAPH_SECTIONS; ++s)
    {
        lay->sections[s] = lay->block + at;
        at += (lay->sizes[s] + 7u) & ~(uint64_t)7u;
    }
    return true;
}

// A name into the names section, its offset, length and hash after it.
static void PutName(Layout* lay, const char* name, uint32_t length, uint8_t* entry)
{
    memcpy(lay->sections[10] + lay->namesAt, name, length);
    maniStoreU32(entry, (uint32_t)lay->namesAt);
    maniStoreU32(entry + 4, length);
    lay->namesAt += length;
}

static void PutFloat(uint8_t* p, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    maniStoreU32(p, bits);
}

static void PutParameter(Layout* lay, const maniGraphParameterSource* p, uint8_t* e)
{
    maniStoreU32(e, p->type);
    maniStoreU32(e + 4, p->bounded != 0 ? MANI_GRAPH_BOUNDED : 0u);
    PutName(lay, p->name, p->nameLength, e + 8);
    maniStoreU64(e + 16, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)p->name, p->nameLength));
    maniGraphParameterType t = p->type;
    if (t == maniGraphParameterInt)
    {
        maniStoreU32(e + 24, (uint32_t)p->defaultValue.i);
        maniStoreU32(e + 40, p->bounded != 0 ? (uint32_t)p->minimum.i : 0u);
        maniStoreU32(e + 44, p->bounded != 0 ? (uint32_t)p->maximum.i : 0u);
        return;
    }
    if (t == maniGraphParameterBool || t == maniGraphParameterTrigger)
    {
        maniStoreU32(e + 24, p->defaultValue.b);
        return;
    }
    uint32_t n = t == maniGraphParameterFloat  ? 1u
                 : t == maniGraphParameterVec2 ? 2u
                 : t == maniGraphParameterVec3 ? 3u
                                               : 4u;
    double length = 0.0;
    for (uint32_t k = 0; k < n; ++k)
    {
        length += (double)p->defaultValue.f[k] * (double)p->defaultValue.f[k];
    }
    double scale = t == maniGraphParameterQuat ? 1.0 / sqrt(length) : 1.0;
    for (uint32_t k = 0; k < n; ++k)
    {
        PutFloat(e + 24 + 4 * k, (float)((double)p->defaultValue.f[k] * scale));
    }
    if (p->bounded != 0)
    {
        PutFloat(e + 40, p->minimum.f[0]);
        PutFloat(e + 44, p->maximum.f[0]);
    }
}

static uint32_t PutScalar(uint8_t* data, uint32_t at, maniGraphScalar s)
{
    maniStoreU32(data + 4 * (size_t)at, s.parameter);
    PutFloat(data + 4 * (size_t)at + 4, s.parameter == MANI_NO_PARAMETER ? s.value : 0.0f);
    return at + 2;
}

static uint32_t PutWords(uint8_t* data, uint32_t at, const uint32_t* words, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        maniStoreU32(data + 4 * (size_t)(at + k), words[k]);
    }
    return at + count;
}

static uint32_t PutFloats(uint8_t* data, uint32_t at, const float* values, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        PutFloat(data + 4 * (size_t)(at + k), values[k]);
    }
    return at + count;
}

// A triangulated space's count, points and triangles (the authored ones,
// or the points' Delaunay triangulation made in mesh); the reserved
// triangles past the count stay zero.
static uint32_t PutTriangles(uint8_t* d, uint32_t at, const maniGraphTrianglesNodeSource* s,
                             uint32_t n, maniTriangulation* mesh)
{
    uint32_t count = s->triangleCount;
    if (count == 0)
    {
        count = maniTriangulate(s->points, n, mesh);
        for (uint32_t t = 0; t < count; ++t)
        {
            (void)PutWords(d, at + 1 + 2 * n + 3 * t, mesh->t[t].v, 3);
        }
    }
    else
    {
        (void)PutWords(d, at + 1 + 2 * n, s->triangles, 3 * count);
    }
    at = PutFloats(d, PutWords(d, at, &count, 1), s->points, 2 * n);
    return at + 3 * MANI_GRAPH_MAX_TRIANGLES(n);
}

// A blend space's position, its word (period, polar flag or triangle
// count), its points and, on a line, their order, in a triangulated
// space its triangles.
static uint32_t PutSpace(uint8_t* d, uint32_t at, const maniGraphNodeSource* node,
                         maniTriangulation* mesh)
{
    bool line = node->kind == maniGraphNodeBlendSpace1D;
    bool bands = node->kind == maniGraphNodeBlendSpaceBands;
    const maniGraphPosition* p = line    ? &node->blendSpace1D.position
                                 : bands ? &node->bands.position
                                         : &node->triangulated.position;
    bool vector = p->vector != MANI_NO_PARAMETER;
    const maniGraphScalar zero = {MANI_NO_PARAMETER, 0.0f};
    at = PutWords(d, at, &p->vector, 1);
    at = PutScalar(d, PutScalar(d, at, vector ? zero : p->x), vector || line ? zero : p->y);
    uint32_t n = node->inputCount;
    if (bands)
    {
        at = PutWords(d, at, &node->bands.polar, 1);
        return PutFloats(d, at, node->bands.points, 2 * n);
    }
    if (!line)
    {
        return PutTriangles(d, at, &node->triangulated, n, mesh);
    }
    PutFloat(d + 4 * (size_t)at, node->blendSpace1D.period);
    at = PutFloats(d, at + 1, node->blendSpace1D.points, n);
    uint64_t keys[MANI_GRAPH_MAX_SPACE_POINTS];
    uint32_t order[MANI_GRAPH_MAX_SPACE_POINTS];
    uint32_t scratch[MANI_GRAPH_MAX_SPACE_POINTS];
    for (uint32_t k = 0; k < n; ++k)
    {
        keys[k] = maniFloatKey(node->blendSpace1D.points[k]);
    }
    maniSortByKey(keys, n, order, scratch);
    return PutWords(d, at, order, n);
}

// A node's data words, by kind; a mask's weights made.
// A mask's per-bone weights, the next in the masks section.
static void PutMask(const maniGraphSource* source, const maniGraphMaskNodeSource* m, Layout* lay)
{
    uint32_t bones = maniSkeletonBoneCount(source->skeleton);
    // Checked already: the chains' bones and weights are in range.
    (void)maniBuildMask(source->skeleton, m->chains, m->chainCount, m->defaultWeight, lay->weights,
                        bones);
    memcpy(lay->sections[7] + 4 * (size_t)lay->maskAt * bones, lay->weights, 4 * (size_t)bones);
    ++lay->maskAt;
}

// A one-shot's words, its mask's weights, and its block taken.
static uint32_t PutShot(const maniGraphSource* source, const maniGraphNodeSource* node, uint8_t* d,
                        uint32_t at, Layout* lay)
{
    const maniGraphOneShotNodeSource* s = &node->oneShot;
    uint32_t count = maniGraphSortedClips(&lay->machine, node->inputs[1], NULL);
    float fades[2] = {s->fadeIn, s->fadeOut};
    uint32_t bits[2];
    memcpy(bits, fades, sizeof bits);
    const uint32_t words[9] = {s->fire,
                               s->fadeOutTrigger,
                               s->abort,
                               s->curve,
                               bits[0],
                               bits[1],
                               s->mask != NULL ? lay->maskAt : MANI_GRAPH_NO_MASK,
                               lay->blockAt,
                               count};
    at = PutWords(d, at, words, 9);
    for (uint32_t c = 0; c < count; ++c)
    {
        at = PutWords(d, at, &lay->machine.clips[lay->machine.order[c]], 1);
    }
    if (s->mask != NULL)
    {
        PutMask(source, s->mask, lay);
    }
    lay->blockAt += MANI_GRAPH_SHOT_BLOCK_WORDS;
    return at;
}

// A motion matching node's words, and its block taken.
static uint32_t PutMatch(uint8_t* d, uint32_t at, const maniGraphMatchNodeSource* m, Layout* lay)
{
    const uint32_t head[2] = {m->database, m->trigger};
    const float numbers[6] = {m->period,      m->jumpThreshold,  m->endExclusion,
                              m->historyTime, m->continuingBias, m->transitionCost};
    const uint32_t tags[5] = {m->historyCount, m->requiredTags.parameter, m->requiredTags.value,
                              m->excludedTags.parameter, m->excludedTags.value};
    const uint32_t tail[2] = {lay->blockAt, m->queryCount};
    at = PutFloats(d, PutWords(d, at, head, 2), numbers, 6);
    at = PutScalar(d, PutWords(d, at, tags, 5), m->duration);
    at = PutWords(d, PutWords(d, at, tail, 2), m->query, m->queryCount);
    lay->blockAt += MANI_GRAPH_MATCH_BLOCK_WORDS;
    return at;
}

static uint32_t PutLearned(const maniGraphSource* source, uint8_t* d, uint32_t at,
                           const maniGraphLearnedNodeSource* l, Layout* lay)
{
    const maniLearnedModel* m = source->models[l->model].model;
    uint32_t states = m->featureCount + m->latentCount;
    const uint32_t head[2] = {l->model, l->trigger};
    const float numbers[2] = {l->period, l->transitionCost};
    const uint32_t tags[4] = {l->requiredTags.parameter, l->requiredTags.value,
                              l->excludedTags.parameter, l->excludedTags.value};
    const uint32_t tail[5] = {lay->blockAt, states, m->featureCount, m->eventCount, l->queryCount};
    at = PutFloats(d, PutWords(d, at, head, 2), numbers, 2);
    at = PutScalar(d, PutWords(d, at, tags, 4), l->duration);
    at = PutWords(d, PutWords(d, at, tail, 5), l->query, l->queryCount);
    lay->blockAt += MANI_GRAPH_LEARNED_BLOCK_WORDS(states, m->eventCount);
    return at;
}

// A spring node's words: its slot and parameters, its block, its set's
// state words.
static uint32_t PutSpring(const maniGraphSource* source, uint8_t* d, uint32_t at,
                          const maniGraphSpringNodeSource* s, Layout* lay)
{
    uint32_t words = (uint32_t)maniGraphSpringSetWords(source->springSets[s->springSet].set);
    const uint32_t head[MANI_GRAPH_SPRING_WORDS] = {
        s->springSet, s->position, s->rotation, s->reset, lay->blockAt, words, s->force};
    lay->blockAt += maniGraphSpringState + words;
    return PutWords(d, at, head, MANI_GRAPH_SPRING_WORDS);
}

// A two-bone node's words: its bones and parameters, hinge and scalars.
static uint32_t PutTwoBone(uint8_t* d, uint32_t at, const maniGraphTwoBoneNodeSource* s)
{
    const uint32_t head[5] = {s->start, s->middle, s->end, s->goal, s->pole};
    const float hinge[3] = {s->hinge.x, s->hinge.y, s->hinge.z};
    at = PutFloats(d, PutWords(d, at, head, 5), hinge, 3);
    return PutScalar(d, PutScalar(d, PutScalar(d, at, s->twist), s->soften), s->weight);
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
    at = PutWords(d, PutWords(d, at, head, 3), bones, MANI_AIM_MAX_BONES);
    at = PutFloats(d, PutFloats(d, at, shares, MANI_AIM_MAX_BONES), axes, 9);
    return PutScalar(d, PutScalar(d, PutScalar(d, at, s->twist), s->cone), s->weight);
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
    return PutFloats(d, PutWords(d, at, &l->kind, 1), words, 8);
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
    at = PutFloats(d, PutWords(d, at, head, 6), numbers, 2);
    at = PutScalar(d, PutScalar(d, at, s->rotationWeight), s->weight);
    at = PutWords(d, at, &limits, 1);
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
    at = PutScalar(d, PutFloats(d, PutWords(d, at, head, 4), &s->tolerance, 1), s->weight);
    at = PutWords(d, at, counts, 2);
    const maniGraphScalar zero = {.parameter = MANI_NO_PARAMETER, .value = 0.0f};
    for (uint32_t e = 0; e < s->effectorCount; ++e)
    {
        const maniGraphEffectorSource* f = &s->effectors[e];
        const uint32_t words[3] = {f->bone, f->position, f->rotation};
        const float numbers[2] = {f->strength, f->pullChain};
        at = PutScalar(d, PutWords(d, at, words, 3), f->positionWeight);
        at = PutScalar(d, at, f->rotation == MANI_NO_PARAMETER ? zero : f->rotationWeight);
        at = PutFloats(d, at, numbers, 2);
    }
    for (uint32_t k = 0; k < s->boneCount; ++k)
    {
        const maniBodyBone* b = &s->bones[k];
        const float rest[5] = {b->preferred.x, b->preferred.y, b->preferred.z, b->rotationStiffness,
                               b->positionStiffness};
        at = PutFloats(d, PutLimit(d, PutWords(d, at, &b->bone, 1), &b->limit), rest, 5);
    }
    return at;
}

// An IK node's words (kinds two-bone to full-body SDLS), by its kind.
static uint32_t PutIk(const maniGraphSource* source, uint8_t* d, uint32_t at,
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

// A blend's direct flag and weights, or an additive's weights.
static uint32_t PutWeights(uint8_t* d, uint32_t at, const maniGraphNodeSource* node)
{
    uint32_t n = node->inputCount;
    bool blend = node->kind == maniGraphNodeBlend;
    at = blend ? PutWords(d, at, &node->blend.direct, 1) : at;
    const maniGraphScalar* weights = blend ? node->blend.weights : node->additive.weights;
    for (uint32_t k = 0; k < (blend ? n : n - 1); ++k)
    {
        at = PutScalar(d, at, weights[k]);
    }
    return at;
}

static void PutData(const maniGraphSource* source, const maniGraphNodeSource* node, Layout* lay)
{
    uint8_t* d = lay->sections[4];
    uint32_t at = lay->dataAt;
    uint32_t n = node->inputCount;
    if (node->kind == maniGraphNodeClip)
    {
        const maniGraphClipNodeSource* c = &node->clip;
        const uint32_t words[4] = {c->clip, c->syncGroup, c->loop, c->flags};
        at = PutScalar(d, PutWords(d, at, words, 4), c->speed);
    }
    else if (node->kind == maniGraphNodeBlend || node->kind == maniGraphNodeAdditive)
    {
        at = PutWeights(d, at, node);
    }
    else if (node->kind == maniGraphNodeMask)
    {
        at = PutWords(d, at, &lay->maskAt, 1);
        PutMask(source, &node->mask, lay);
    }
    else if (node->kind == maniGraphNodeOneShot)
    {
        at = PutShot(source, node, d, at, lay);
    }
    else if (node->kind == maniGraphNodeInertialize)
    {
        const maniGraphInertializeNodeSource* s = &node->inertialize;
        const float halfLives[3] = {s->halfLifeScale, s->halfLifeMin, s->halfLifeMax};
        const uint32_t head[2] = {s->trigger, s->method};
        at = PutScalar(d, PutWords(d, at, head, 2), s->duration);
        at = PutWords(d, PutFloats(d, at, halfLives, 3), &lay->blockAt, 1);
        lay->blockAt +=
            MANI_GRAPH_INERTIA_BLOCK(maniSkeletonBoneCount(source->skeleton), s->method);
    }
    else if (node->kind == maniGraphNodeMatch)
    {
        at = PutMatch(d, at, &node->match, lay);
    }
    else if (node->kind == maniGraphNodeLearned)
    {
        at = PutLearned(source, d, at, &node->learned, lay);
    }
    else if (node->kind == maniGraphNodeSpring)
    {
        at = PutSpring(source, d, at, &node->spring, lay);
    }
    else if (node->kind == maniGraphNodeSelect)
    {
        at = PutWords(d, at, &node->select.parameter, 1);
    }
    else if (node->kind == maniGraphNodePose)
    {
        at = PutWords(d, at, &node->pose.parameter, 1);
    }
    else if (node->kind >= maniGraphNodeTwoBone && node->kind <= maniGraphNodeBodySdls)
    {
        at = PutIk(source, d, at, node);
    }
    else if (node->kind == maniGraphNodeTimeScale)
    {
        at = PutScalar(d, at, node->timeScale.scale);
    }
    else if (node->kind == maniGraphNodeBlendSpace1D ||
             node->kind == maniGraphNodeBlendSpaceBands ||
             node->kind == maniGraphNodeBlendSpaceTriangles)
    {
        at = PutSpace(d, at, node, lay->mesh);
    }
    else if (node->kind == maniGraphNodeMachine)
    {
        at = maniPutGraphMachine(d, at, node, lay->blockAt, &lay->machine);
        lay->blockAt += MANI_GRAPH_BLOCK_SIZE(n, node->machine.transitionCount,
                                              maniGraphMachineConditions(node));
    }
    else
    {
        at = PutScalar(d, PutWords(d, at, &node->seek.trigger, 1), node->seek.time);
    }
    lay->dataAt = at;
}

static void PutNodes(const maniGraphSource* source, const Work* w, Layout* lay)
{
    for (uint32_t p = 0; p < source->nodeCount; ++p)
    {
        uint32_t v = w->order[p];
        const maniGraphNodeSource* node = &source->nodes[v];
        uint8_t* e = lay->sections[2] + (size_t)p * MANI_GRAPH_NODE_SIZE;
        maniStoreU16(e, (uint16_t)node->kind);
        maniStoreU16(e + 2, (uint16_t)(w->difference[v] != 0 ? MANI_GRAPH_DIFFERENCE : 0u));
        maniStoreU32(e + 4, lay->inputAt);
        maniStoreU32(e + 8, node->inputCount);
        maniStoreU32(e + 12, lay->dataAt);
        maniStoreU32(e + 20, w->slot[v]);
        for (uint32_t k = 0; k < node->inputCount; ++k)
        {
            maniStoreU32(lay->sections[3] + 4 * (size_t)lay->inputAt++,
                         w->position[node->inputs[k]]);
        }
        uint32_t first = lay->dataAt;
        PutData(source, node, lay);
        bool counted = node->kind == maniGraphNodeMachine || node->kind == maniGraphNodeOneShot ||
                       node->kind == maniGraphNodeMatch || node->kind == maniGraphNodeLearned ||
                       (node->kind >= maniGraphNodeCcd && node->kind <= maniGraphNodeBodySdls);
        maniStoreU32(e + 16, counted ? lay->dataAt - first
                                     : maniGraphDataWords(node->kind, node->inputCount));
    }
}

// A slot's entry: the asset's identity, the name's hash and the name.
static void PutSlot(Layout* lay, uint8_t* e, uint64_t identity, const char* name, uint32_t length)
{
    maniStoreU64(e, identity);
    maniStoreU64(e + 8, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length));
    PutName(lay, name, length, e + 16);
}

// The model and spring set slots.
static void PutSlots(const maniGraphSource* source, Layout* lay)
{
    for (uint32_t i = 0; i < source->modelCount; ++i)
    {
        const maniGraphModelSource* m = &source->models[i];
        PutSlot(lay, lay->sections[11] + (size_t)i * MANI_GRAPH_MODEL_SIZE,
                maniLearnedModelIdentity(m->model), m->name, m->nameLength);
    }
    for (uint32_t i = 0; i < source->springSetCount; ++i)
    {
        const maniGraphSpringSetSource* p = &source->springSets[i];
        PutSlot(lay, lay->sections[12] + (size_t)i * MANI_GRAPH_SPRING_SET_SIZE,
                maniSpringSetIdentity(p->set), p->name, p->nameLength);
    }
}

static void PutLists(const maniGraphSource* source, Layout* lay)
{
    for (uint32_t i = 0; i < source->parameterCount; ++i)
    {
        PutParameter(lay, &source->parameters[i],
                     lay->sections[1] + (size_t)i * MANI_GRAPH_PARAMETER_SIZE);
    }
    for (uint32_t i = 0; i < source->clipCount; ++i)
    {
        const maniGraphClipSource* c = &source->clips[i];
        uint8_t* e = lay->sections[5] + (size_t)i * MANI_GRAPH_CLIP_SIZE;
        maniStoreU64(e, maniClipIdentity(c->clip));
        maniStoreU64(e + 8, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)c->name, c->nameLength));
        PutName(lay, c->name, c->nameLength, e + 16);
        bool difference = maniClipAdditiveBasis(c->clip) != maniAdditiveNone;
        maniStoreU32(e + 24, difference ? MANI_GRAPH_DIFFERENCE : 0u);
    }
    for (uint32_t i = 0; i < source->syncGroupCount; ++i)
    {
        const maniGraphSyncGroupSource* g = &source->syncGroups[i];
        uint8_t* e = lay->sections[6] + (size_t)i * MANI_GRAPH_SYNC_GROUP_SIZE;
        maniStoreU64(e, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)g->name, g->nameLength));
        PutName(lay, g->name, g->nameLength, e + 8);
        maniStoreU32(e + 16, g->flags);
    }
    uint32_t first = 0;
    for (uint32_t i = 0; i < source->databaseCount; ++i)
    {
        const maniGraphDatabaseSource* b = &source->databases[i];
        uint8_t* e = lay->sections[8] + (size_t)i * MANI_GRAPH_DATABASE_SIZE;
        maniStoreU64(e, maniMotionDatabaseIdentity(b->database));
        maniStoreU64(e + 8, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)b->name, b->nameLength));
        PutName(lay, b->name, b->nameLength, e + 16);
        maniStoreU32(e + 24, first);
        maniStoreU32(e + 28, b->database->clipCount);
        for (uint32_t c = 0; c < b->database->clipCount; ++c, ++first)
        {
            maniStoreU32(lay->sections[9] + 4 * (size_t)first,
                         maniGraphDatabaseClipSlot(source, b->database, c));
        }
    }
    PutSlots(source, lay);
}

static void PutHeader(const maniGraphSource* source, const maniGraphTally* tally,
                      uint32_t slotCount, uint8_t* h)
{
    maniStoreU64(h, maniSkeletonIdentity(source->skeleton));
    const uint32_t counts[13] = {maniSkeletonBoneCount(source->skeleton),
                                 source->parameterCount,
                                 source->nodeCount,
                                 tally->inputCount,
                                 (uint32_t)tally->dataWords,
                                 source->clipCount,
                                 source->syncGroupCount,
                                 tally->maskCount,
                                 slotCount,
                                 source->databaseCount,
                                 (uint32_t)tally->databaseClips,
                                 source->modelCount,
                                 source->springSetCount};
    for (uint32_t k = 0; k < 13; ++k)
    {
        maniStoreU32(h + 8 + 4 * k, counts[k]);
    }
}

// Writes the container and gives it to the builder.
static maniBuildResult Keep(maniBuilder* builder, const Layout* lay)
{
    // Empty sections are left out, as every kind leaves them.
    maniSectionSource sections[MANI_GRAPH_SECTIONS];
    uint32_t count = 0;
    for (uint32_t s = 0; s < MANI_GRAPH_SECTIONS; ++s)
    {
        if (lay->sizes[s] > 0)
        {
            sections[count++] = (maniSectionSource){s + 1, lay->sections[s], lay->sizes[s]};
        }
    }
    const uint8_t* data[MANI_GRAPH_SECTIONS];
    for (uint32_t s = 0; s < MANI_GRAPH_SECTIONS; ++s)
    {
        data[s] = lay->sections[s];
    }
    uint64_t identity = maniGraphComputeIdentity(data, lay->sizes);
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    maniWriteContainer(file, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, identity, sections, count);
    maniBuilderKeep(builder, maniKindGraph, file, (size_t)fileSize, NULL, 0);
    return Outcome(mani_success);
}

// Whether a space of the source is to be triangulated.
static bool Triangulates(const maniGraphSource* source)
{
    for (uint32_t v = 0; v < source->nodeCount; ++v)
    {
        const maniGraphNodeSource* node = &source->nodes[v];
        if (node->kind == maniGraphNodeBlendSpaceTriangles && node->triangulated.triangleCount == 0)
        {
            return true;
        }
    }
    return false;
}

static void Release(const maniAllocator* allocator, Layout* lay, uint32_t bones)
{
    maniRelease(allocator, lay->block, lay->blockSize, 1, 8);
    maniRelease(allocator, lay->weights, bones, sizeof(float), 4);
    maniRelease(allocator, lay->mesh, 1, sizeof(maniTriangulation), 8);
    maniRelease(allocator, lay->keys, lay->sortCount, sizeof(uint64_t), 8);
    maniRelease(allocator, lay->order, 2 * (size_t)lay->sortCount, sizeof(uint32_t), 4);
}

static maniBuildResult Write(maniBuilder* builder, const maniGraphSource* source,
                             const maniGraphTally* tally, const Work* w)
{
    Layout lay = {0};
    uint32_t bones = maniSkeletonBoneCount(source->skeleton);
    lay.weights = maniAllocate(&builder->allocator, bones, sizeof(float), 4);
    bool mesh = Triangulates(source);
    lay.mesh = mesh ? maniAllocate(&builder->allocator, 1, sizeof(maniTriangulation), 8) : NULL;
    lay.sortCount =
        tally->maxTransitions > tally->longestList ? tally->maxTransitions : tally->longestList;
    bool sort = lay.sortCount > 0;
    lay.keys = sort ? maniAllocate(&builder->allocator, lay.sortCount, sizeof(uint64_t), 8) : NULL;
    lay.order =
        sort ? maniAllocate(&builder->allocator, 2 * (size_t)lay.sortCount, sizeof(uint32_t), 4)
             : NULL;
    if (lay.weights == NULL || (mesh && lay.mesh == NULL) ||
        (sort && (lay.keys == NULL || lay.order == NULL)) ||
        !Lay(source, tally, bones, &builder->allocator, &lay))
    {
        Release(&builder->allocator, &lay, bones);
        return Outcome(mani_errorCapacity);
    }
    // The order's state, stack and next inputs are free once it is made.
    memset(w->state, 0, source->nodeCount * sizeof(uint32_t));
    lay.reach = (maniGraphReach){source, w->position, w->state, w->stack, 0, 0};
    lay.machine =
        (maniGraphMachineWork){lay.keys, lay.order, lay.order + lay.sortCount, w->next, &lay.reach};
    PutHeader(source, tally, w->slotCount, lay.sections[0]);
    PutLists(source, &lay);
    PutNodes(source, w, &lay);
    maniBuildResult r = Keep(builder, &lay);
    Release(&builder->allocator, &lay, bones);
    return r;
}

// Orders, marks and slots the checked source's nodes, then writes it.
static maniBuildResult Arrange(maniBuilder* builder, const maniGraphSource* source,
                               const maniGraphTally* tally)
{
    size_t n = source->nodeCount;
    uint32_t* block = maniAllocate(&builder->allocator, WORK_ARRAYS * n, sizeof(uint32_t), 4);
    if (block == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    Work w = {block,         block + n,     block + 2 * n, block + 3 * n, block + 4 * n,
              block + 5 * n, block + 6 * n, block + 7 * n, block + 8 * n, 0};
    maniBuildResult r = Order(source, &w);
    for (uint32_t p = 0; p < n && r.result == mani_success; ++p)
    {
        r = Kind(source, w.order[p], &w);
    }
    if (r.result == mani_success && w.difference[source->output] != 0)
    {
        r = maniGraphRefuse(maniBuildReasonDifference, source->output, 0);
    }
    if (r.result == mani_success)
    {
        Slots(source, &w);
        r = Write(builder, source, tally, &w);
    }
    maniRelease(&builder->allocator, block, WORK_ARRAYS * n, sizeof(uint32_t), 4);
    return r;
}

maniBuildResult maniBuildGraph(maniBuilder* builder, const maniGraphSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Outcome(mani_errorInvalid);
    }
    if (source == NULL || source->skeleton == NULL)
    {
        maniBuilderMisuse(builder);
        return Outcome(mani_errorInvalid);
    }
    maniGraphFlat flat;
    maniBuildResult r = maniInlineSubgraphs(source, &builder->allocator, &flat);
    maniGraphTally tally;
    r = r.result == mani_success ? maniCheckGraphSource(&flat.source, &builder->allocator, &tally)
                                 : r;
    r = r.result == mani_success ? Arrange(builder, &flat.source, &tally) : r;
    maniReleaseInlined(&flat, &builder->allocator);
    return r;
}
