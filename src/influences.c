// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening an influences file: every joint below the skin's
// joint count, every weight finite and from 0 to 1, the optional
// sections checked when present, so the skinning jobs read without
// checks.

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "influence_file.h"

#include "maul-anim/base.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, offset};
}

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// Finite (the exponent not all ones).
static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

// From 0 to 1 (NaN fails).
static bool Unit(float value)
{
    return value >= 0.0f && value <= 1.0f;
}

uint64_t maniInfluenceComputeIdentity(const uint8_t* const data[MANI_INFLUENCE_SECTIONS],
                                      const uint64_t sizes[MANI_INFLUENCE_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_INFLUENCE_SECTIONS; ++i)
    {
        hash = maniHashBytes(hash, data[i], sizes[i]);
    }
    return hash;
}

// The file's sections by tag (index tag - 1), an absent one empty.
typedef struct Sections
{
    const uint8_t* data[MANI_INFLUENCE_SECTIONS];
    uint64_t sizes[MANI_INFLUENCE_SECTIONS];
    uint64_t at[MANI_INFLUENCE_SECTIONS];
    // Where each one's size is in the table; 12 (the section count) for
    // an absent one.
    uint64_t entry[MANI_INFLUENCE_SECTIONS];
} Sections;

static const uint8_t kEmpty = 0;

// Tags 1 to 5 (the container keeps them increasing), none empty: an
// optional section is left out instead.
static maniFileResult LoadSections(const maniContainer* c, Sections* s)
{
    for (uint32_t i = 0; i < MANI_INFLUENCE_SECTIONS; ++i)
    {
        s->data[i] = &kEmpty;
        s->sizes[i] = 0;
        s->at[i] = 0;
        s->entry[i] = 12;
    }
    for (uint32_t i = 0; i < c->sectionCount; ++i)
    {
        uint64_t entryAt = MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        const uint8_t* entry = c->bytes + entryAt;
        uint32_t tag = maniLoadU32(entry);
        if (tag == 0 || tag > MANI_INFLUENCE_SECTIONS)
        {
            return Fail(entryAt);
        }
        uint64_t size = maniLoadU64(entry + 16);
        if (size == 0)
        {
            return Fail(entryAt + 16);
        }
        s->data[tag - 1] = c->bytes + maniLoadU64(entry + 8);
        s->sizes[tag - 1] = size;
        s->at[tag - 1] = maniLoadU64(entry + 8);
        s->entry[tag - 1] = entryAt + 16;
    }
    return (maniFileResult){mani_success, 0};
}

// The binding's counts, and every section's size against them.
static maniFileResult CheckSizes(const Sections* s, maniInfluences* v)
{
    if (s->sizes[0] != MANI_INFLUENCE_BINDING_SIZE)
    {
        return Fail(s->entry[0]);
    }
    const uint8_t* b = s->data[0];
    v->skinIdentity = maniLoadU64(b);
    v->skinJointCount = maniLoadU32(b + 8);
    v->vertexCount = maniLoadU32(b + 12);
    v->groupCount = maniLoadU32(b + 16);
    if (v->skinJointCount == 0 || v->skinJointCount > MANI_MAX_BONES)
    {
        return Fail(s->at[0] + 8);
    }
    if (v->vertexCount == 0)
    {
        return Fail(s->at[0] + 12);
    }
    if (v->groupCount == 0 || v->groupCount > MANI_INFLUENCE_MAX_GROUPS)
    {
        return Fail(s->at[0] + 16);
    }
    if (maniLoadU32(b + 20) != 0)
    {
        return Fail(s->at[0] + 20);
    }
    uint64_t slots = (uint64_t)v->vertexCount * v->groupCount * MANI_INFLUENCE_GROUP;
    const uint64_t want[MANI_INFLUENCE_SECTIONS - 1] = {
        2 * slots, 4 * slots, 4 * (uint64_t)v->vertexCount, 12 * (uint64_t)v->vertexCount};
    for (uint32_t i = 1; i < MANI_INFLUENCE_SECTIONS; ++i)
    {
        // The optional sections (tags 4 and 5) may be absent.
        bool optional = i >= 3 && s->sizes[i] == 0;
        if (!optional && s->sizes[i] != want[i - 1])
        {
            return Fail(s->entry[i]);
        }
    }
    return (maniFileResult){mani_success, 0};
}

// Every joint below the skin's count, every weight from 0 to 1, the
// dual quaternion weights from 0 to 1, the centers finite.
static maniFileResult CheckValues(const Sections* s, const maniInfluences* v)
{
    uint64_t slots = s->sizes[1] / 2;
    for (uint64_t k = 0; k < slots; ++k)
    {
        if (maniLoadU16(s->data[1] + 2 * k) >= v->skinJointCount)
        {
            return Fail(s->at[1] + 2 * k);
        }
        if (!Unit(LoadF32(s->data[2] + 4 * k)))
        {
            return Fail(s->at[2] + 4 * k);
        }
    }
    for (uint64_t k = 0; k < s->sizes[3] / 4; ++k)
    {
        if (!Unit(LoadF32(s->data[3] + 4 * k)))
        {
            return Fail(s->at[3] + 4 * k);
        }
    }
    for (uint64_t k = 0; k < s->sizes[4] / 4; ++k)
    {
        if (!Finite(LoadF32(s->data[4] + 4 * k)))
        {
            return Fail(s->at[4] + 4 * k);
        }
    }
    return (maniFileResult){mani_success, 0};
}

static MANI_NOINLINE maniFileResult OpenInfluencesWork(const void* bytes, size_t size,
                                                       maniInfluences* influencesOut)
{
    if (influencesOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0};
    }
    maniContainer c;
    maniFileResult r =
        maniOpenContainer(bytes, size, maniKindInfluences, MANI_INFLUENCE_FORMAT_VERSION, &c);
    Sections s;
    if (r.result == mani_success)
    {
        r = LoadSections(&c, &s);
    }
    maniInfluences v;
    memset(&v, 0, sizeof v);
    if (r.result == mani_success)
    {
        r = CheckSizes(&s, &v);
    }
    if (r.result == mani_success)
    {
        r = CheckValues(&s, &v);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniInfluenceComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    v.joints = s.data[1];
    v.weights = s.data[2];
    v.dualQuaternionWeights = s.sizes[3] > 0 ? s.data[3] : NULL;
    v.centers = s.sizes[4] > 0 ? s.data[4] : NULL;
    v.identity = c.identity;
    *influencesOut = v;
    return (maniFileResult){mani_success, 0};
}

maniFileResult maniOpenInfluences(const void* bytes, size_t size, maniInfluences* influencesOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = OpenInfluencesWork(bytes, size, influencesOut);
    maniFloatEnvLeave(env);
    return result;
}

uint32_t maniInfluencesVertexCount(const maniInfluences* influences)
{
    return influences->vertexCount;
}

uint32_t maniInfluencesPerVertex(const maniInfluences* influences)
{
    return influences->groupCount * MANI_INFLUENCE_GROUP;
}

bool maniInfluencesFitSkin(const maniInfluences* influences, const maniSkin* skin)
{
    return influences != NULL && skin != NULL &&
           influences->skinIdentity == maniSkinIdentity(skin) &&
           influences->skinJointCount == maniSkinJointCount(skin);
}

uint32_t maniInfluence(const maniInfluences* influences, uint32_t vertex, uint32_t slot,
                       float* weightOut)
{
    uint32_t per = influences->groupCount * MANI_INFLUENCE_GROUP;
    bool inside = vertex < influences->vertexCount && slot < per;
    size_t k = inside ? (size_t)vertex * per + slot : 0;
    if (weightOut != NULL)
    {
        *weightOut = inside ? LoadF32(influences->weights + 4 * k) : 0.0f;
    }
    return inside ? maniLoadU16(influences->joints + 2 * k) : 0;
}

float maniInfluencesDualQuaternionWeight(const maniInfluences* influences, uint32_t vertex)
{
    if (influences->dualQuaternionWeights == NULL || vertex >= influences->vertexCount)
    {
        return 0.0f;
    }
    return LoadF32(influences->dualQuaternionWeights + 4 * (size_t)vertex);
}

bool maniInfluencesCenter(const maniInfluences* influences, uint32_t vertex, maniVec3* centerOut)
{
    if (influences->centers == NULL || vertex >= influences->vertexCount || centerOut == NULL)
    {
        return false;
    }
    const uint8_t* c = influences->centers + 12 * (size_t)vertex;
    *centerOut = (maniVec3){LoadF32(c), LoadF32(c + 4), LoadF32(c + 8)};
    return true;
}
