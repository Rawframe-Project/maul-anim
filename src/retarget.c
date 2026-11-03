// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a retargeting map: the container, the header (bone counts),
// both sides' references (finite, rotations of length 1), the pelvis
// when there is one (its bones and numbers), each target bone's plan in
// its one form, and the identity. Whether the plan
// fits the skeletons (chains being paths, the pelvis's heights) needs
// them: their identities, checked where the map is used, stand for the
// builder's checks.

#include "maul-anim/retarget.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "retarget_file.h"

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint64_t maniRetargetComputeIdentity(const uint8_t* const data[MANI_RETARGET_SECTIONS],
                                     const uint64_t sizes[MANI_RETARGET_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_RETARGET_SECTIONS; ++i)
    {
        hash = maniHashBytes(hash, data[i], sizes[i]);
    }
    return hash;
}

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static const maniFileResult kOk = {mani_success, 0, 0};

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

static bool Share(float v)
{
    return v >= 0.0f && v <= 1.0f;
}

// Within a thousandth of length 1, as the builder takes it.
static bool Unit(float lengthSquared)
{
    return lengthSquared >= 0.998f && lengthSquared <= 1.002f;
}

// The first of count binary32 values at p not finite, or count.
static uint32_t FirstNotFinite(const uint8_t* p, uint32_t count)
{
    uint32_t k = 0;
    while (k < count && Finite(LoadF32(p + 4 * k)))
    {
        ++k;
    }
    return k;
}

// The offset of the first wrong byte of a unit quaternion at p (at), or
// 0 when it is one.
static uint64_t QuatWrong(const uint8_t* p, uint64_t at)
{
    uint32_t k = FirstNotFinite(p, 4);
    if (k < 4)
    {
        return at + 4 * k;
    }
    float x = LoadF32(p);
    float y = LoadF32(p + 4);
    float z = LoadF32(p + 8);
    float w = LoadF32(p + 12);
    return Unit(x * x + y * y + z * z + w * w) ? 0u : at;
}

typedef struct Sections
{
    const uint8_t* data[MANI_RETARGET_SECTIONS];
    uint64_t sizes[MANI_RETARGET_SECTIONS];
    uint64_t at[MANI_RETARGET_SECTIONS];
} Sections;

// A map without a pelvis: MANI_NO_BONE (0xFFFF) both, the axis +y, the
// scales and the ratio 1 (binary32 0x3F800000), the rest 0; little-endian.
static const uint8_t kNoPelvis[MANI_RETARGET_PELVIS_SIZE] = {
    [maniRetargetPelvisSourceBone] = 0xFF,     [maniRetargetPelvisSourceBone + 1] = 0xFF,
    [maniRetargetPelvisTargetBone] = 0xFF,     [maniRetargetPelvisTargetBone + 1] = 0xFF,
    [maniRetargetPelvisUp + 6] = 0x80,         [maniRetargetPelvisUp + 7] = 0x3F,
    [maniRetargetPelvisHorizontal + 2] = 0x80, [maniRetargetPelvisHorizontal + 3] = 0x3F,
    [maniRetargetPelvisVertical + 2] = 0x80,   [maniRetargetPelvisVertical + 3] = 0x3F,
    [maniRetargetPelvisRatio + 2] = 0x80,      [maniRetargetPelvisRatio + 3] = 0x3F,
};

// The pelvis section, when there is one: its size, its bones within the
// counts, its numbers finite and in range. Its tag is the last, so its
// size is in the table's last entry, of count.
static maniFileResult Pelvis(const Sections* s, uint32_t count, const maniRetargetMap* m)
{
    const uint8_t* h = s->data[4];
    uint64_t at = s->at[4];
    if (s->sizes[4] != MANI_RETARGET_PELVIS_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + (uint64_t)(count - 1) * MANI_CONTAINER_ENTRY_SIZE +
                    16);
    }
    if (maniLoadU32(h + maniRetargetPelvisSourceBone) >= m->sourceBoneCount ||
        maniLoadU32(h + maniRetargetPelvisTargetBone) >= m->targetBoneCount)
    {
        return Fail(at + maniRetargetPelvisSourceBone);
    }
    // The axis, the two scales, the 13 numbers after them and the ratio.
    uint32_t k = FirstNotFinite(h + maniRetargetPelvisUp, 19);
    if (k < 19)
    {
        return Fail(at + maniRetargetPelvisUp + 4 * k);
    }
    float x = LoadF32(h + maniRetargetPelvisUp);
    float y = LoadF32(h + maniRetargetPelvisUp + 4);
    float z = LoadF32(h + maniRetargetPelvisUp + 8);
    if (!Unit(x * x + y * y + z * z))
    {
        return Fail(at + maniRetargetPelvisUp);
    }
    for (uint32_t w = 0; w < 2; ++w)
    {
        if (!(LoadF32(h + maniRetargetPelvisHorizontal + 4 * w) >= 0.0f))
        {
            return Fail(at + maniRetargetPelvisHorizontal + 4 * w);
        }
    }
    // The blends and the weight in [0, 1]; the positions, any finite
    // values.
    for (uint32_t w = 0; w < 7; ++w)
    {
        if (!Share(LoadF32(h + maniRetargetPelvisAbsolute + 4 * w)))
        {
            return Fail(at + maniRetargetPelvisAbsolute + 4 * w);
        }
    }
    return LoadF32(h + maniRetargetPelvisRatio) > 0.0f ? kOk : Fail(at + maniRetargetPelvisRatio);
}

static maniFileResult Header(const Sections* s, maniRetargetMap* m)
{
    const uint8_t* h = s->data[0];
    uint64_t at = s->at[0];
    if (s->sizes[0] != MANI_RETARGET_HEADER_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE);
    }
    m->sourceIdentity = maniLoadU64(h + maniRetargetHeaderSourceIdentity);
    m->targetIdentity = maniLoadU64(h + maniRetargetHeaderTargetIdentity);
    m->sourceBoneCount = maniLoadU32(h + maniRetargetHeaderSourceBones);
    m->targetBoneCount = maniLoadU32(h + maniRetargetHeaderTargetBones);
    if (m->sourceBoneCount < 1 || m->sourceBoneCount > MANI_MAX_BONES)
    {
        return Fail(at + maniRetargetHeaderSourceBones);
    }
    if (m->targetBoneCount < 1 || m->targetBoneCount > MANI_MAX_BONES)
    {
        return Fail(at + maniRetargetHeaderTargetBones);
    }
    return kOk;
}

// The references: a rotation (or two) of length 1, the rest finite.
static maniFileResult References(const Sections* s, const maniRetargetMap* m)
{
    if (s->sizes[1] != (uint64_t)m->sourceBoneCount * MANI_RETARGET_SOURCE_SIZE)
    {
        return Fail(s->at[0] + maniRetargetHeaderSourceBones);
    }
    if (s->sizes[2] != (uint64_t)m->targetBoneCount * MANI_RETARGET_TARGET_SIZE)
    {
        return Fail(s->at[0] + maniRetargetHeaderTargetBones);
    }
    for (uint32_t side = 1; side <= 2; ++side)
    {
        uint32_t count = side == 1 ? m->sourceBoneCount : m->targetBoneCount;
        uint32_t size = side == 1 ? MANI_RETARGET_SOURCE_SIZE : MANI_RETARGET_TARGET_SIZE;
        uint32_t quats = side == 1 ? 1u : 2u;
        for (uint32_t b = 0; b < count; ++b)
        {
            const uint8_t* e = s->data[side] + (size_t)b * size;
            uint64_t at = s->at[side] + (uint64_t)b * size;
            for (uint32_t q = 0; q < quats; ++q)
            {
                uint64_t wrong = QuatWrong(e + 16 * q, at + 16 * q);
                if (wrong != 0)
                {
                    return Fail(wrong);
                }
            }
            uint32_t k = FirstNotFinite(e + 16 * quats, 6);
            if (k < 6)
            {
                return Fail(at + 16 * quats + 4 * k);
            }
        }
    }
    return kOk;
}

// A plan record in its one form: the offset of its first wrong byte
// from the record's start, or UINT32_MAX.
static uint32_t PlanWrong(const uint8_t* e, const maniRetargetMap* m, uint32_t bone,
                          uint32_t pelvisTarget, uint32_t pelvisSource)
{
    uint32_t kind = maniLoadU32(e + maniRetargetPlanKind);
    uint32_t a = maniLoadU32(e + maniRetargetPlanA);
    uint32_t b = maniLoadU32(e + maniRetargetPlanB);
    float t = LoadF32(e + maniRetargetPlanT);
    float weight = LoadF32(e + maniRetargetPlanWeight);
    uint32_t translation = maniLoadU32(e + maniRetargetPlanTranslation);
    uint32_t flags = maniLoadU32(e + maniRetargetPlanFlags);
    bool reference = kind == maniRetargetPlanReference;
    bool two = kind == maniRetargetPlanTwo;
    bool pelvis = bone == pelvisTarget;
    const bool ok[7] = {
        kind <= maniRetargetPlanTwo && (!pelvis || kind == maniRetargetPlanOne),
        reference ? a == MANI_NO_BONE : a < m->sourceBoneCount && (!pelvis || a == pelvisSource),
        two ? b < m->sourceBoneCount && b != a : b == MANI_NO_BONE,
        two ? t > 0.0f && t < 1.0f : t == 0.0f && !signbit(t),
        reference ? weight == 0.0f && !signbit(weight) : Share(weight),
        pelvis ? translation == maniRetargetPlanPelvis
               : translation <= (reference ? 0u : (uint32_t)maniRetargetTranslationAbsolute),
        flags <= (reference || pelvis ? 0u
                  : kind == maniRetargetPlanOne && weight == 1.0f
                      ? MANI_RETARGET_PLAN_SCALE | MANI_RETARGET_PLAN_SAME
                      : MANI_RETARGET_PLAN_SCALE)};
    for (uint32_t k = 0; k < 7; ++k)
    {
        if (!ok[k])
        {
            return 4 * k;
        }
    }
    return UINT32_MAX;
}

static maniFileResult Plans(const Sections* s, const maniRetargetMap* m)
{
    if (s->sizes[3] != (uint64_t)m->targetBoneCount * MANI_RETARGET_PLAN_SIZE)
    {
        return Fail(s->at[0] + maniRetargetHeaderTargetBones);
    }
    uint32_t pelvisSource = maniLoadU32(m->pelvis + maniRetargetPelvisSourceBone);
    uint32_t pelvisTarget = maniLoadU32(m->pelvis + maniRetargetPelvisTargetBone);
    for (uint32_t b = 0; b < m->targetBoneCount; ++b)
    {
        const uint8_t* e = s->data[3] + (size_t)b * MANI_RETARGET_PLAN_SIZE;
        uint32_t wrong = PlanWrong(e, m, b, pelvisTarget, pelvisSource);
        if (wrong != UINT32_MAX)
        {
            return Fail(s->at[3] + (uint64_t)b * MANI_RETARGET_PLAN_SIZE + wrong);
        }
    }
    return kOk;
}

static MANI_NOINLINE maniFileResult OpenRetargetMapWork(const void* bytes, size_t size, void* out)
{
    maniRetargetMap* mapOut = out;
    if (mapOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindRetargetMap,
                                         MANI_RETARGET_FORMAT_VERSION, MANI_RETARGET_SECTIONS, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    static const uint8_t kEmpty = 0;
    Sections s;
    for (uint32_t t = 1; t <= MANI_RETARGET_SECTIONS; ++t)
    {
        s.data[t - 1] = &kEmpty;
        s.sizes[t - 1] = 0;
        s.at[t - 1] = 0;
        (void)maniContainerSection(&c, t, &s.data[t - 1], &s.sizes[t - 1], &s.at[t - 1]);
    }
    maniRetargetMap map;
    memset(&map, 0, sizeof map);
    map.pelvis = s.sizes[4] != 0 ? s.data[4] : kNoPelvis;
    r = Header(&s, &map);
    r = r.result == mani_success ? References(&s, &map) : r;
    r = r.result == mani_success && s.sizes[4] != 0 ? Pelvis(&s, c.sectionCount, &map) : r;
    r = r.result == mani_success ? Plans(&s, &map) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniRetargetComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    map.sourceBones = s.data[1];
    map.targetBones = s.data[2];
    map.plan = s.data[3];
    map.identity = c.identity;
    *mapOut = map;
    return kOk;
}

maniFileResult maniOpenRetargetMap(const void* bytes, size_t size, maniRetargetMap* mapOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = maniContainerOpenTagged(bytes, size, mapOut, OpenRetargetMapWork);
    maniFloatEnvLeave(env);
    return result;
}

uint64_t maniRetargetMap_GetIdentity(const maniRetargetMap* map)
{
    return map != NULL ? map->identity : 0;
}

uint64_t maniRetargetMap_GetSourceIdentity(const maniRetargetMap* map)
{
    return map != NULL ? map->sourceIdentity : 0;
}

uint64_t maniRetargetMap_GetTargetIdentity(const maniRetargetMap* map)
{
    return map != NULL ? map->targetIdentity : 0;
}
