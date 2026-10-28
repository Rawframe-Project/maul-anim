// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a mirror map: the container, the header, each
// bone's partner (pairing back) and corrections (finite, of length 1),
// the name pairs (by increasing hash, each partner's name inside the
// names, UTF-8, of its hash), the identity; and the map's readers.
// Whether the partners suit the skeleton's names needs the skeleton: its
// identity, checked where the map is used, stands for the builder's
// checks.

#include "maul-anim/mirror.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "mirror_file.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint64_t maniMirrorComputeIdentity(const uint8_t* const data[MANI_MIRROR_SECTIONS],
                                   const uint64_t sizes[MANI_MIRROR_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_MIRROR_SECTIONS; ++i)
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

// The offset of the first wrong byte of a quaternion at p (at): finite
// and within a thousandth of length 1, as the builder writes it; 0 when
// it is one.
static uint64_t QuatWrong(const uint8_t* p, uint64_t at)
{
    double length = 0.0;
    for (uint32_t k = 0; k < 4; ++k)
    {
        float v = LoadF32(p + 4 * k);
        if (!(fabsf(v) < HUGE_VALF))
        {
            return at + 4 * k;
        }
        length += (double)v * (double)v;
    }
    return length >= 0.998 && length <= 1.002 ? 0u : at;
}

typedef struct Sections
{
    const uint8_t* data[MANI_MIRROR_SECTIONS];
    uint64_t sizes[MANI_MIRROR_SECTIONS];
    uint64_t at[MANI_MIRROR_SECTIONS];
} Sections;

static maniFileResult Header(const Sections* s, maniMirrorMap* m)
{
    if (s->sizes[0] != MANI_MIRROR_HEADER_SIZE)
    {
        return Fail(s->at[0]);
    }
    const uint8_t* h = s->data[0];
    uint64_t at = s->at[0];
    m->skeletonIdentity = maniLoadU64(h);
    m->boneCount = maniLoadU32(h + 8);
    m->axis = maniLoadU32(h + 12);
    m->nameCount = maniLoadU32(h + 16);
    uint32_t unmatched = maniLoadU32(h + 20);
    if (m->boneCount == 0 || m->boneCount > MANI_MAX_BONES)
    {
        return Fail(at + 8);
    }
    if (m->axis > maniMirrorAxisZ)
    {
        return Fail(at + 12);
    }
    if (unmatched != MANI_NO_BONE && unmatched >= m->boneCount)
    {
        return Fail(at + 20);
    }
    for (uint32_t k = 24; k < MANI_MIRROR_HEADER_SIZE; k += 4)
    {
        if (maniLoadU32(h + k) != 0)
        {
            return Fail(at + k);
        }
    }
    return kOk;
}

// A bone's flags: known bits only, a permutation or the approximate flag
// alone, the header's first unmatched bone the first flagged unmatched.
static bool FlagsValid(uint32_t flags, uint32_t bone, uint32_t firstUnmatched)
{
    uint32_t known =
        MANI_MIRROR_PERMUTATION | MANI_MIRROR_FLAG_APPROXIMATE | MANI_MIRROR_FLAG_UNMATCHED;
    if ((flags & ~known) != 0 ||
        ((flags & MANI_MIRROR_FLAG_APPROXIMATE) != 0 && (flags & MANI_MIRROR_PERMUTATION) != 0))
    {
        return false;
    }
    uint32_t used = 0;
    for (uint32_t i = 0; i < 3; ++i)
    {
        uint32_t source = ((flags >> (2 * i)) & 3u) ^ i;
        used |= source < 3 ? 1u << source : 8u;
    }
    bool unmatched = (flags & MANI_MIRROR_FLAG_UNMATCHED) != 0;
    bool firstOrLater = bone >= firstUnmatched;
    return used == 7u && (bone != firstUnmatched || unmatched) && (firstOrLater || !unmatched);
}

static maniFileResult Bones(const Sections* s, const maniMirrorMap* m)
{
    if (s->sizes[1] != (uint64_t)m->boneCount * MANI_MIRROR_BONE_SIZE)
    {
        return Fail(s->at[0] + 8);
    }
    for (uint32_t b = 0; b < m->boneCount; ++b)
    {
        const uint8_t* e = s->data[1] + (size_t)b * MANI_MIRROR_BONE_SIZE;
        uint64_t at = s->at[1] + (uint64_t)b * MANI_MIRROR_BONE_SIZE;
        uint32_t partner = maniLoadU32(e);
        if (partner >= m->boneCount ||
            maniLoadU32(s->data[1] + (size_t)partner * MANI_MIRROR_BONE_SIZE) != b)
        {
            return Fail(at);
        }
        if (!FlagsValid(maniLoadU32(e + 4), b, maniLoadU32(s->data[0] + 20)))
        {
            return Fail(at + 4);
        }
        uint64_t wrong = QuatWrong(e + 8, at + 8);
        wrong = wrong != 0 ? wrong : QuatWrong(e + 24, at + 24);
        if (wrong != 0)
        {
            return Fail(wrong);
        }
    }
    return kOk;
}

static maniFileResult Names(const Sections* s, const maniMirrorMap* m)
{
    if (s->sizes[2] != (uint64_t)m->nameCount * MANI_MIRROR_NAME_SIZE)
    {
        return Fail(s->at[0] + 16);
    }
    for (uint32_t i = 0; i < m->nameCount; ++i)
    {
        const uint8_t* e = s->data[2] + (size_t)i * MANI_MIRROR_NAME_SIZE;
        uint64_t at = s->at[2] + (uint64_t)i * MANI_MIRROR_NAME_SIZE;
        if (i > 0 && maniLoadU64(e) <= maniLoadU64(e - MANI_MIRROR_NAME_SIZE))
        {
            return Fail(at);
        }
        uint64_t offset = maniLoadU32(e + 16);
        uint64_t length = maniLoadU32(e + 20);
        if (length == 0 || offset > s->sizes[3] || length > s->sizes[3] - offset)
        {
            return Fail(at + 16);
        }
        const uint8_t* name = s->data[3] + offset;
        if (!maniValidUtf8(name, (size_t)length) ||
            maniHashBytes(MANI_HASH_INIT, name, length) != maniLoadU64(e + 8))
        {
            return Fail(at + 8);
        }
    }
    return kOk;
}

static MANI_NOINLINE maniFileResult OpenMirrorMapWork(const void* bytes, size_t size, void* out)
{
    maniMirrorMap* mapOut = out;
    if (mapOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r =
        maniOpenContainer(bytes, size, maniKindMirrorMap, MANI_MIRROR_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    static const uint8_t kEmpty = 0;
    Sections s;
    for (uint32_t t = 1; t <= MANI_MIRROR_SECTIONS; ++t)
    {
        s.data[t - 1] = &kEmpty;
        s.sizes[t - 1] = 0;
        s.at[t - 1] = 0;
        (void)maniContainerSection(&c, t, &s.data[t - 1], &s.sizes[t - 1], &s.at[t - 1]);
    }
    maniMirrorMap map;
    memset(&map, 0, sizeof map);
    r = Header(&s, &map);
    r = r.result == mani_success ? Bones(&s, &map) : r;
    r = r.result == mani_success ? Names(&s, &map) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniMirrorComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    map.header = s.data[0];
    map.bones = s.data[1];
    map.names = s.data[2];
    map.strings = s.data[3];
    map.stringsSize = s.sizes[3];
    map.identity = c.identity;
    *mapOut = map;
    return kOk;
}

maniFileResult maniOpenMirrorMap(const void* bytes, size_t size, maniMirrorMap* mapOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = maniContainerOpenTagged(bytes, size, mapOut, OpenMirrorMapWork);
    maniFloatEnvLeave(env);
    return result;
}

uint64_t maniMirrorMapIdentity(const maniMirrorMap* map)
{
    return map != NULL ? map->identity : 0;
}

uint64_t maniMirrorMapSkeletonIdentity(const maniMirrorMap* map)
{
    return map != NULL ? map->skeletonIdentity : 0;
}

maniMirrorAxis maniMirrorMapAxis(const maniMirrorMap* map)
{
    return map != NULL ? map->axis : maniMirrorAxisX;
}

uint32_t maniMirrorPartner(const maniMirrorMap* map, uint32_t bone)
{
    if (map == NULL || bone >= map->boneCount)
    {
        return MANI_NO_BONE;
    }
    return maniLoadU32(map->bones + (size_t)bone * MANI_MIRROR_BONE_SIZE);
}

uint32_t maniMirrorBoneFlags(const maniMirrorMap* map, uint32_t bone)
{
    if (map == NULL || bone >= map->boneCount)
    {
        return 0;
    }
    uint32_t flags = maniLoadU32(map->bones + (size_t)bone * MANI_MIRROR_BONE_SIZE + 4);
    return ((flags & MANI_MIRROR_FLAG_UNMATCHED) != 0 ? MANI_MIRROR_BONE_UNMATCHED : 0u) |
           ((flags & MANI_MIRROR_FLAG_APPROXIMATE) != 0 ? MANI_MIRROR_BONE_SCALE_APPROXIMATE : 0u);
}

uint32_t maniMirrorMapUnmatchedBone(const maniMirrorMap* map)
{
    return map != NULL ? maniLoadU32(map->header + 20) : MANI_NO_BONE;
}

// The name record of a hash; NULL for none.
static const uint8_t* Find(const maniMirrorMap* map, uint64_t hash)
{
    uint32_t low = 0;
    uint32_t high = map->nameCount;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        const uint8_t* e = map->names + (size_t)mid * MANI_MIRROR_NAME_SIZE;
        uint64_t at = maniLoadU64(e);
        if (at == hash)
        {
            return e;
        }
        if (at < hash)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    return NULL;
}

const char* maniMirrorName(const maniMirrorMap* map, const char* name, size_t length,
                           size_t* lengthOut)
{
    if (lengthOut != NULL)
    {
        *lengthOut = length;
    }
    if (map == NULL || (name == NULL && length > 0))
    {
        return name;
    }
    const uint8_t* e = Find(map, maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length));
    if (e == NULL)
    {
        return name;
    }
    if (lengthOut != NULL)
    {
        *lengthOut = maniLoadU32(e + 20);
    }
    return (const char*)map->strings + maniLoadU32(e + 16);
}

uint64_t maniMirrorNameHash(const maniMirrorMap* map, uint64_t hash)
{
    const uint8_t* e = map != NULL ? Find(map, hash) : NULL;
    return e != NULL ? maniLoadU64(e + 8) : hash;
}
