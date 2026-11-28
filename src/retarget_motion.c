// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion through a retargeting map: followers and
// moves carried between the bones' parents' reference frames.

#include "retarget_motion.h"

#include "bytes.h"
#include "retarget_file.h"
#include "retarget_view.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/retarget.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static maniQuat LoadQuat(const uint8_t* p)
{
    return (maniQuat){LoadF32(p), LoadF32(p + 4), LoadF32(p + 8), LoadF32(p + 12)};
}

uint32_t maniRetargetFollower(const maniRetargetMap* map, uint32_t a)
{
    for (uint32_t t = 0; t < map->targetBoneCount; ++t)
    {
        const uint8_t* e = maniRetargetPlan(map, t);
        if (maniLoadU32(e + maniRetargetPlanKind) == maniRetargetPlanOne &&
            maniLoadU32(e + maniRetargetPlanA) == a)
        {
            return t;
        }
    }
    return MANI_NO_BONE;
}

maniRetargetMotion maniRetargetMotionOf(const maniRetargetMap* map, const maniSkeleton* source,
                                        uint32_t sourceBone, const maniSkeleton* target,
                                        uint32_t targetBone)
{
    uint32_t parentS = maniSkeleton_GetParent(source, sourceBone);
    uint32_t parentT = maniSkeleton_GetParent(target, targetBone);
    maniRetargetMotion m = {map, maniQuatIdentity(), maniQuatIdentity(), maniQuatIdentity()};
    // A view no open wrote, as a graph's bound map may have become, carries
    // no turn; so does a parent past its count, as of a smaller map put in
    // the bound one's place.
    bool valid = maniRetargetMapValid(map);
    if (valid && parentS < map->sourceBoneCount)
    {
        m.sourceFrame = LoadQuat(maniRetargetSourceBone(map, parentS));
    }
    if (valid && parentT < map->targetBoneCount)
    {
        m.targetFrame = LoadQuat(maniRetargetTargetBone(map, parentT));
    }
    m.turn = maniQuatMul(maniQuatConjugate(m.targetFrame), m.sourceFrame);
    return m;
}

maniVec3 maniRetargetMotionTranslation(const maniRetargetMotion* motion, maniVec3 t)
{
    maniVec3 model =
        maniRetargetMap_RetargetRootMotion(motion->map, maniQuatRotate(motion->sourceFrame, t));
    return maniQuatRotate(maniQuatConjugate(motion->targetFrame), model);
}

maniRootMove maniRetargetMove(const maniRetargetMotion* motion, maniRootMove move)
{
    maniQuat r = maniQuatMul(maniQuatMul(motion->turn, maniQuatNormalize(move.rotation)),
                             maniQuatConjugate(motion->turn));
    return (maniRootMove){maniRetargetMotionTranslation(motion, move.translation),
                          maniQuatNormalize(r)};
}
