// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "spring_frame.h"

#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

maniVec3 maniSpringRestDirection(const maniSkeleton* skeleton, uint32_t head, uint32_t tail)
{
    maniTransform t = maniSkeletonRest(skeleton, tail);
    maniAffine m = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    for (uint32_t b = maniSkeletonParent(skeleton, tail); b != head && b != MANI_NO_BONE;
         b = maniSkeletonParent(skeleton, b))
    {
        maniTransform r = maniSkeletonRest(skeleton, b);
        maniAffine p = maniAffineFromTrs(r.translation, r.rotation, r.scale);
        m = maniAffineMul(&p, &m);
    }
    maniVec3 at = maniAffineTransformPoint(&m, maniVec3Make(0.0f, 0.0f, 0.0f));
    return maniVec3Normalize(at, maniVec3Make(0.0f, 1.0f, 0.0f));
}

maniQuat maniSpringTurnFromY(maniVec3 axis)
{
    return axis.y == -1.0f ? (maniQuat){1.0f, 0.0f, 0.0f, 0.0f}
                           : maniQuatNormalize((maniQuat){axis.z, 0.0f, -axis.x, axis.y + 1.0f});
}
