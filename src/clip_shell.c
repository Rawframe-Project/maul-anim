// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The shell metric in binary64, so measuring adds no error
// of its own to what compression made.

#include "clip_shell.h"

#include "additive.h"

#include <math.h>

// A local T R S as a 3x4 matrix: rotation columns scaled, then the
// translation.
maniShellMatrix maniShellLocal(maniTransform t)
{
    double x = (double)t.rotation.x;
    double y = (double)t.rotation.y;
    double z = (double)t.rotation.z;
    double w = (double)t.rotation.w;
    double r[9] = {
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z),       2.0 * (x * z - w * y),
        2.0 * (x * y - w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x),
        2.0 * (x * z + w * y),       2.0 * (y * z - w * x),       1.0 - 2.0 * (x * x + y * y),
    };
    double s[3] = {(double)t.scale.x, (double)t.scale.y, (double)t.scale.z};
    maniShellMatrix m;
    for (int c = 0; c < 3; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            m.m[3 * c + k] = r[3 * c + k] * s[c];
        }
    }
    m.m[9] = (double)t.translation.x;
    m.m[10] = (double)t.translation.y;
    m.m[11] = (double)t.translation.z;
    return m;
}

// a times b.
maniShellMatrix maniShellMul(const maniShellMatrix* a, const maniShellMatrix* b)
{
    maniShellMatrix out;
    for (int c = 0; c < 4; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            double v = a->m[k] * b->m[3 * c] + a->m[3 + k] * b->m[3 * c + 1] +
                       a->m[6 + k] * b->m[3 * c + 2];
            out.m[3 * c + k] = c == 3 ? v + a->m[9 + k] : v;
        }
    }
    return out;
}

maniTransform maniShellAbsolute(const maniSkeleton* skeleton, uint32_t bone, bool additive,
                                maniTransform t)
{
    if (!additive)
    {
        return t;
    }
    maniTransform rest = maniSkeletonRest(skeleton, bone);
    maniAddTransform(&rest, &t, 1.0f);
    return rest;
}

void maniShellModels(const maniSkeleton* skeleton, const maniPose* pose, maniShellMatrix* out)
{
    uint32_t n = maniSkeletonBoneCount(skeleton);
    bool additive = maniPoseIsDifference(pose);
    for (uint32_t b = 0; b < n; ++b)
    {
        maniShellMatrix local =
            maniShellLocal(maniShellAbsolute(skeleton, b, additive, maniPoseLocal(pose, b)));
        uint32_t parent = maniSkeletonParent(skeleton, b);
        out[b] = parent == MANI_NO_BONE ? local : maniShellMul(&out[parent], &local);
    }
}

double maniShellError(const maniShellMatrix* a, const maniShellMatrix* b, double distance)
{
    double worst = 0.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        double squared = 0.0;
        for (int k = 0; k < 3; ++k)
        {
            double pa = a->m[3 * axis + k] * distance + a->m[9 + k];
            double pb = b->m[3 * axis + k] * distance + b->m[9 + k];
            squared += (pa - pb) * (pa - pb);
        }
        double reach = sqrt(squared);
        worst = reach > worst ? reach : worst;
    }
    return worst;
}
