// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body solve by selectively damped least squares (Buss and
// Kim, 2005; mani-0031). Columns: three rotation axes (model x, y, z at
// the bone's origin) per part bone, zero for a held top or a locked
// bone, scaled by one minus its rotation stiffness; three translations
// for a free top. Rows per effector: its position miss and, with a
// rotation weight, its rotation miss (a rotation vector times the
// effector's reach, the summed offsets from the top, so both are
// lengths), each clamped to half the mean bone length, scaled by the
// strength. Each iteration: the Jacobian's transpose decomposed by
// one-sided Jacobi, then the paper's step: for each singular triple,
// gamma_i = min(1, N_i / M_i) pi/4 and phi_i = ClampMaxAbs(alpha_i /
// sigma_i v_i, gamma_i); the sum clamped to pi/4 in its rotations (the
// paper's hard clamps, where Buss's code scales every direction down).
// The step is taken from the tips up, each bone's turn turning the
// bones below it; then limits from the top down, a limited bone's
// change carried to the bones below it.

#include "body_ik.h"
#include "jacobi_svd.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GAMMA_MAX 0.785398163f

// The solver's floats in the part's room: A (the Jacobian's transpose,
// n by m, column by column), W (m by m), the singular values and the
// misses (m each), the step, the columns' scales and their responses
// (n each).
typedef struct Sdls
{
    float* a;
    float* w;
    float* sigma;
    float* miss;
    float* step;
    float* scale;
    float* rho;
    float* phi;
    uint32_t n;
    uint32_t m;
} Sdls;

size_t maniBodySdlsRoom(uint32_t count, uint32_t effectors)
{
    size_t n = 3u * (size_t)count + 3u;
    size_t m = 6u * (size_t)effectors;
    return n * m + m * m + 2u * m + 4u * n;
}

static Sdls Lay(const maniBodyPart* part)
{
    Sdls s;
    s.n = 3u * part->count + 3u;
    s.m = 6u * part->effectors;
    s.a = part->room;
    s.w = s.a + (size_t)s.n * s.m;
    s.sigma = s.w + (size_t)s.m * s.m;
    s.miss = s.sigma + s.m;
    s.step = s.miss + s.m;
    s.scale = s.step + s.n;
    s.rho = s.scale + s.n;
    s.phi = s.rho + s.n;
    return s;
}

// Each column's scale: one minus the bone's rotation stiffness, 0 for a
// held top or a locked bone; 1 for a free top's translations, 0 else.
static void Scales(const maniBodyPart* part, const maniBodySolve* solve, const Sdls* s)
{
    for (uint32_t k = 0; k < part->count; ++k)
    {
        const maniBodyBone* setting = maniBodySetting(solve, &part->nodes[k]);
        float scale = setting != NULL ? 1.0f - setting->rotationStiffness : 1.0f;
        scale = k == 0 && solve->topMode != maniBodyTopFree ? 0.0f : scale;
        for (uint32_t axis = 0; axis < 3; ++axis)
        {
            s->scale[3u * k + axis] = scale;
            s->scale[3u * part->count + axis] = solve->topMode == maniBodyTopFree ? 1.0f : 0.0f;
        }
    }
}

// The summed offsets from the top to node k, no less than the mean
// offset (so an effector on the top still has rotation rows).
static float Reach(const maniBodyPart* part, uint32_t k, float mean)
{
    float reach = 0.0f;
    for (; k != 0; k = part->nodes[k].parent)
    {
        reach += maniVec3Length(part->nodes[k].offset);
    }
    return reach > mean ? reach : mean;
}

static maniVec3 Clamped(maniVec3 v, float most)
{
    float l = maniVec3Length(v);
    return l > most ? maniVec3Scale(v, most / l) : v;
}

// Effector e's misses, clamped and weighted, in its six rows.
static void Misses(const maniBodyPart* part, const maniBodySolve* solve, const Sdls* s, uint32_t e,
                   float most, float reach)
{
    const maniBodyEffector* effector = &solve->effectors[e];
    const maniBodyGoal* goal = &part->goals[e];
    const maniBodyNode* node = &part->nodes[goal->node];
    maniVec3 position = Clamped(maniVec3Sub(goal->position, node->origin), most);
    maniQuat miss = maniQuatMul(goal->rotation, maniQuatConjugate(maniBodyRotation(node)));
    miss = miss.w < 0.0f ? (maniQuat){-miss.x, -miss.y, -miss.z, -miss.w} : miss;
    maniVec3 rotation = effector->rotationWeight > 0.0f
                            ? Clamped(maniVec3Scale(maniQuatToRotationVector(miss), reach), most)
                            : maniVec3Make(0.0f, 0.0f, 0.0f);
    const float misses[6] = {position.x, position.y, position.z,
                             rotation.x, rotation.y, rotation.z};
    for (uint32_t r = 0; r < 6; ++r)
    {
        s->miss[6u * e + r] = misses[r] * effector->strength;
    }
}

// Effector e's rows: its misses, and its entries in A (row r of the
// Jacobian is A's column r).
static void Rows(const maniBodyPart* part, const maniBodySolve* solve, const Sdls* s, uint32_t e,
                 float most)
{
    const maniBodyGoal* goal = &part->goals[e];
    const maniBodyNode* node = &part->nodes[goal->node];
    float weight = solve->effectors[e].strength;
    float reach = Reach(part, goal->node, 2.0f * most);
    float turns = solve->effectors[e].rotationWeight > 0.0f ? reach : 0.0f;
    Misses(part, solve, s, e, most, reach);
    float* rows = s->a + (size_t)6u * e * s->n;
    for (uint32_t k = goal->node; k != MANI_BODY_NONE; k = part->nodes[k].parent)
    {
        maniVec3 arm = maniVec3Sub(node->origin, part->nodes[k].origin);
        const maniVec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        for (uint32_t axis = 0; axis < 3; ++axis)
        {
            uint32_t column = 3u * k + axis;
            float w = weight * s->scale[column];
            maniVec3 u = axes[axis];
            maniVec3 moved = maniVec3Scale(maniVec3Cross(u, arm), w);
            // The rotation rows: the axis times the reach, none without a
            // rotation weight.
            const float entries[6] = {moved.x,         moved.y,         moved.z,
                                      u.x * turns * w, u.y * turns * w, u.z * turns * w};
            for (uint32_t r = 0; r < 6; ++r)
            {
                rows[(size_t)r * s->n + column] = entries[r];
            }
        }
    }
    for (uint32_t axis = 0; axis < 3; ++axis)
    {
        uint32_t column = 3u * part->count + axis;
        rows[(size_t)axis * s->n + column] = weight * s->scale[column];
    }
}

// Each column's response: the lengths of its 3-row blocks summed, read
// before the decomposition turns A.
static void Responses(const Sdls* s)
{
    for (uint32_t j = 0; j < s->n; ++j)
    {
        float sum = 0.0f;
        for (uint32_t block = 0; block < s->m; block += 3)
        {
            const float* r = s->a + (size_t)block * s->n + j;
            float x = r[0];
            float y = r[s->n];
            float z = r[2u * (size_t)s->n];
            sum += sqrtf(x * x + y * y + z * z);
        }
        s->rho[j] = sum;
    }
}

// v scaled so its largest rotation entry (the first rotations entries)
// is at most most, as ClampMaxAbs.
static void ClampRotations(float* v, uint32_t rotations, uint32_t n, float most)
{
    float largest = 0.0f;
    for (uint32_t j = 0; j < rotations; ++j)
    {
        largest = fabsf(v[j]) > largest ? fabsf(v[j]) : largest;
    }
    if (largest > most)
    {
        float scale = most / largest;
        for (uint32_t j = 0; j < n; ++j)
        {
            v[j] = j < rotations ? v[j] * scale : v[j];
        }
    }
}

// The SDLS step from the decomposed A (columns sigma_i v_i), W (columns
// u_i) and the misses, into step.
static void Step(const Sdls* s, uint32_t rotations)
{
    float largest = 0.0f;
    for (uint32_t i = 0; i < s->m; ++i)
    {
        largest = s->sigma[i] > largest ? s->sigma[i] : largest;
    }
    for (uint32_t j = 0; j < s->n; ++j)
    {
        s->step[j] = 0.0f;
    }
    for (uint32_t i = 0; i < s->m; ++i)
    {
        float sigma = s->sigma[i];
        if (!(sigma > 1e-6f * largest))
        {
            continue;
        }
        const float* u = s->w + (size_t)i * s->m;
        const float* b = s->a + (size_t)i * s->n;
        float alpha = 0.0f;
        float spread = 0.0f;
        for (uint32_t r = 0; r < s->m; r += 3)
        {
            alpha += u[r] * s->miss[r] + u[r + 1] * s->miss[r + 1] + u[r + 2] * s->miss[r + 2];
            spread += sqrtf(u[r] * u[r] + u[r + 1] * u[r + 1] + u[r + 2] * u[r + 2]);
        }
        float moved = 0.0f;
        for (uint32_t j = 0; j < s->n; ++j)
        {
            moved += fabsf(b[j]) * s->rho[j];
        }
        // b = sigma v: M_i = sum |v_j| rho_j / sigma = sum |b_j| rho_j / sigma^2.
        moved /= sigma * sigma;
        float gamma = moved > spread ? GAMMA_MAX * spread / moved : GAMMA_MAX;
        for (uint32_t j = 0; j < s->n; ++j)
        {
            s->phi[j] = alpha * b[j] / (sigma * sigma);
        }
        ClampRotations(s->phi, rotations, s->n, gamma);
        for (uint32_t j = 0; j < s->n; ++j)
        {
            s->step[j] += s->phi[j];
        }
    }
    ClampRotations(s->step, rotations, s->n, GAMMA_MAX);
}

// The step taken: each bone's turn (its columns times their scales)
// from the tips up, turning the bones below it; a free top moved.
static void Take(maniBodyPart* part, const Sdls* s)
{
    for (uint32_t k = part->count; k-- > 0;)
    {
        maniVec3 v = maniVec3Make(s->step[3u * k] * s->scale[3u * k],
                                  s->step[3u * k + 1u] * s->scale[3u * k + 1u],
                                  s->step[3u * k + 2u] * s->scale[3u * k + 2u]);
        if (maniVec3Dot(v, v) > 0.0f)
        {
            maniBodyTurn(part, k, maniQuatFromRotationVector(v));
        }
    }
    uint32_t t = 3u * part->count;
    maniVec3 moved = maniVec3Make(s->step[t] * s->scale[t], s->step[t + 1u] * s->scale[t + 1u],
                                  s->step[t + 2u] * s->scale[t + 2u]);
    if (maniVec3Dot(moved, moved) > 0.0f)
    {
        part->nodes[0].origin = maniVec3Add(part->nodes[0].origin, moved);
        maniBodyPlace(part);
    }
}

// Limits from the top down, a limited bone's change carried to the
// bones below it.
static void Limits(maniBodyPart* part, const maniSkeleton* skeleton, const maniBodySolve* solve)
{
    for (uint32_t k = 0; k < part->count; ++k)
    {
        maniQuat before = part->nodes[k].turn;
        maniBodyLimit(part, skeleton, solve, k);
        maniQuat after = part->nodes[k].turn;
        if (after.x == before.x && after.y == before.y && after.z == before.z &&
            after.w == before.w)
        {
            continue;
        }
        maniQuat change = maniQuatMul(after, maniQuatConjugate(before));
        for (uint32_t j = k + 1; j <= k + part->nodes[k].below; ++j)
        {
            part->nodes[j].turn = maniQuatNormalize(maniQuatMul(change, part->nodes[j].turn));
        }
    }
    maniBodyPlace(part);
}

// Half the mean of the part's nonzero offsets, the most a miss counts.
static float MostMiss(const maniBodyPart* part)
{
    float sum = 0.0f;
    uint32_t lengths = 0;
    for (uint32_t k = 1; k < part->count; ++k)
    {
        float l = maniVec3Length(part->nodes[k].offset);
        sum += l;
        lengths += l > 0.0f;
    }
    return lengths > 0 ? 0.5f * sum / (float)lengths : 0.5f;
}

maniResult maniSolveBodySdls(maniPose* pose, const maniSkeleton* skeleton,
                             const maniBodySolve* solve, void* scratch, size_t scratchSize,
                             bool* reachedOut)
{
    uint32_t count = maniBodyPartCount(pose, skeleton, solve, scratch, scratchSize);
    if (count == 0 || solve->stretch)
    {
        return mani_errorInvalid;
    }
    maniBodyPart part;
    maniBodyBegin(&part, pose, skeleton, solve, scratch, count);
    Sdls s = Lay(&part);
    Scales(&part, solve, &s);
    float most = MostMiss(&part);
    maniBodyPrepare(&part, solve);
    for (uint32_t i = 0; i < solve->iterations && maniBodyMiss(&part) > solve->tolerance; ++i)
    {
        for (size_t k = 0; k < (size_t)s.n * s.m; ++k)
        {
            s.a[k] = 0.0f;
        }
        for (uint32_t e = 0; e < part.effectors; ++e)
        {
            Rows(&part, solve, &s, e, most);
        }
        Responses(&s);
        (void)maniJacobiSvd(s.a, s.n, s.m, s.w, s.sigma);
        Step(&s, 3u * count);
        Take(&part, &s);
        Limits(&part, skeleton, solve);
    }
    bool reached = maniBodyFinish(&part, pose, solve);
    if (reachedOut != NULL)
    {
        *reachedOut = reached;
    }
    return mani_success;
}
