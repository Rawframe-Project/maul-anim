// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The rigs the maps' JSON tests and fuzz target (mani-0034) name the
// bones of: a retargeting source and target of different spine and arm
// lengths, and a rig whose left and right sides rest on unmirrored axes.

#ifndef MAUL_ANIM_TEST_MAP_JSON_ASSETS_H
#define MAUL_ANIM_TEST_MAP_JSON_ASSETS_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

// Root, hips, two spine bones and a chest, an arm of two, a head.
static const maniBoneSource kSourceRig[8] = {{"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
                                             {"hips", 4, 0, {{0, 1, 0}, REST, {1, 1, 1}}},
                                             {"s1", 2, 1, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
                                             {"s2", 2, 2, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
                                             {"chest", 5, 3, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
                                             {"armL", 4, 4, {{0.2f, 0.1f, 0}, REST, {1, 1, 1}}},
                                             {"handL", 5, 5, {{0.3f, 0, 0}, REST, {1, 1, 1}}},
                                             {"head", 4, 4, {{0, 0.3f, 0}, REST, {1, 1, 1}}}};

// Root, a pelvis, three spine bones and a chest, an arm of three, a
// head.
static const maniBoneSource kTargetRig[10] = {
    {"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
    {"pelvis", 6, 0, {{0, 0.9f, 0}, REST, {1, 1, 1}}},
    {"sp1", 3, 1, {{0, 0.15f, 0}, REST, {1, 1, 1}}},
    {"sp2", 3, 2, {{0, 0.15f, 0}, REST, {1, 1, 1}}},
    {"sp3", 3, 3, {{0, 0.15f, 0}, REST, {1, 1, 1}}},
    {"chest", 5, 4, {{0, 0.15f, 0}, REST, {1, 1, 1}}},
    {"upperarm", 8, 5, {{0.2f, 0.1f, 0}, REST, {1, 1, 1}}},
    {"forearm", 7, 6, {{0.15f, 0, 0}, REST, {1, 1, 1}}},
    {"hand", 4, 7, {{0.15f, 0, 0}, REST, {1, 1, 1}}},
    {"head", 4, 5, {{0, 0.3f, 0}, REST, {1, 1, 1}}}};

enum
{
    kRoot,
    kHips,
    kHead,
    kArmL,
    kHandL,
    kArmR,
    kHandR,
    kPropR,
    kLegLeft,
    kLegRight,
    kBones
};

// Arms and hands paired by suffix, legs by an anywhere rule, a prop
// under the right hand paired to itself.
static const maniBoneSource kMirrorRig[kBones] = {
    {"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
    {"hips", 4, kRoot, {{0, 1, 0}, REST, {1, 1, 1}}},
    {"head", 4, kHips, {{0, 0.6f, 0}, REST, {1, 1, 1}}},
    {"armL", 4, kHips, {{0.2f, 0.5f, 0}, {0, 0, 0.25881904f, 0.96592583f}, {1, 1, 1}}},
    {"handL", 5, kArmL, {{0.3f, 0, 0}, REST, {1, 1, 1}}},
    {"armR", 4, kHips, {{-0.2f, 0.5f, 0}, {0, 0.70710678f, 0, 0.70710678f}, {1, 1, 1}}},
    {"handR", 5, kArmR, {{0, 0, 0.3f}, REST, {1, 1, 1}}},
    {"propR", 5, kHandR, {{0, 0, 0.1f}, REST, {1, 1, 1}}},
    {"leg_left_upper", 14, kHips, {{0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
    {"leg_right_upper", 15, kHips, {{-0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
};

#undef REST

// A rig built into file (4096 bytes, 16-byte aligned) and opened.
static bool OpenRig(const maniBoneSource* bones, uint32_t count, uint8_t* file, maniSkeleton* out)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) != mani_success)
    {
        return false;
    }
    maniSkeletonSource source = {bones, count, 0};
    size_t size = 0;
    bool ok = maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, 4096, &size) == mani_success &&
              maniOpenSkeleton(file, size, out).result == mani_success;
    maniDestroyBuilder(builder);
    return ok;
}

#endif // MAUL_ANIM_TEST_MAP_JSON_ASSETS_H
