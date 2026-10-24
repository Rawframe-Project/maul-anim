// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Godot differential of pose retargeting: a 53-bone
// humanoid source and a 60-bone target (more spine and neck bones, twist
// bones in the arms) whose bones rest turned differently on each side,
// mapped by name. Godot 4.7.2's RetargetModifier3D in its global mode
// (tools/retarget_godot.gd, run headless) gives each mapped target bone
// the source bone's model turn from its rest times its own rest's model
// rotation, the rule the library's transfer follows; its translations are
// the source's (the library keeps the target's lengths), so only model
// rotations are compared.
//
//   retarget_godot write <in.txt> <poses>   the rigs and random poses
//   retarget_godot check <in.txt> <out.txt> the library's transfer of
//                                           the same poses against
//                                           Godot's
//
// in.txt: "rigs S T", S then T lines "name parent tx ty tz qx qy qz qw"
// (rests); "poses P", then P times S lines "qx qy qz qw" (the source's
// local rotations; translations stay at the rests). out.txt, written by
// the script: P times T lines "qx qy qz qw", each target bone's model
// rotation. Not library code.

#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
    kMaxBones = 64
};

typedef struct Bone
{
    char name[24];
    int parent;
    maniVec3 offset;
    maniQuat rest;
} Bone;

typedef struct Rig
{
    Bone bones[kMaxBones];
    int count;
} Rig;

static Rig s_source;
static Rig s_target;
static uint64_t s_state = 0x9E3779B97F4A7C15u;

static double Random(void)
{
    s_state ^= s_state << 13;
    s_state ^= s_state >> 7;
    s_state ^= s_state << 17;
    return (double)(s_state >> 11) / 9007199254740992.0;
}

// A unit quaternion of a random axis and an angle up to limit.
static maniQuat Turn(double limit)
{
    double x = Random() * 2.0 - 1.0;
    double y = Random() * 2.0 - 1.0;
    double z = Random() * 2.0 - 1.0;
    double n = sqrt(x * x + y * y + z * z);
    double a = (Random() * 2.0 - 1.0) * limit;
    double s = sin(0.5 * a) / (n > 1e-9 ? n : 1.0);
    return (maniQuat){(float)(x * s), (float)(y * s), (float)(z * s), (float)cos(0.5 * a)};
}

static int Add(Rig* rig, const char* name, int parent, float x, float y, float z)
{
    Bone* b = &rig->bones[rig->count];
    snprintf(b->name, sizeof b->name, "%s", name);
    b->parent = parent;
    b->offset = (maniVec3){x, y, z};
    b->rest = Turn(1.2);
    return rig->count++;
}

// Arms and legs; twist bones in the target's arms.
static void Limbs(Rig* rig, int chest, int hips, int twist)
{
    for (int side = 0; side < 2; ++side)
    {
        float s = side == 0 ? 1.0f : -1.0f;
        const char* l = side == 0 ? "L" : "R";
        char n[24];
        snprintf(n, sizeof n, "clav%s", l);
        int c = Add(rig, n, chest, 0.05f * s, 0.1f, 0.0f);
        snprintf(n, sizeof n, "upper%s", l);
        int u = Add(rig, n, c, 0.15f * s, 0.0f, 0.0f);
        if (twist)
        {
            snprintf(n, sizeof n, "upperTwist%s", l);
            u = Add(rig, n, u, 0.12f * s, 0.0f, 0.0f);
        }
        snprintf(n, sizeof n, "fore%s", l);
        int f = Add(rig, n, u, 0.15f * s, 0.0f, 0.0f);
        if (twist)
        {
            snprintf(n, sizeof n, "foreTwist%s", l);
            f = Add(rig, n, f, 0.12f * s, 0.0f, 0.0f);
        }
        snprintf(n, sizeof n, "hand%s", l);
        int h = Add(rig, n, f, 0.12f * s, 0.0f, 0.0f);
        for (int g = 0; g < 5; ++g)
        {
            int p = h;
            for (int k = 0; k < 3; ++k)
            {
                snprintf(n, sizeof n, "f%d%d%s", g, k, l);
                p = Add(rig, n, p, 0.03f * s, 0.0f, 0.01f * (float)(g - 2));
            }
        }
        snprintf(n, sizeof n, "thigh%s", l);
        int t = Add(rig, n, hips, 0.1f * s, -0.05f, 0.0f);
        snprintf(n, sizeof n, "calf%s", l);
        int k = Add(rig, n, t, 0.0f, -0.45f, 0.0f);
        snprintf(n, sizeof n, "foot%s", l);
        int ft = Add(rig, n, k, 0.0f, -0.45f, 0.0f);
        snprintf(n, sizeof n, "toe%s", l);
        Add(rig, n, ft, 0.0f, -0.05f, 0.12f);
    }
}

static void Rigs(void)
{
    int root = Add(&s_source, "root", -1, 0, 0, 0);
    int hips = Add(&s_source, "hips", root, 0, 1.0f, 0);
    int p = hips;
    for (int i = 0; i < 3; ++i)
    {
        char n[8];
        snprintf(n, sizeof n, "s%d", i);
        p = Add(&s_source, n, p, 0, 0.15f, 0);
    }
    int neck = Add(&s_source, "neck", p, 0, 0.15f, 0);
    Add(&s_source, "head", neck, 0, 0.1f, 0);
    Limbs(&s_source, p, hips, 0);
    root = Add(&s_target, "root", -1, 0, 0, 0);
    hips = Add(&s_target, "hips", root, 0, 0.9f, 0);
    p = hips;
    for (int i = 0; i < 5; ++i)
    {
        char n[8];
        snprintf(n, sizeof n, "t%d", i);
        p = Add(&s_target, n, p, 0, 0.09f, 0);
    }
    neck = Add(&s_target, "neck", p, 0, 0.07f, 0);
    int neck2 = Add(&s_target, "neck2", neck, 0, 0.07f, 0);
    Add(&s_target, "head", neck2, 0, 0.1f, 0);
    Limbs(&s_target, p, hips, 1);
}

static void Die(const char* what)
{
    fprintf(stderr, "retarget_godot: %s\n", what);
    exit(2);
}

static maniBuilder* s_builder;

// The skeleton of a rig, and where its builder put each of the rig's
// bones (it orders them depth first).
static void Skeleton(const Rig* rig, uint8_t* file, size_t capacity, maniSkeleton* out,
                     uint32_t* at)
{
    maniBoneSource bones[kMaxBones];
    for (int i = 0; i < rig->count; ++i)
    {
        const Bone* b = &rig->bones[i];
        bones[i] = (maniBoneSource){
            b->name, (uint32_t)strlen(b->name), b->parent, {b->offset, b->rest, {1, 1, 1}}};
    }
    maniSkeletonSource source = {bones, (uint32_t)rig->count, 0};
    size_t size = 0;
    if (maniBuildSkeleton(s_builder, &source).result != mani_success ||
        maniCopyBuilt(s_builder, file, capacity, &size) != mani_success ||
        maniOpenSkeleton(file, size, out).result != mani_success)
    {
        Die("a skeleton");
    }
    for (int i = 0; i < rig->count; ++i)
    {
        at[i] = maniSkeletonFindBone(out, rig->bones[i].name, strlen(rig->bones[i].name));
        if (at[i] == MANI_NO_BONE)
        {
            Die("a bone not found");
        }
    }
}

static void WriteRig(FILE* f, const Rig* rig)
{
    for (int i = 0; i < rig->count; ++i)
    {
        const Bone* b = &rig->bones[i];
        fprintf(f, "%s %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", b->name, b->parent,
                (double)b->offset.x, (double)b->offset.y, (double)b->offset.z, (double)b->rest.x,
                (double)b->rest.y, (double)b->rest.z, (double)b->rest.w);
    }
}

// Pose k's source local rotations: each bone's rest turned by up to 1.5
// radians.
static void Pose(maniQuat* rotations)
{
    for (int i = 0; i < s_source.count; ++i)
    {
        maniQuat t = Turn(1.5);
        maniQuat r = s_source.bones[i].rest;
        rotations[i] = (maniQuat){t.w * r.x + t.x * r.w + t.y * r.z - t.z * r.y,
                                  t.w * r.y - t.x * r.z + t.y * r.w + t.z * r.x,
                                  t.w * r.z + t.x * r.y - t.y * r.x + t.z * r.w,
                                  t.w * r.w - t.x * r.x - t.y * r.y - t.z * r.z};
    }
}

typedef struct Q
{
    double x, y, z, w;
} Q;

static Q Mul(Q a, Q b)
{
    return (Q){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
               a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
               a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
               a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// The angle between two rotations, in radians.
static double Angle(Q a, Q b)
{
    double na = sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
    double nb = sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w);
    double d = fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) / (na * nb);
    return 2.0 * acos(d < 1.0 ? d : 1.0);
}

static int Check(const char* inPath, const char* outPath)
{
    static _Alignas(16) uint8_t files[3][1 << 14];
    static _Alignas(MANI_POSE_ALIGNMENT) uint8_t poses[2][1 << 14];
    static _Alignas(16) uint8_t scratch[1 << 14];
    maniSkeleton source;
    maniSkeleton target;
    uint32_t sourceAt[kMaxBones];
    uint32_t targetAt[kMaxBones];
    Skeleton(&s_source, files[0], sizeof files[0], &source, sourceAt);
    Skeleton(&s_target, files[1], sizeof files[1], &target, targetAt);
    maniRetargetMapSource ms = {.source = &source, .target = &target, .matchNames = true};
    maniRetargetMap map;
    size_t size = 0;
    if (maniBuildRetargetMap(s_builder, &ms).result != mani_success ||
        maniCopyBuilt(s_builder, files[2], sizeof files[2], &size) != mani_success ||
        maniOpenRetargetMap(files[2], size, &map).result != mani_success)
    {
        Die("the map");
    }
    maniPose* in = NULL;
    maniPose* out = NULL;
    if (maniInitPose(&source, poses[0], sizeof poses[0], &in) != mani_success ||
        maniInitPose(&target, poses[1], sizeof poses[1], &out) != mani_success)
    {
        Die("the poses");
    }
    FILE* given = fopen(inPath, "r");
    FILE* godot = fopen(outPath, "r");
    if (given == NULL || godot == NULL)
    {
        Die("the files");
    }
    int s = 0;
    int t = 0;
    int count = 0;
    char line[256];
    if (fscanf(given, "rigs %d %d\n", &s, &t) != 2 || s != s_source.count || t != s_target.count)
    {
        Die("in.txt's rigs");
    }
    for (int i = 0; i < s + t; ++i)
    {
        if (fgets(line, sizeof line, given) == NULL)
        {
            Die("in.txt's bones");
        }
    }
    if (fscanf(given, "poses %d\n", &count) != 1 || count < 1)
    {
        Die("in.txt's poses");
    }
    double worst = 0.0;
    uint32_t compared = 0;
    for (int k = 0; k < count; ++k)
    {
        if (maniPoseSetRest(in, &source) != mani_success)
        {
            Die("the rest");
        }
        for (int i = 0; i < s; ++i)
        {
            maniTransform local = maniPoseLocal(in, sourceAt[i]);
            if (fscanf(given, "%f %f %f %f\n", &local.rotation.x, &local.rotation.y,
                       &local.rotation.z, &local.rotation.w) != 4)
            {
                Die("in.txt's pose");
            }
            maniPoseSetLocal(in, sourceAt[i], local);
        }
        if (maniRetargetPose(&map, &source, in, &target, out, scratch, sizeof scratch) !=
            mani_success)
        {
            Die("the transfer");
        }
        Q model[kMaxBones];
        for (int i = 0; i < t; ++i)
        {
            maniQuat r = maniPoseLocal(out, targetAt[i]).rotation;
            Q local = {(double)r.x, (double)r.y, (double)r.z, (double)r.w};
            int parent = s_target.bones[i].parent;
            model[i] = parent < 0 ? local : Mul(model[parent], local);
            Q theirs;
            if (fscanf(godot, "%lf %lf %lf %lf\n", &theirs.x, &theirs.y, &theirs.z, &theirs.w) != 4)
            {
                Die("out.txt");
            }
            double angle = Angle(model[i], theirs);
            worst = angle > worst ? angle : worst;
            ++compared;
        }
    }
    fclose(given);
    fclose(godot);
    printf("%d poses, %u target bones' model rotations: largest difference %.3g rad\n", count,
           compared, worst);
    // Godot works in binary32 too; a turn's rounding over ten bones of a
    // path stays under 1e-5 radians.
    return worst <= 1e-5 ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        Die("usage: retarget_godot write <in.txt> <poses> | check <in.txt> <out.txt>");
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    if (maniCreateBuilder(&def, &s_builder) != mani_success)
    {
        Die("a builder");
    }
    Rigs();
    int result = 0;
    if (strcmp(argv[1], "write") == 0)
    {
        int count = atoi(argv[3]);
        FILE* f = fopen(argv[2], "w");
        if (f == NULL || count < 1)
        {
            Die("in.txt");
        }
        fprintf(f, "rigs %d %d\n", s_source.count, s_target.count);
        WriteRig(f, &s_source);
        WriteRig(f, &s_target);
        fprintf(f, "poses %d\n", count);
        maniQuat rotations[kMaxBones];
        for (int k = 0; k < count; ++k)
        {
            Pose(rotations);
            for (int i = 0; i < s_source.count; ++i)
            {
                fprintf(f, "%.9g %.9g %.9g %.9g\n", (double)rotations[i].x, (double)rotations[i].y,
                        (double)rotations[i].z, (double)rotations[i].w);
            }
        }
        fclose(f);
    }
    else if (strcmp(argv[1], "check") == 0)
    {
        result = Check(argv[2], argv[3]);
    }
    else
    {
        Die("a mode");
    }
    maniDestroyBuilder(s_builder);
    return result;
}
