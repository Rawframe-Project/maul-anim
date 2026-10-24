// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The benchmark's fixed data, built once through the builders: a rig of
// 64 bones (a spine to the head with a jaw and eyes, two hair chains of
// six, two arms with three fingers of three bones, two legs, a tail of
// nine); 16 clips of 10 s at 30 samples a second, every bone turning but
// the fingers, which hold, and the hips walking ahead (root motion), at
// full precision and compressed with the default options; a graph
// blending two compressed clips by a parameter; a motion database of
// every clip's frames; spring chains on the hair and the tail against
// spheres on the head, chest and hips; a skin of every bone with 20,000
// vertices of four influences; and 32 morph targets of 2,000 vertices
// each over the same mesh, moving positions and normals.

#ifndef MAUL_ANIM_BENCH_SCENES_H
#define MAUL_ANIM_BENCH_SCENES_H

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/ik.h"
#include "maul-anim/match.h"
#include "maul-anim/morph.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BENCH_BONES    64u
#define BENCH_CLIPS    16u
#define BENCH_VERTICES 20000u
#define BENCH_TARGETS  32u
#define BENCH_QUERIES  64u
#define BENCH_DURATION 10.0f

typedef struct BenchScenes
{
    maniSkeleton skeleton;
    maniClip full[BENCH_CLIPS];
    maniClip compressed[BENCH_CLIPS];
    // The graph's instance, its parameter and its scratch.
    maniGraphInstance* instance;
    uint32_t mix;
    void* scratch;
    size_t scratchSize;
    maniMotionDatabase database;
    // Queries near the database's frames, their features raw.
    float* queryFeatures;
    maniMatchQuery queries[BENCH_QUERIES];
    maniSpringState* springs;
    maniSkin skin;
    maniInfluences influences;
    maniMorphTargets morphs;
    float channelWeights[BENCH_TARGETS];
    float* targetWeights;
    // The output pose, two poses to blend (clips 0 and 1 at 1 s), the
    // first's locals to restore before a solve, and a cursor for any
    // compressed clip.
    maniPose* pose;
    maniPose* layers[2];
    maniTransform* locals;
    maniClipCursor* cursor;
    // The solves, toward goals near where the first pose puts the foot,
    // the head and the tail's end.
    maniTwoBoneSolve leg;
    maniAimSolve look;
    maniChainSolve tail;
    // The skin's palette (for the first pose) as matrices and as dual
    // quaternions; the mesh's positions and normals, in and out.
    maniAffine* palette;
    float* dualQuaternions;
    float* positions;
    float* normals;
    float* movedPositions;
    float* movedNormals;
    // The bytes of the skeleton, the first full clip, the first compressed
    // clip and the motion database, kept to open again.
    const void* skeletonFile;
    size_t skeletonFileSize;
    const void* clipFile;
    size_t clipFileSize;
    const void* compressedFile;
    size_t compressedFileSize;
    const void* databaseFile;
    size_t databaseFileSize;
} BenchScenes;

// Builds every scene; false, with the failing step printed, when one
// does not build.
bool BenchBuild(BenchScenes* scenes);

// Puts the first pose's locals back in the output pose.
void BenchRestore(BenchScenes* scenes);

#endif // MAUL_ANIM_BENCH_SCENES_H
