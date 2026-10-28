// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Pose parameters: the rest pose until set, their bytes set
// aside in the instance; a pose set and
// read back, the other pose parameter untouched; a parameter of another
// type, past the count, a difference or another skeleton's pose refused;
// the value calls refusing the type; the saved state holding each pose
// bone by bone, restored into another instance, a word out of form
// refused at its offset with the instance unchanged.

#include "bytes.h"
#include "graph_fixture.h"
#include "pose_memory.h"

#include "maul-anim/pose.h"

static _Alignas(32) uint8_t s_poseMemory[2][4096];
static _Alignas(32) uint8_t s_otherPoseMemory[4096];
static _Alignas(8) uint8_t s_otherInstance[8192];
static _Alignas(16) uint8_t s_otherSkeletonFile[4096];
static _Alignas(8) uint8_t s_saved[2][4096];
static maniPose* s_pose;
static maniPose* s_read;
static maniGraphParameterSource s_parameters[4];

// Parameters: 0 a float, 1 a pose, 2 a bool, 3 a pose.
static maniGraphInstance* PoseGraph(maniGraph* g)
{
    s_parameters[0] = Parameter("speed", maniGraphParameterFloat, Float(1.0f));
    s_parameters[1] = Parameter("ragdoll", maniGraphParameterPose, Int(0));
    s_parameters[2] = Parameter("lean", maniGraphParameterBool, Int(0));
    s_parameters[3] = Parameter("held", maniGraphParameterPose, Int(0));
    static maniGraphNodeSource nodes[1];
    nodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    return Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 4, .nodeCount = 1},
        g);
}

static bool Same(maniTransform a, maniTransform b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

// Bone b of a pose the test sets: each bone moved and turned apart.
static maniTransform Moved(uint32_t b)
{
    float f = (float)(b + 1);
    return (maniTransform){
        {0.25f * f, -0.5f * f, 2.0f}, {0.0f, 0.6f, 0.0f, 0.8f}, {1.0f, 1.5f * f, 0.5f}};
}

// Every bone of a pose parameter as the skeleton's rest.
static bool AtRest(const maniGraphInstance* instance, uint32_t parameter)
{
    bool ok = maniGetGraphPose(instance, parameter, s_read) == mani_success;
    for (uint32_t b = 0; b < 3; ++b)
    {
        ok = ok && Same(maniPoseLocal(s_read, b), maniSkeletonRest(&s_skeleton, b));
    }
    return ok;
}

static bool AsMoved(const maniGraphInstance* instance, uint32_t parameter)
{
    bool ok = maniGetGraphPose(instance, parameter, s_read) == mani_success;
    for (uint32_t b = 0; b < 3; ++b)
    {
        ok = ok && Same(maniPoseLocal(s_read, b), Moved(b));
    }
    return ok;
}

static void MovePose(void)
{
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniPoseSetLocal(s_pose, b, Moved(b));
    }
}

// Rest until set; set and read back, the other pose untouched; read
// into a difference, it becomes a pose with its model stale; reset
// leaves the poses as set (they are the host's inputs).
static void TestSetAndGet(void)
{
    maniGraph g;
    maniGraphInstance* instance = PoseGraph(&g);
    CHECK(AtRest(instance, 1) && AtRest(instance, 3), "the rest pose until set");
    // Two poses of one group of bones each, between the machines' blocks
    // and the room, inside the instance.
    CHECK(instance->roomAt - instance->posesAt ==
                  2 * MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS * sizeof(float) &&
              instance->posesAt >= instance->machinesAt && instance->size >= instance->roomAt,
          "the poses' bytes set aside before the room");
    MovePose();
    CHECK(maniSetGraphPose(instance, 1, s_pose) == mani_success, "set");
    CHECK(AsMoved(instance, 1) && AtRest(instance, 3), "read back, the other untouched");
    maniPoseSetKind(s_read, true);
    CHECK(maniGetGraphPose(instance, 1, s_read) == mani_success && !maniPoseIsDifference(s_read) &&
              !maniPoseModelCurrent(s_read),
          "read as a pose, its model stale");
    CHECK(maniComputeModel(s_read, &s_skeleton) == mani_success &&
              maniGetGraphPose(instance, 3, s_read) == mani_success &&
              !maniPoseModelCurrent(s_read),
          "a current model made stale by a read");
    maniGraphValue v = Float(5.0f);
    CHECK(maniGetGraphParameter(instance, 1, &v) == mani_success && v.f[0] == 0.0f &&
              v.f[1] == 0.0f && v.f[2] == 0.0f && v.f[3] == 0.0f,
          "a pose parameter's value is 0");
}

// A parameter of another type or past the count, NULL, a difference or
// another skeleton's pose refused with nothing changed; the value call
// refuses the type.
static void TestRefusals(void)
{
    maniGraph g;
    maniGraphInstance* instance = PoseGraph(&g);
    MovePose();
    CHECK(maniSetGraphPose(instance, 0, s_pose) == mani_errorInvalid &&
              maniSetGraphPose(instance, 2, s_pose) == mani_errorInvalid &&
              maniSetGraphPose(instance, 4, s_pose) == mani_errorInvalid &&
              maniSetGraphPose(NULL, 1, s_pose) == mani_errorInvalid &&
              maniSetGraphPose(instance, 1, NULL) == mani_errorInvalid,
          "set: another type, past the count, NULL");
    CHECK(maniGetGraphPose(instance, 0, s_read) == mani_errorInvalid &&
              maniGetGraphPose(instance, 2, s_read) == mani_errorInvalid &&
              maniGetGraphPose(instance, 4, s_read) == mani_errorInvalid &&
              maniGetGraphPose(NULL, 1, s_read) == mani_errorInvalid &&
              maniGetGraphPose(instance, 1, NULL) == mani_errorInvalid,
          "get: another type, past the count, NULL");
    maniPoseSetKind(s_pose, true);
    CHECK(maniSetGraphPose(instance, 1, s_pose) == mani_errorInvalid, "a difference refused");
    maniPoseSetKind(s_pose, false);
    // A skeleton of the fixture's first two bones.
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource two = {kBones, 2};
    maniSkeleton other;
    maniPose* otherPose = NULL;
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &two).result == mani_success &&
              maniCopyBuilt(builder, s_otherSkeletonFile, sizeof s_otherSkeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_otherSkeletonFile, size, &other).result == mani_success &&
              maniInitPose(&other, s_otherPoseMemory, sizeof s_otherPoseMemory, &otherPose) ==
                  mani_success,
          "another skeleton's pose");
    maniDestroyBuilder(builder);
    CHECK(maniSetGraphPose(instance, 1, otherPose) == mani_errorInvalid &&
              maniGetGraphPose(instance, 1, otherPose) == mani_errorInvalid,
          "another skeleton's pose refused");
    CHECK(maniSetGraphParameter(instance, 1, maniGraphParameterPose, Int(0)) == mani_errorInvalid &&
              maniSetGraphParameter(instance, 1, maniGraphParameterFloat, Float(1.0f)) ==
                  mani_errorInvalid,
          "the value call refuses a pose parameter");
    CHECK(AtRest(instance, 1) && AtRest(instance, 3), "nothing changed");
}

// The saved state: its size, each pose's locals bone by bone after the
// machines' blocks (none here), restored into another instance.
static void TestSaved(void)
{
    maniGraph g;
    maniGraphInstance* instance = PoseGraph(&g);
    MovePose();
    CHECK(maniSetGraphPose(instance, 3, s_pose) == mani_success, "set");
    size_t size = maniGraphStateSize(instance->binding);
    size_t poses = MANI_GRAPH_STATE_HEADER_SIZE + (4 + 1) * MANI_GRAPH_VALUE_SIZE;
    CHECK(size == poses + 2 * 3 * MANI_POSE_COMPONENTS * 4, "two poses of three bones");
    CHECK(maniSaveGraphInstance(instance, s_saved[0], size - 1) == mani_errorCapacity &&
              maniSaveGraphInstance(instance, s_saved[0], size) == mani_success,
          "saved");
    float word = 0.0f;
    memcpy(&word, s_saved[0] + poses + 3 * MANI_POSE_COMPONENTS * 4 + MANI_POSE_COMPONENTS * 4 + 8,
           sizeof word);
    CHECK(word == Moved(1).translation.z, "the second pose's bone 1 after the first pose");
    maniGraphInstance* other = NULL;
    CHECK(maniInitGraphInstance(instance->binding, s_otherInstance, sizeof s_otherInstance,
                                &other) == mani_success &&
              AtRest(other, 3),
          "another instance");
    maniFileResult r = maniRestoreGraphInstance(other, s_saved[0], size);
    CHECK(r.result == mani_success && AtRest(other, 1) && AsMoved(other, 3), "restored");
}

// A word out of form refused at its offset, the instance unchanged:
// translation, rotation and scale not finite, a rotation of length 1.002
// (its first word); one of length 1.0005 taken; a pose parameter's value
// not 0.
static void TestSavedRefused(void)
{
    maniGraph g;
    maniGraphInstance* instance = PoseGraph(&g);
    size_t size = maniGraphStateSize(instance->binding);
    CHECK(maniSaveGraphInstance(instance, s_saved[0], size) == mani_success, "saved");
    size_t bone = MANI_GRAPH_STATE_HEADER_SIZE + (4 + 1) * MANI_GRAPH_VALUE_SIZE +
                  (3 + 2) * MANI_POSE_COMPONENTS * 4;
    MovePose();
    CHECK(maniSetGraphPose(instance, 3, s_pose) == mani_success, "set");
    const struct
    {
        size_t at;
        uint32_t bits;
        size_t refused;
    } kBad[5] = {{bone + 4, 0x7F800000u, bone + 4},
                 {bone + 24, 0x7FC00000u, bone + 24},
                 {bone + 36, 0xFF800000u, bone + 36},
                 // The rest rotation (0, 0, 0, 1) as (0, 0, 0, 1.001).
                 {bone + 24, 0x3F8020C5u, bone + 12},
                 {MANI_GRAPH_STATE_HEADER_SIZE + 3 * MANI_GRAPH_VALUE_SIZE + 4, 0x3F800000u,
                  MANI_GRAPH_STATE_HEADER_SIZE + 3 * MANI_GRAPH_VALUE_SIZE}};
    for (int k = 0; k < 5; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + kBad[k].at, kBad[k].bits);
        maniFileResult r = maniRestoreGraphInstance(instance, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == kBad[k].refused && AsMoved(instance, 3),
              "refused at the word, the instance unchanged");
    }
    memcpy(s_saved[1], s_saved[0], size);
    // (0, 0, 0, 1.0005): its square within 0.002 of 1.
    maniStoreU32(s_saved[1] + bone + 24, 0x3F801062u);
    CHECK(maniRestoreGraphInstance(instance, s_saved[1], size).result == mani_success,
          "a rotation within a thousandth taken");
}

int main(void)
{
    Open();
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[0], sizeof s_poseMemory[0], &s_pose) ==
                  mani_success &&
              maniInitPose(&s_skeleton, s_poseMemory[1], sizeof s_poseMemory[1], &s_read) ==
                  mani_success,
          "poses");
    TestSetAndGet();
    TestRefusals();
    TestSaved();
    TestSavedRefused();
    return s_failures == 0 ? 0 : 1;
}
