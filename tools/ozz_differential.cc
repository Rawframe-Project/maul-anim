// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ozz differential: an FBX file's skeleton and clips,
// imported by Maul Anim, sampled at random times into poses that both
// libraries then blend: ozz-animation's BlendingJob and maniBlendPoses,
// with the same layers, weights, masks and threshold, then additive
// layers (differences from the rest) by both. Printed: the largest
// difference of each component kind over every round, and the
// nanoseconds a blend of two poses takes in each. Built with
// MAUL_ANIM_DIFFERENTIAL against ozz-animation at the commit CMake pins.
//
// usage: ozz_differential <file.fbx> [rounds [repetitions [side]]]: side
//        0 or 1 times only ozz or only Maul Anim, for counting
//        instructions.

#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <ozz/animation/offline/raw_skeleton.h>
#include <ozz/animation/offline/skeleton_builder.h>
#include <ozz/animation/runtime/blending_job.h>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/base/maths/simd_math.h>
#include <ozz/base/maths/soa_transform.h>
#include <vector>

namespace
{

constexpr float kRate = 30.0f;
constexpr float kThreshold = 0.1f;
constexpr int kMaxLayers = 4;

struct Bytes
{
    std::vector<std::max_align_t> words;
    size_t size = 0;
    const void* data() const
    {
        return words.data();
    }
};

bool ReadFile(const char* path, Bytes* out)
{
    FILE* file = std::fopen(path, "rb");
    if (file == nullptr)
    {
        return false;
    }
    std::fseek(file, 0, SEEK_END);
    long size = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    out->size = size > 0 ? static_cast<size_t>(size) : 0;
    out->words.resize(out->size / sizeof(std::max_align_t) + 1);
    bool read = std::fread(out->words.data(), 1, out->size, file) == out->size;
    std::fclose(file);
    return read;
}

bool CopyBuilt(maniBuilder* builder, Bytes* out)
{
    size_t size = 0;
    if (maniCopyBuilt(builder, nullptr, 0, &size) != mani_errorCapacity)
    {
        return false;
    }
    out->words.resize(size / sizeof(std::max_align_t) + 1);
    out->size = size;
    return maniCopyBuilt(builder, out->words.data(), size, nullptr) == mani_success;
}

int Fail(const char* what)
{
    std::fprintf(stderr, "ozz_differential: %s\n", what);
    return 1;
}

uint64_t s_random = 0x9E3779B97F4A7C15ull;

uint32_t Next()
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return static_cast<uint32_t>(s_random >> 32);
}

float Uniform(float low, float high)
{
    return low + (high - low) * (static_cast<float>(Next() >> 8) / 16777216.0f);
}

// A pose in caller memory, aligned as the library asks.
struct Pose
{
    std::vector<std::max_align_t> words;
    maniPose* pose = nullptr;

    bool Init(const maniSkeleton* skeleton)
    {
        size_t size = maniPoseSize(skeleton) + MANI_POSE_ALIGNMENT;
        words.resize(size / sizeof(std::max_align_t) + 1);
        auto at = reinterpret_cast<uintptr_t>(words.data());
        auto* memory = reinterpret_cast<uint8_t*>(words.data()) +
                       (MANI_POSE_ALIGNMENT - at % MANI_POSE_ALIGNMENT) % MANI_POSE_ALIGNMENT;
        return maniInitPose(skeleton, memory, size - MANI_POSE_ALIGNMENT, &pose) == mani_success;
    }
};

// ozz's skeleton of the same joints in the same order: each bone's
// children in increasing order, which ozz walks depth-first as Maul Anim
// stores them.
ozz::animation::offline::RawSkeleton::Joint MakeJoint(const maniSkeleton* skeleton, uint32_t bone)
{
    ozz::animation::offline::RawSkeleton::Joint joint;
    uint32_t length = 0;
    const char* name = maniSkeletonBoneName(skeleton, bone, &length);
    joint.name = ozz::string(name, length);
    maniTransform rest = maniSkeletonRest(skeleton, bone);
    joint.transform.translation =
        ozz::math::Float3(rest.translation.x, rest.translation.y, rest.translation.z);
    joint.transform.rotation =
        ozz::math::Quaternion(rest.rotation.x, rest.rotation.y, rest.rotation.z, rest.rotation.w);
    joint.transform.scale = ozz::math::Float3(rest.scale.x, rest.scale.y, rest.scale.z);
    for (uint32_t c = bone + 1; c < maniSkeletonBoneCount(skeleton); ++c)
    {
        if (maniSkeletonParent(skeleton, c) == bone)
        {
            joint.children.push_back(MakeJoint(skeleton, c));
        }
    }
    return joint;
}

ozz::unique_ptr<ozz::animation::Skeleton> MakeOzzSkeleton(const maniSkeleton* skeleton)
{
    ozz::animation::offline::RawSkeleton raw;
    for (uint32_t b = 0; b < maniSkeletonBoneCount(skeleton); ++b)
    {
        if (maniSkeletonParent(skeleton, b) == MANI_NO_BONE)
        {
            raw.roots.push_back(MakeJoint(skeleton, b));
        }
    }
    ozz::animation::offline::SkeletonBuilder build;
    ozz::unique_ptr<ozz::animation::Skeleton> out = build(raw);
    if (out == nullptr ||
        static_cast<uint32_t>(out->num_joints()) != maniSkeletonBoneCount(skeleton))
    {
        return nullptr;
    }
    for (uint32_t b = 0; b < maniSkeletonBoneCount(skeleton); ++b)
    {
        uint32_t length = 0;
        const char* name = maniSkeletonBoneName(skeleton, b, &length);
        if (std::strlen(out->joint_names()[b]) != length ||
            std::memcmp(out->joint_names()[b], name, length) != 0)
        {
            return nullptr;
        }
    }
    return out;
}

using Soa = std::vector<ozz::math::SoaTransform>;

// A pose's locals as ozz's four-joint SoA, padding joints the identity.
void ToSoa(const maniPose* pose, uint32_t bones, Soa* out)
{
    out->resize((bones + 3) / 4);
    for (size_t g = 0; g < out->size(); ++g)
    {
        maniTransform t[4];
        for (uint32_t k = 0; k < 4; ++k)
        {
            uint32_t b = static_cast<uint32_t>(g) * 4 + k;
            t[k] = b < bones ? maniPoseLocal(pose, b)
                             : maniTransform{{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
        }
        auto L = [&](auto f)
        { return ozz::math::simd_float4::Load(f(t[0]), f(t[1]), f(t[2]), f(t[3])); };
        ozz::math::SoaTransform& s = (*out)[g];
        s.translation.x = L([](const maniTransform& v) { return v.translation.x; });
        s.translation.y = L([](const maniTransform& v) { return v.translation.y; });
        s.translation.z = L([](const maniTransform& v) { return v.translation.z; });
        s.rotation.x = L([](const maniTransform& v) { return v.rotation.x; });
        s.rotation.y = L([](const maniTransform& v) { return v.rotation.y; });
        s.rotation.z = L([](const maniTransform& v) { return v.rotation.z; });
        s.rotation.w = L([](const maniTransform& v) { return v.rotation.w; });
        s.scale.x = L([](const maniTransform& v) { return v.scale.x; });
        s.scale.y = L([](const maniTransform& v) { return v.scale.y; });
        s.scale.z = L([](const maniTransform& v) { return v.scale.z; });
    }
}

// One joint's ten components from ozz's SoA.
void FromSoa(const Soa& soa, uint32_t bone, float out[10])
{
    const ozz::math::SoaTransform& s = soa[bone / 4];
    const ozz::math::SimdFloat4 parts[10] = {
        s.translation.x, s.translation.y, s.translation.z, s.rotation.x, s.rotation.y,
        s.rotation.z,    s.rotation.w,    s.scale.x,       s.scale.y,    s.scale.z};
    for (int c = 0; c < 10; ++c)
    {
        float lanes[4];
        ozz::math::StorePtrU(parts[c], lanes);
        out[c] = lanes[bone % 4];
    }
}

// Joint weights per layer as ozz's SoA of four.
void ToSoaWeights(const std::vector<float>& weights, std::vector<ozz::math::SimdFloat4>* out)
{
    out->resize((weights.size() + 3) / 4);
    for (size_t g = 0; g < out->size(); ++g)
    {
        float w[4];
        for (size_t k = 0; k < 4; ++k)
        {
            w[k] = g * 4 + k < weights.size() ? weights[g * 4 + k] : 0.0f;
        }
        (*out)[g] = ozz::math::simd_float4::Load(w[0], w[1], w[2], w[3]);
    }
}

// The largest differences: translations, rotations (sign-aware) and
// scales, relative to their size above 1.
struct Apart
{
    double translation = 0.0;
    double rotation = 0.0;
    double scale = 0.0;

    void Add(const maniPose* ours, const Soa& theirs, uint32_t bones)
    {
        for (uint32_t b = 0; b < bones; ++b)
        {
            maniTransform t = maniPoseLocal(ours, b);
            float o[10] = {t.translation.x, t.translation.y, t.translation.z, t.rotation.x,
                           t.rotation.y,    t.rotation.z,    t.rotation.w,    t.scale.x,
                           t.scale.y,       t.scale.z};
            float z[10];
            FromSoa(theirs, b, z);
            float dot = o[3] * z[3] + o[4] * z[4] + o[5] * z[5] + o[6] * z[6];
            for (int c = 0; c < 10; ++c)
            {
                float other = c >= 3 && c < 7 && dot < 0.0f ? -z[c] : z[c];
                double size = std::fmax(1.0, std::fabs(static_cast<double>(o[c])));
                double e = std::fabs(static_cast<double>(o[c]) - static_cast<double>(other)) / size;
                double& worst = c < 3 ? translation : (c < 7 ? rotation : scale);
                worst = std::fmax(worst, e);
            }
        }
    }
};

struct Run
{
    const maniSkeleton* skeleton = nullptr;
    ozz::unique_ptr<ozz::animation::Skeleton> ozz;
    std::vector<maniClip> clips;
    uint32_t bones = 0;
};

// A clip at a random time into a pose.
bool SampleRandom(const Run& run, maniPose* pose)
{
    const maniClip& clip = run.clips[Next() % run.clips.size()];
    return maniSampleClip(&clip, run.skeleton, Uniform(0.0f, maniClipDuration(&clip)), pose) ==
           mani_success;
}

int Compare(const Run& run, long rounds)
{
    Pose inputs[kMaxLayers];
    Pose additive[2];
    Pose rest;
    Pose out;
    for (Pose& p : inputs)
    {
        p.Init(run.skeleton);
    }
    for (Pose& p : additive)
    {
        p.Init(run.skeleton);
    }
    if (!rest.Init(run.skeleton) || !out.Init(run.skeleton))
    {
        return Fail("poses");
    }
    Soa soa[kMaxLayers];
    Soa addSoa[2];
    Soa ozzOut(run.ozz->num_soa_joints());
    std::vector<float> weights[kMaxLayers + 2];
    std::vector<ozz::math::SimdFloat4> soaWeights[kMaxLayers + 2];
    Apart blend;
    Apart added;
    for (long round = 0; round < rounds; ++round)
    {
        uint32_t count = 1 + Next() % kMaxLayers;
        maniBlendLayer layers[kMaxLayers];
        ozz::animation::BlendingJob::Layer ozzLayers[kMaxLayers];
        // A total weight below the threshold in one round of eight.
        float scale = Next() % 8 == 0 ? 0.02f : 1.0f;
        for (uint32_t i = 0; i < count; ++i)
        {
            if (!SampleRandom(run, inputs[i].pose))
            {
                return Fail("sampling");
            }
            ToSoa(inputs[i].pose, run.bones, &soa[i]);
            bool masked = Next() % 2 == 0;
            weights[i].assign(run.bones, 1.0f);
            for (uint32_t b = 0; masked && b < run.bones; ++b)
            {
                weights[i][b] = Next() % 4 == 0 ? 0.0f : Uniform(0.0f, 1.0f);
            }
            ToSoaWeights(weights[i], &soaWeights[i]);
            float w = scale * Uniform(0.0f, 1.0f);
            layers[i] = {inputs[i].pose, w, masked ? weights[i].data() : nullptr};
            ozzLayers[i].weight = w;
            ozzLayers[i].transform = ozz::make_span(soa[i]);
            if (masked)
            {
                ozzLayers[i].joint_weights = ozz::make_span(soaWeights[i]);
            }
        }
        // Additive layers: differences of sampled poses from the rest.
        uint32_t adds = Next() % 3;
        maniBlendLayer addLayers[2];
        ozz::animation::BlendingJob::Layer ozzAdds[2];
        for (uint32_t i = 0; i < adds; ++i)
        {
            if (!SampleRandom(run, additive[i].pose) ||
                maniPoseSetRest(rest.pose, run.skeleton) != mani_success ||
                maniPoseDifference(run.skeleton, additive[i].pose, rest.pose, additive[i].pose) !=
                    mani_success)
            {
                return Fail("differences");
            }
            ToSoa(additive[i].pose, run.bones, &addSoa[i]);
            bool masked = Next() % 2 == 0;
            std::vector<float>& aw = weights[kMaxLayers + i];
            aw.assign(run.bones, 1.0f);
            for (uint32_t b = 0; masked && b < run.bones; ++b)
            {
                aw[b] = Uniform(0.0f, 1.0f);
            }
            ToSoaWeights(aw, &soaWeights[kMaxLayers + i]);
            float w = Uniform(0.0f, 1.0f);
            addLayers[i] = {additive[i].pose, w, masked ? aw.data() : nullptr};
            ozzAdds[i].weight = w;
            ozzAdds[i].transform = ozz::make_span(addSoa[i]);
            if (masked)
            {
                ozzAdds[i].joint_weights = ozz::make_span(soaWeights[kMaxLayers + i]);
            }
        }
        ozz::animation::BlendingJob job;
        job.threshold = kThreshold;
        job.layers = ozz::span<const ozz::animation::BlendingJob::Layer>(ozzLayers, count);
        job.rest_pose = run.ozz->joint_rest_poses();
        job.output = ozz::make_span(ozzOut);
        if (!job.Run() ||
            maniBlendPoses(run.skeleton, layers, count, kThreshold, out.pose) != mani_success)
        {
            return Fail("a blend");
        }
        blend.Add(out.pose, ozzOut, run.bones);
        if (adds > 0)
        {
            job.additive_layers =
                ozz::span<const ozz::animation::BlendingJob::Layer>(ozzAdds, adds);
            if (!job.Run() || maniAddPoses(run.skeleton, out.pose, addLayers, adds) != mani_success)
            {
                return Fail("additive layers");
            }
            added.Add(out.pose, ozzOut, run.bones);
        }
    }
    std::printf("%ld rounds: blends apart by translation %.3g, rotation %.3g, scale %.3g; with "
                "additive layers by %.3g, %.3g, %.3g\n",
                rounds, blend.translation, blend.rotation, blend.scale, added.translation,
                added.rotation, added.scale);
    // ozz's estimates are refined to binary32 rounding on this file; the
    // bounds hold both to a few units in the last place.
    bool within = blend.translation < 2e-6 && blend.scale < 2e-6 && blend.rotation < 2e-6 &&
                  added.translation < 4e-6 && added.scale < 4e-6 && added.rotation < 4e-6;
    return within ? 0 : 2;
}

double Seconds()
{
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

void Timings(const Run& run, long reps, int only)
{
    Pose a;
    Pose b;
    Pose out;
    a.Init(run.skeleton);
    b.Init(run.skeleton);
    out.Init(run.skeleton);
    SampleRandom(run, a.pose);
    SampleRandom(run, b.pose);
    Soa sa;
    Soa sb;
    Soa so(run.ozz->num_soa_joints());
    ToSoa(a.pose, run.bones, &sa);
    ToSoa(b.pose, run.bones, &sb);
    maniBlendLayer layers[2] = {{a.pose, 0.3f, nullptr}, {b.pose, 0.7f, nullptr}};
    ozz::animation::BlendingJob::Layer ozzLayers[2];
    ozzLayers[0].weight = 0.3f;
    ozzLayers[0].transform = ozz::make_span(sa);
    ozzLayers[1].weight = 0.7f;
    ozzLayers[1].transform = ozz::make_span(sb);
    ozz::animation::BlendingJob job;
    job.threshold = kThreshold;
    job.layers = ozz::make_span(ozzLayers);
    job.rest_pose = run.ozz->joint_rest_poses();
    job.output = ozz::make_span(so);
    double start = Seconds();
    for (long r = 0; only != 1 && r < reps; ++r)
    {
        job.Run();
    }
    double ozzTime = Seconds() - start;
    start = Seconds();
    for (long r = 0; only != 0 && r < reps; ++r)
    {
        (void)maniBlendPoses(run.skeleton, layers, 2, kThreshold, out.pose);
    }
    double ourTime = Seconds() - start;
    std::printf("ns a blend of two poses: ozz %.0f, Maul Anim %.0f\n", ozzTime / reps * 1e9,
                ourTime / reps * 1e9);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        return Fail("usage: ozz_differential <file.fbx> [rounds [repetitions [side]]]");
    }
    long rounds = argc > 2 ? std::atol(argv[2]) : 20000;
    long reps = argc > 3 ? std::atol(argv[3]) : 1000000;
    int only = argc > 4 ? std::atoi(argv[4]) : -1;
    Bytes file;
    if (!ReadFile(argv[1], &file))
    {
        return Fail("cannot read the file");
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = nullptr;
    maniFbx* fbx = nullptr;
    maniFbxOptions options = maniDefaultFbxOptions();
    options.unitMeters = 1.0;
    options.foldPlacement = 1;
    Bytes skeletonFile;
    maniSkeleton skeleton{};
    if (maniCreateBuilder(&def, &builder) != mani_success ||
        maniOpenFbx(builder, file.data(), file.size, &fbx).result != mani_success ||
        maniFbxSkeletonCount(fbx) == 0 ||
        maniFbxBuildSkeleton(builder, fbx, 0, &options).result != mani_success ||
        !CopyBuilt(builder, &skeletonFile) ||
        maniOpenSkeleton(skeletonFile.data(), skeletonFile.size, &skeleton).result != mani_success)
    {
        return Fail("the skeleton does not import");
    }
    Run run;
    run.skeleton = &skeleton;
    run.bones = maniSkeletonBoneCount(&skeleton);
    run.ozz = MakeOzzSkeleton(&skeleton);
    if (run.ozz == nullptr)
    {
        return Fail("ozz's skeleton differs");
    }
    std::vector<Bytes> files(maniFbxClipCount(fbx));
    run.clips.resize(files.size());
    maniImportClipOptions clipOptions = maniDefaultImportClipOptions();
    clipOptions.sampleRate = kRate;
    for (uint32_t k = 0; k < files.size(); ++k)
    {
        if (maniFbxBuildClip(builder, fbx, k, &skeleton, &options, &clipOptions, nullptr).result !=
                mani_success ||
            !CopyBuilt(builder, &files[k]) ||
            maniOpenClip(files[k].data(), files[k].size, &run.clips[k]).result != mani_success)
        {
            return Fail("a clip does not import");
        }
    }
    std::printf("%u bones, %zu clips\n", run.bones, run.clips.size());
    int status = Compare(run, rounds);
    if (reps > 0)
    {
        Timings(run, reps, only);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return status;
}
