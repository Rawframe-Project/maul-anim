// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ACL differential (mani-0019): an FBX file's clips, imported by Maul
// Anim at 30 samples a second in metres, each handed as the same samples
// to ACL (its default settings, its transform metric) and to
// maniCompressClip, with the same precision, shell distance and rest
// values. Printed per clip and in total: sizes, the largest error of
// each by one shell metric (binary64, the bones' own shells, every
// sample), ACL's own error for ACL, the compression times, and the
// nanoseconds a pose takes at random times and played forward (Maul Anim
// with its cursor). The one C++ program; built with
// MAUL_ANIM_DIFFERENTIAL against ACL and RTM at the commits CMake pins.
//
// usage: acl_differential <file.fbx> [repetitions [precision in metres
//        [loop]]]: loop 0 to 4 times only that one (ACL random, ACL
//        forward, Maul Anim random, its cursor forward, full precision
//        forward), for counting instructions.

#include <acl/compression/compress.h>
#include <acl/compression/track_array.h>
#include <acl/compression/track_error.h>
#include <acl/compression/transform_error_metrics.h>
#include <acl/core/ansi_allocator.h>
#include <acl/decompression/decompress.h>

extern "C"
{
#include "clip_view.h"
}

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
#include <vector>

namespace
{

constexpr float kRate = 30.0f;
float kPrecision = 0.0001f;
constexpr float kShell = 0.03f;

double D(float v)
{
    return static_cast<double>(v);
}

double Now()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// A file's bytes, 16-byte aligned.
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
    out->size = size;
    out->words.assign(size / sizeof(std::max_align_t) + 1, std::max_align_t{});
    return maniCopyBuilt(builder, out->words.data(), size, nullptr) == mani_success;
}

// A pose in caller memory.
struct Pose
{
    std::vector<std::max_align_t> words;
    maniPose* pose = nullptr;

    explicit Pose(const maniSkeleton* skeleton)
    {
        size_t size = maniPoseSize(skeleton) + MANI_POSE_ALIGNMENT;
        words.resize(size / sizeof(std::max_align_t) + 1);
        auto at = reinterpret_cast<uintptr_t>(words.data());
        at += (MANI_POSE_ALIGNMENT - at % MANI_POSE_ALIGNMENT) % MANI_POSE_ALIGNMENT;
        if (maniInitPose(skeleton, reinterpret_cast<void*>(at), maniPoseSize(skeleton), &pose) !=
            mani_success)
        {
            std::abort();
        }
    }
};

// The shell metric in binary64: points at the shell distance on each
// bone's axes, carried through the hierarchy.
struct Matrix
{
    double m[12];
};

Matrix Local(const rtm::qvvf& t)
{
    double x = D(rtm::quat_get_x(t.rotation));
    double y = D(rtm::quat_get_y(t.rotation));
    double z = D(rtm::quat_get_z(t.rotation));
    double w = D(rtm::quat_get_w(t.rotation));
    double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + w * z),     2 * (x * z - w * y),
                   2 * (x * y - w * z),     1 - 2 * (x * x + z * z), 2 * (y * z + w * x),
                   2 * (x * z + w * y),     2 * (y * z - w * x),     1 - 2 * (x * x + y * y)};
    double s[3] = {D(rtm::vector_get_x(t.scale)), D(rtm::vector_get_y(t.scale)),
                   D(rtm::vector_get_z(t.scale))};
    Matrix out{};
    for (int c = 0; c < 3; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            out.m[3 * c + k] = r[3 * c + k] * s[c];
        }
    }
    out.m[9] = D(rtm::vector_get_x(t.translation));
    out.m[10] = D(rtm::vector_get_y(t.translation));
    out.m[11] = D(rtm::vector_get_z(t.translation));
    return out;
}

Matrix Mul(const Matrix& a, const Matrix& b)
{
    Matrix out{};
    for (int c = 0; c < 4; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            double v =
                a.m[k] * b.m[3 * c] + a.m[3 + k] * b.m[3 * c + 1] + a.m[6 + k] * b.m[3 * c + 2];
            out.m[3 * c + k] = c == 3 ? v + a.m[9 + k] : v;
        }
    }
    return out;
}

void Models(const std::vector<uint32_t>& parents, const std::vector<rtm::qvvf>& locals,
            std::vector<Matrix>* out)
{
    out->resize(locals.size());
    for (size_t b = 0; b < locals.size(); ++b)
    {
        Matrix local = Local(locals[b]);
        (*out)[b] = parents[b] == MANI_NO_BONE ? local : Mul((*out)[parents[b]], local);
    }
}

double ShellError(const Matrix& a, const Matrix& b)
{
    double worst = 0;
    for (int axis = 0; axis < 3; ++axis)
    {
        double squared = 0;
        for (int k = 0; k < 3; ++k)
        {
            double d =
                (a.m[3 * axis + k] - b.m[3 * axis + k]) * D(kShell) + a.m[9 + k] - b.m[9 + k];
            squared += d * d;
        }
        worst = std::sqrt(squared) > worst ? std::sqrt(squared) : worst;
    }
    return worst;
}

rtm::qvvf ToQvv(const maniTransform& t)
{
    return rtm::qvv_set(rtm::quat_set(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                        rtm::vector_set(t.translation.x, t.translation.y, t.translation.z),
                        rtm::vector_set(t.scale.x, t.scale.y, t.scale.z));
}

// ACL's output, default sub-tracks written as the rest (as Maul Anim
// writes them), not as ACL's default identity.
struct Writer : acl::track_writer
{
    rtm::qvvf* out = nullptr;
    const rtm::qvvf* rest = nullptr;
    static constexpr acl::default_sub_track_mode get_default_rotation_mode()
    {
        return acl::default_sub_track_mode::variable;
    }
    static constexpr acl::default_sub_track_mode get_default_translation_mode()
    {
        return acl::default_sub_track_mode::variable;
    }
    static constexpr acl::default_sub_track_mode get_default_scale_mode()
    {
        return acl::default_sub_track_mode::variable;
    }
    rtm::quatf RTM_SIMD_CALL get_variable_default_rotation(uint32_t i) const
    {
        return rest[i].rotation;
    }
    rtm::vector4f RTM_SIMD_CALL get_variable_default_translation(uint32_t i) const
    {
        return rest[i].translation;
    }
    rtm::vector4f RTM_SIMD_CALL get_variable_default_scale(uint32_t i) const
    {
        return rest[i].scale;
    }
    void RTM_SIMD_CALL write_rotation(uint32_t i, rtm::quatf_arg0 r)
    {
        out[i].rotation = r;
    }
    void RTM_SIMD_CALL write_translation(uint32_t i, rtm::vector4f_arg0 t)
    {
        out[i].translation = t;
    }
    void RTM_SIMD_CALL write_scale(uint32_t i, rtm::vector4f_arg0 s)
    {
        out[i].scale = s;
    }
};

using AclContext = acl::decompression_context<acl::default_transform_decompression_settings>;

// One clip: its full and compressed files, ACL's tracks, the measures.
struct Clip
{
    Bytes full;
    Bytes packed;
    maniClip fullClip{};
    maniClip packedClip{};
    acl::compressed_tracks* acl = nullptr;
    double aclSeconds = 0;
    double ourSeconds = 0;
    double aclError = 0;
    double aclShell = 0;
    double ourShell = 0;
    double reported = 0;
    uint32_t reportedBone = 0;
    uint32_t reportedSample = 0;
};

struct Run
{
    maniBuilder* builder = nullptr;
    const maniSkeleton* skeleton = nullptr;
    std::vector<uint32_t> parents;
    std::vector<rtm::qvvf> rest;
    acl::ansi_allocator allocator;
};

// The full clip's samples as ACL's raw tracks.
void RawTracks(Run& run, const maniClip& clip, acl::track_array_qvvf* raw)
{
    uint32_t bones = maniSkeletonBoneCount(run.skeleton);
    uint32_t samples = maniClipSampleCount(&clip);
    *raw = acl::track_array_qvvf(run.allocator, bones);
    for (uint32_t b = 0; b < bones; ++b)
    {
        acl::track_desc_transformf desc;
        desc.output_index = b;
        desc.parent_index =
            run.parents[b] == MANI_NO_BONE ? acl::k_invalid_track_index : run.parents[b];
        desc.precision = kPrecision;
        desc.shell_distance = kShell;
        desc.default_value = ToQvv(maniSkeletonRest(run.skeleton, b));
        (*raw)[b] = acl::track_qvvf::make_reserve(desc, run.allocator, samples, kRate);
    }
    Pose pose(run.skeleton);
    for (uint32_t k = 0; k < samples; ++k)
    {
        maniSampleClipAt(&clip, run.skeleton, k, pose.pose, nullptr);
        for (uint32_t b = 0; b < bones; ++b)
        {
            rtm::qvvf t = ToQvv(maniPoseLocal(pose.pose, b));
            (*raw)[b][k] = rtm::qvv_set(rtm::quat_normalize(t.rotation), t.translation, t.scale);
        }
    }
}

// The largest shell error of ACL's and Maul Anim's playback at the
// samples, against the full clip.
void Measure(Run& run, Clip& c)
{
    uint32_t bones = maniSkeletonBoneCount(run.skeleton);
    Pose full(run.skeleton);
    Pose packed(run.skeleton);
    std::vector<rtm::qvvf> reference(bones);
    std::vector<rtm::qvvf> ours(bones);
    std::vector<rtm::qvvf> theirs(bones);
    std::vector<Matrix> mr;
    std::vector<Matrix> mo;
    std::vector<Matrix> mt;
    AclContext context;
    context.initialize(*c.acl);
    Writer writer;
    writer.out = theirs.data();
    writer.rest = run.rest.data();
    for (uint32_t k = 0; k < maniClipSampleCount(&c.fullClip); ++k)
    {
        maniSampleClipAt(&c.fullClip, run.skeleton, k, full.pose, nullptr);
        maniSampleClipAt(&c.packedClip, run.skeleton, k, packed.pose, nullptr);
        context.seek(static_cast<float>(k) / kRate, acl::sample_rounding_policy::nearest);
        context.decompress_tracks(writer);
        for (uint32_t b = 0; b < bones; ++b)
        {
            reference[b] = ToQvv(maniPoseLocal(full.pose, b));
            ours[b] = ToQvv(maniPoseLocal(packed.pose, b));
        }
        Models(run.parents, reference, &mr);
        Models(run.parents, ours, &mo);
        Models(run.parents, theirs, &mt);
        for (uint32_t b = 0; b < bones; ++b)
        {
            c.ourShell = std::fmax(c.ourShell, ShellError(mr[b], mo[b]));
            c.aclShell = std::fmax(c.aclShell, ShellError(mr[b], mt[b]));
        }
    }
}

bool CompressBoth(Run& run, Clip& c)
{
    acl::track_array_qvvf raw;
    RawTracks(run, c.fullClip, &raw);
    acl::compression_settings settings = acl::get_default_compression_settings();
    acl::qvvf_transform_error_metric metric;
    settings.error_metric = &metric;
    acl::output_stats stats;
    double t0 = Now();
    acl::error_result r = acl::compress_track_list(run.allocator, raw, settings, c.acl, stats);
    c.aclSeconds = Now() - t0;
    if (r.any())
    {
        std::printf("ACL refused: %s\n", r.c_str());
        return false;
    }
    AclContext context;
    context.initialize(*c.acl);
    c.aclError = D(acl::calculate_compression_error(run.allocator, raw, context, metric).error);
    maniCompressOptions options = maniDefaultCompressOptions();
    options.precision = kPrecision;
    options.shellDistance = kShell;
    maniCompressReport report{};
    t0 = Now();
    maniResult ours = maniCompressClip(run.builder, &c.fullClip, run.skeleton, &options, &report);
    c.ourSeconds = Now() - t0;
    c.reported = report.shellError;
    c.reportedBone = report.shellBone;
    c.reportedSample = report.shellSample;
    return ours == mani_success && CopyBuilt(run.builder, &c.packed) &&
           maniOpenClip(c.packed.data(), c.packed.size, &c.packedClip).result == mani_success;
}

// Nanoseconds a pose: random times over every clip, then forward, each
// clip in turn in 600 steps.
void Timings(Run& run, std::vector<Clip>& clips, long reps, int only)
{
    uint32_t bones = maniSkeletonBoneCount(run.skeleton);
    std::vector<rtm::qvvf> out(bones);
    Writer writer;
    writer.out = out.data();
    writer.rest = run.rest.data();
    Pose pose(run.skeleton);
    size_t cursorSize = 0;
    for (const Clip& c : clips)
    {
        size_t size = maniClipCursorSize(&c.packedClip);
        cursorSize = size > cursorSize ? size : cursorSize;
    }
    std::vector<std::max_align_t> cursorWords(cursorSize / 16 + 2);
    maniClipCursor* cursor = nullptr;
    if (maniInitClipCursor(cursorWords.data(), cursorWords.size() * 16, &cursor) != mani_success)
    {
        std::abort();
    }
    AclContext context;
    double ns[6] = {0, 0, 0, 0, 0, 0};
    for (int pass = 0; pass < 2; ++pass)
    {
        std::srand(3);
        double t0 = Now();
        for (long r = 0; (only < 0 || only == 0) && r < reps; ++r)
        {
            Clip& c = clips[static_cast<size_t>(r) % clips.size()];
            context.initialize(*c.acl);
            context.seek(c.acl->get_duration() * (std::rand() / static_cast<float>(RAND_MAX)),
                         acl::sample_rounding_policy::none);
            context.decompress_tracks(writer);
        }
        double t1 = Now();
        // Forward: each clip in turn, played in 600 steps over its length.
        for (long r = 0; (only < 0 || only == 1) && r < reps; ++r)
        {
            Clip& c = clips[static_cast<size_t>(r / 600) % clips.size()];
            if (r % 600 == 0)
            {
                context.initialize(*c.acl);
            }
            context.seek(c.acl->get_duration() * static_cast<float>(r % 600) / 600.0f,
                         acl::sample_rounding_policy::none);
            context.decompress_tracks(writer);
        }
        double t2 = Now();
        std::srand(3);
        for (long r = 0; (only < 0 || only == 2) && r < reps; ++r)
        {
            Clip& c = clips[static_cast<size_t>(r) % clips.size()];
            float t =
                maniClipDuration(&c.packedClip) * (std::rand() / static_cast<float>(RAND_MAX));
            (void)maniSampleClip(&c.packedClip, run.skeleton, t, pose.pose);
        }
        double t3 = Now();
        for (long r = 0; (only < 0 || only == 3) && r < reps; ++r)
        {
            Clip& c = clips[static_cast<size_t>(r / 600) % clips.size()];
            float t = maniClipDuration(&c.packedClip) * static_cast<float>(r % 600) / 600.0f;
            (void)maniSampleClipCursor(cursor, &c.packedClip, run.skeleton, t, pose.pose);
        }
        double t4 = Now();
        for (long r = 0; (only < 0 || only == 4) && r < reps; ++r)
        {
            Clip& c = clips[static_cast<size_t>(r / 600) % clips.size()];
            float t = maniClipDuration(&c.fullClip) * static_cast<float>(r % 600) / 600.0f;
            (void)maniSampleClip(&c.fullClip, run.skeleton, t, pose.pose);
        }
        double t5 = Now();
        double scale = 1e9 / static_cast<double>(reps);
        double now[6] = {(t1 - t0) * scale, (t2 - t1) * scale, (t3 - t2) * scale,
                         (t4 - t3) * scale, (t5 - t4) * scale, 0};
        for (int i = 0; pass == 1 && i < 5; ++i)
        {
            ns[i] = now[i];
        }
    }
    std::printf("ns a pose: ACL random %.0f, forward %.0f; Maul Anim random %.0f, forward with "
                "the cursor %.0f, full precision forward %.0f\n",
                ns[0], ns[1], ns[2], ns[3], ns[4]);
}

// The bytes of a compressed clip's own sections (tags 11 to 15, forms to
// segment data), what remains being the container, header, track table,
// names and the sections' alignment.
size_t Payload(const Bytes& file)
{
    const auto* bytes = static_cast<const uint8_t*>(file.data());
    uint32_t count = 0;
    std::memcpy(&count, bytes + 12, sizeof count);
    size_t total = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t tag = 0;
        uint64_t size = 0;
        std::memcpy(&tag, bytes + 64 + 24 * i, sizeof tag);
        std::memcpy(&size, bytes + 64 + 24 * i + 16, sizeof size);
        total += tag >= 11 ? static_cast<size_t>(size) : 0;
    }
    return total;
}

int Fail(const char* what)
{
    std::printf("%s\n", what);
    return 1;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        return Fail("usage: acl_differential <file.fbx> [repetitions [precision]]");
    }
    long reps = argc > 2 ? std::atol(argv[2]) : 200000;
    kPrecision = argc > 3 ? std::strtof(argv[3], nullptr) : kPrecision;
    int only = argc > 4 ? std::atoi(argv[4]) : -1;
    Bytes file;
    if (!ReadFile(argv[1], &file))
    {
        return Fail("cannot read the file");
    }
    Run run;
    maniBuilderDef def = maniDefaultBuilderDef();
    maniFbx* fbx = nullptr;
    maniFbxOptions options = maniDefaultFbxOptions();
    options.unitMeters = 1.0;
    options.foldPlacement = 1;
    Bytes skeletonFile;
    maniSkeleton skeleton{};
    if (maniCreateBuilder(&def, &run.builder) != mani_success ||
        maniOpenFbx(run.builder, file.data(), file.size, &fbx).result != mani_success ||
        maniFbxSkeletonCount(fbx) == 0 ||
        maniFbxBuildSkeleton(run.builder, fbx, 0, &options).result != mani_success ||
        !CopyBuilt(run.builder, &skeletonFile) ||
        maniOpenSkeleton(skeletonFile.data(), skeletonFile.size, &skeleton).result != mani_success)
    {
        return Fail("the skeleton does not import");
    }
    run.skeleton = &skeleton;
    for (uint32_t b = 0; b < maniSkeletonBoneCount(&skeleton); ++b)
    {
        run.parents.push_back(maniSkeletonParent(&skeleton, b));
        run.rest.push_back(ToQvv(maniSkeletonRest(&skeleton, b)));
    }
    std::vector<Clip> clips(maniFbxClipCount(fbx));
    size_t aclBytes = 0;
    size_t ourBytes = 0;
    size_t payload = 0;
    double seconds = 0;
    double aclTime = 0;
    double ourTime = 0;
    double worst[4] = {0, 0, 0, 0};
    std::printf("%u bones, %zu clips at %.0f samples a second, %.4f m at %.2f m\n",
                maniSkeletonBoneCount(&skeleton), clips.size(), D(kRate), D(kPrecision), D(kShell));
    maniImportClipOptions clipOptions = maniDefaultImportClipOptions();
    clipOptions.sampleRate = kRate;
    for (uint32_t k = 0; k < clips.size(); ++k)
    {
        Clip& c = clips[k];
        uint32_t length = 0;
        const char* name = maniFbxClipName(fbx, k, &length);
        if (maniFbxBuildClip(run.builder, fbx, k, &skeleton, &options, &clipOptions, nullptr)
                    .result != mani_success ||
            !CopyBuilt(run.builder, &c.full) ||
            maniOpenClip(c.full.data(), c.full.size, &c.fullClip).result != mani_success ||
            !CompressBoth(run, c))
        {
            return Fail("a clip does not import or compress");
        }
        Measure(run, c);
        std::printf("%-28.*s %6.2f s  ACL %7zu B  Maul Anim %7zu B  error (mm) ACL %.4f (its "
                    "metric %.4f) Maul Anim %.4f (reported %.4f)  compression (ms) ACL %.1f Maul "
                    "Anim %.1f\n",
                    static_cast<int>(length), name, D(maniClipDuration(&c.fullClip)),
                    static_cast<size_t>(c.acl->get_size()), c.packed.size, c.aclShell * 1e3,
                    c.aclError * 1e3, c.ourShell * 1e3, c.reported * 1e3, c.aclSeconds * 1e3,
                    c.ourSeconds * 1e3);
        if (c.ourShell > static_cast<double>(kPrecision))
        {
            uint32_t n = 0;
            const char* bone = maniSkeletonBoneName(&skeleton, c.reportedBone, &n);
            std::printf("  past the precision at bone %u (%.*s), sample %u\n", c.reportedBone,
                        static_cast<int>(n), bone, c.reportedSample);
        }
        aclBytes += c.acl->get_size();
        ourBytes += c.packed.size;
        payload += Payload(c.packed);
        seconds += D(maniClipDuration(&c.fullClip));
        aclTime += c.aclSeconds;
        ourTime += c.ourSeconds;
        worst[0] = std::fmax(worst[0], c.aclShell);
        worst[1] = std::fmax(worst[1], c.aclError);
        worst[2] = std::fmax(worst[2], c.ourShell);
        worst[3] = std::fmax(worst[3], std::fabs(c.ourShell - c.reported));
    }
    std::printf("total %.1f s: ACL %zu B (%.0f B/s), Maul Anim %zu B (%.0f B/s, %.3f of ACL's); "
                "largest error (mm) ACL %.4f (its metric %.4f), Maul Anim %.4f; report off by %g; "
                "compression ACL %.2f s, Maul Anim %.2f s (%.2f of ACL's)\n",
                seconds, aclBytes, aclBytes / seconds, ourBytes, ourBytes / seconds,
                static_cast<double>(ourBytes) / static_cast<double>(aclBytes), worst[0] * 1e3,
                worst[1] * 1e3, worst[2] * 1e3, worst[3], aclTime, ourTime, ourTime / aclTime);
    std::printf("Maul Anim's compressed sections %zu B (%.3f of ACL's), the rest %zu B\n", payload,
                static_cast<double>(payload) / static_cast<double>(aclBytes), ourBytes - payload);
    if (reps > 0)
    {
        Timings(run, clips, reps, only);
    }
    for (Clip& c : clips)
    {
        run.allocator.deallocate(c.acl, c.acl->get_size());
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(run.builder);
    return worst[2] <= static_cast<double>(kPrecision) ? 0 : 2;
}
