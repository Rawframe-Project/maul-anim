// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The base of the Maul Anim API: the library version, the export and
// attribute macros, and the result codes every fallible function
// returns.

#ifndef MAUL_ANIM_BASE_H
#define MAUL_ANIM_BASE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The library version. CMake reads it from here, and release-please
// moves it between the markers.
// x-release-please-start-major
#define MANI_VERSION_MAJOR 0
// x-release-please-end
// x-release-please-start-minor
#define MANI_VERSION_MINOR 4
// x-release-please-end
// x-release-please-start-patch
#define MANI_VERSION_PATCH 0
// x-release-please-end

// The releases that share a binary interface, which a root def's default
// function stamps into the def from the headers the program was built
// with: the whole version before 1.0, where any release may break the
// interface, and the major and minor version from 1.0 on. The top bit is
// always set, so a zeroed or hand-written small value never matches. The
// creation checks the def's cookie first and this second, refusing a
// mismatch with mani_errorVersion before it reads anything else.
#define MANI_ABI_VERSION                                                                           \
    (UINT32_C(0x80000000) | ((uint32_t)MANI_VERSION_MAJOR << 16) |                                 \
     (MANI_VERSION_MAJOR == 0                                                                      \
          ? (((uint32_t)MANI_VERSION_MINOR << 8) | (uint32_t)MANI_VERSION_PATCH)                   \
          : (uint32_t)MANI_VERSION_MINOR))
#if MANI_VERSION_MAJOR > 0x7FFF || MANI_VERSION_MINOR > 0xFFFF
#error "MANI_VERSION_MAJOR or MANI_VERSION_MINOR does not fit MANI_ABI_VERSION"
#endif
#if MANI_VERSION_MAJOR == 0 && (MANI_VERSION_MINOR > 0xFF || MANI_VERSION_PATCH > 0xFF)
#error "before 1.0, MANI_VERSION_MINOR and MANI_VERSION_PATCH must fit 8 bits each"
#endif

// MANI_API marks the public functions: dllexport or dllimport in a
// shared Windows build (maul_anim_EXPORTS is defined while building the
// library), default visibility in a shared build elsewhere.
#if defined(MAUL_ANIM_SHARED) && defined(_WIN32)
#if defined(maul_anim_EXPORTS)
#define MANI_API __declspec(dllexport) extern
#else
#define MANI_API __declspec(dllimport) extern
#endif
#elif defined(MAUL_ANIM_SHARED) && (defined(__GNUC__) || defined(__clang__))
#define MANI_API __attribute__((visibility("default"))) extern
#else
#define MANI_API extern
#endif

// MANI_NODISCARD marks a function whose result must be read: every
// function that returns a status. The attribute is standard in C23 and
// C++17 and left out for older dialects. MSVC keeps __cplusplus at
// 199711L unless /Zc:__cplusplus is given, so _MSVC_LANG is read too.
#if (defined(__cplusplus) && __cplusplus >= 201703L) ||                                            \
    (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)
#define MANI_NODISCARD [[nodiscard]]
#elif !defined(__cplusplus) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 202311L
#define MANI_NODISCARD [[nodiscard]]
#else
#define MANI_NODISCARD
#endif

    // The status a fallible function returns. Zero is success, positive
    // values are outcomes that are not errors, negative values are errors.
    // The type has a fixed width so that result structs have one layout in
    // C and C++.
    typedef int32_t maniResult;
    enum
    {
        // The call did what was asked.
        mani_success = 0,
        // An argument is invalid: a null pointer where one is required, a
        // value out of range.
        mani_errorInvalid = -1,
        // Room is too small for the result: a caller's buffer or the
        // allocator. More room would succeed.
        mani_errorCapacity = -2,
        // A file of a format version, or a def stamped by headers, this
        // library does not read.
        mani_errorVersion = -7,
        // A file's bytes break its format: the result's offset names the
        // first byte found wrong.
        mani_errorFormat = -8,
        // An argument, or a source's content, passes a fixed bound of the
        // library (a count, a depth, a rate, a magnitude), each published
        // as a MANI_MAX_* constant. More room would not help.
        mani_errorLimit = -9,
        // The object is in use by work under way: a function of an object
        // called back in from the host's code that object runs (its
        // allocator, its task hooks). The same call succeeds once that
        // work has returned.
        mani_errorBusy = -10,
    };

    // The result of reading external bytes: the status and, for
    // mani_errorFormat, the first byte found wrong: the tag of the file
    // section it lies in (each kind's tags are listed in file-formats.md;
    // 0 in a file's header, its section table and the padding between
    // sections, and anywhere in a saved graph or spring state) and its
    // offset from the start of the bytes.
    typedef struct maniFileResult
    {
        maniResult result;
        uint32_t tag;
        uint64_t offset;
    } maniFileResult;

    // The allocator an owner object takes in its def and keeps for its
    // lifetime. Alignment is a power of two. A zeroed allocator means the C
    // library's functions, which serve alignments up to that of max_align_t.
    // Its functions call no function of the object they serve: one that
    // does is refused with mani_errorBusy.
    typedef struct maniAllocator
    {
        void* (*alloc)(size_t size, size_t alignment, void* context);
        void (*free)(void* memory, size_t size, size_t alignment, void* context);
        void* context;
    } maniAllocator;

    // A range of work items, [startIndex, endIndex), run by the host on any
    // thread. The library passes taskContext through unchanged.
    typedef void maniTaskFn(int32_t startIndex, int32_t endIndex, void* taskContext);

    // Hands the host itemCount items of work: the host splits [0, itemCount)
    // into ranges of at least minRange items (the last may be shorter),
    // runs task once on each range, on any thread and in any order, and
    // returns a handle that finishTask waits on. A host that ran every range
    // before returning may return NULL, and finishTask is then not called.
    // Results never depend on the split or the order: each item writes only
    // its own slots.
    typedef void* maniEnqueueTaskFn(maniTaskFn* task, int32_t itemCount, int32_t minRange,
                                    void* taskContext, void* userContext);

    // Waits until every range of the work behind userTask has run.
    typedef void maniFinishTaskFn(void* userTask, void* userContext);

    // The host's task system, which an owner object takes in its def; both
    // hooks NULL (the default) runs every item on the calling thread.
    // enqueueTask and finishTask call no function of the object they
    // serve: one that does is refused with mani_errorBusy.
    typedef struct maniTaskHooks
    {
        maniEnqueueTaskFn* enqueueTask;
        maniFinishTaskFn* finishTask;
        // Passed to enqueueTask and finishTask as their userContext.
        void* userContext;
    } maniTaskHooks;

    // A vector or a position, binary32.
    typedef struct maniVec3
    {
        float x;
        float y;
        float z;
    } maniVec3;

    // A rotation as a unit quaternion (x, y, z, w); a * b turns by b, then
    // by a.
    typedef struct maniQuat
    {
        float x;
        float y;
        float z;
        float w;
    } maniQuat;

    // A bone's local transform: scale per axis, then rotation, then
    // translation, as glTF composes a node.
    typedef struct maniTransform
    {
        maniVec3 translation;
        maniQuat rotation;
        maniVec3 scale;
    } maniTransform;

    // An affine matrix, 3x4: its three columns (the x, y and z axes), then
    // the translation; m[0..2], m[3..5], m[6..8], m[9..11].
    typedef struct maniAffine
    {
        float m[12];
    } maniAffine;

    // A version: major, minor and patch.
    typedef struct maniVersion
    {
        uint16_t major;
        uint16_t minor;
        uint16_t patch;
    } maniVersion;

    /// Returns the version of the library that was linked, which may differ
    /// from the MANI_VERSION macros a program was compiled with.
    ///
    /// @return The library version.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniVersion maniGetVersion(void);

    /// Returns the backend the library's vector code was built for, set by
    /// the MAUL_ANIM_SIMD build option.
    ///
    /// @return "avx2", "neon" or "scalar"; a static string.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniGetSimdBackend(void);

    /// Tells whether this CPU runs the backend the library was built for.
    ///
    /// @return 1 when it does, else 0.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API int32_t maniCpuSupportsBackend(void);

    /// The CPU check a host makes once, before any other call into the
    /// library: an AVX2 build on a CPU without AVX2 and FMA would stop on
    /// its first vector instruction, so on 0 the host refuses to run or
    /// loads a scalar build. No job checks it again. This function and the
    /// two above are built without the backend's instructions and are safe
    /// to call on any CPU of the target architecture.
    ///
    /// @param backend  Receives the backend's name, as maniGetSimdBackend
    ///                 returns it. May be NULL.
    /// @return 1 when this CPU runs the backend, else 0.
    /// @par Thread safety
    /// Safe from any thread. It keeps no state.
    MANI_API int32_t maniCheckCpu(const char** backend);

    // The seed of a hash that starts from nothing.
#define MANI_HASH_INIT 0xCBF29CE484222325ull

    /// Hashes bytes into a 64-bit value, the hash the library's determinism
    /// checks are built on and a host can use to compare poses between
    /// machines: eight bytes a round, read in the host's byte order, xored
    /// in, multiplied by an odd constant and folded, the leftover bytes one
    /// at a time. Its constants are frozen.
    ///
    /// @param seed       MANI_HASH_INIT, or a hash to continue.
    /// @param data       The bytes. May be NULL when byteCount is 0.
    /// @param byteCount  The number of bytes, at least 0.
    /// @return The hash.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniHash64(uint64_t seed, const void* data, int32_t byteCount);

    /// Returns the name of a result code, for diagnostics.
    ///
    /// @param result  Any value; an unknown one is named as such.
    /// @return A static, NUL-terminated string such as "mani_errorCapacity".
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniResultName(maniResult result);

#ifdef __cplusplus
}
#endif

#endif
