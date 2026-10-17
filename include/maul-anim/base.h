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

// The library version. CMake reads it from here.
#define MANI_VERSION_MAJOR 0
#define MANI_VERSION_MINOR 0
#define MANI_VERSION_PATCH 1

// The major and minor version a def is stamped with by its default
// function; a call taking a def built against another refuses it with
// mani_errorVersion before it reads anything else of it.
#define MANI_ABI_VERSION (((uint32_t)MANI_VERSION_MAJOR << 16) | (uint32_t)MANI_VERSION_MINOR)

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
        // A caller buffer or a named limit is too small for the result.
        mani_errorCapacity = -2,
        // A file's bytes break its format: the result's offset names the
        // first byte found wrong.
        mani_errorFormat = -3,
        // A file of a format version, or a def stamped by headers, this
        // library does not read.
        mani_errorVersion = -4,
    };

    // The result of reading external bytes: the status and, for
    // mani_errorFormat, the offset of the first byte found wrong.
    typedef struct maniFileResult
    {
        maniResult result;
        uint32_t reserved;
        uint64_t offset;
    } maniFileResult;

    // The allocator an owner object takes in its def and keeps for its
    // lifetime. Alignment is a power of two. A zeroed allocator means the C
    // library's functions, which serve alignments up to that of max_align_t.
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
    // translation, as glTF composes a node (mani-0003).
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
