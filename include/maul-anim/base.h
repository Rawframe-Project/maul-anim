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
    };

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
