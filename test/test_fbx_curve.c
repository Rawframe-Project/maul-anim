// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX curves (mani-0016): each interpolation, tangent kind and
// extrapolation against values derived from the Bézier and the rules of
// research 24 by hand; ASCII attribute words before 7.2 as floats; the
// tick unit; and refused curves.

#include "fbx_curve.h"
#include "fbx_document.h"
#include "fbx_object.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Curves keyed in seconds (FBX ticks in the file): 10 linear to 10,
// constant at 10, constant taking the next key's 8; 11 flat user
// tangents; 12 user tangents of slope 2 weighted 0.5; 13 auto tangents
// at 0 1 3; 14 a TCB key of tension 0.5 between linear ones; 15 and 16
// from 0 to 2 over a second with each extrapolation.
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " AnimationCurve: 10, \"AnimCurve::\", \"\" {\n  KeyTime: *4 {\n   a: "
    "0,46186158000,92372316000,138558474000\n  }\n  KeyValueFloat: *4 {\n   a: 0,10,4,8\n  }\n  "
    "KeyAttrFlags: *4 {\n   a: 4,2,258,4\n  }\n  KeyAttrDataFloat: *16 {\n   a: "
    "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n  }\n  KeyAttrRefCount: *4 {\n   a: 1,1,1,1\n  }\n }\n"
    " AnimationCurve: 11, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   a: 1032\n  }\n  "
    "KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurve: 12, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *2 {\n   a: 50332680,1032\n  }\n  "
    "KeyAttrDataFloat: *8 {\n   a: 1073741824,1073741824,327685000,0,0,0,0,0\n  }\n  "
    "KeyAttrRefCount: *2 {\n   a: 1,1\n  }\n }\n"
    " AnimationCurve: 13, \"AnimCurve::\", \"\" {\n  KeyTime: *3 {\n   a: "
    "0,46186158000,138558474000\n  }\n  KeyValueFloat: *3 {\n   a: 0,1,0\n  }\n  KeyAttrFlags: *1 "
    "{\n   a: 8\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: "
    "3\n  }\n }\n"
    " AnimationCurve: 14, \"AnimCurve::\", \"\" {\n  KeyTime: *3 {\n   a: "
    "0,46186158000,138558474000\n  }\n  KeyValueFloat: *3 {\n   a: 0,1,1\n  }\n  KeyAttrFlags: *3 "
    "{\n   a: 4,520,4\n  }\n  KeyAttrDataFloat: *12 {\n   a: 0,0,0,0,1056964608,0,0,0,0,0,0,0\n  "
    "}\n  KeyAttrRefCount: *3 {\n   a: 1,1,1\n  }\n }\n"
    " AnimationCurve: 15, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,2\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n  Pre-Extrapolation: {\n   "
    "Type: 77\n   Repetition: -1\n  }\n  Post-Extrapolation: {\n   Type: 65\n   Repetition: -1\n  "
    "}\n }\n"
    " AnimationCurve: 16, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,2\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n  Pre-Extrapolation: {\n   "
    "Type: 75\n  }\n  Post-Extrapolation: {\n   Type: 82\n   Repetition: 1\n  }\n }\n"
    "}\n";

static char s_text[8192];

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// Reads curve id of the text, or reports the refusal.
static maniImportResult Read(const char* text, int64_t id, maniFbxCurve* curve)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, text, strlen(text), &fbx);
    CHECK(r.result == mani_success, "opens");
    if (r.result == mani_success)
    {
        r = maniFbxReadCurve(fbx, maniFbxFindObject(fbx, id), curve);
        // The keys outlive the document for the checks: copied out.
        static maniFbxKey s_keys[16];
        uint32_t n = curve->count < 16 ? curve->count : 16;
        if (curve->keys != NULL)
        {
            memcpy(s_keys, curve->keys, n * sizeof *s_keys);
        }
        maniFbxReleaseCurve(fbx, curve);
        curve->keys = s_keys;
        curve->count = n;
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return r;
}

static double At(int64_t id, double time)
{
    maniFbxCurve curve;
    CHECK(Read(kScene, id, &curve).result == mani_success, "reads");
    return maniFbxEvaluateCurve(&curve, time, -99.0);
}

static bool Near(double a, double b, double tolerance)
{
    return fabs(a - b) <= tolerance;
}

static void TestInterpolations(void)
{
    CHECK(At(10, 0.5) == 5.0 && At(10, 1.0) == 10.0 && At(10, 1.5) == 10.0, "linear, constant");
    CHECK(At(10, 2.0) == 4.0 && At(10, 2.5) == 8.0 && At(10, 3.0) == 8.0, "constant next");
    CHECK(At(10, -1.0) == 0.0 && At(10, 4.0) == 8.0, "held past the ends");
    // Flat handles a third long: x(t) is t to float rounding, y the
    // smoothstep.
    CHECK(At(11, 0.5) == 0.5 && Near(At(11, 0.25), 0.15625, 1e-6), "flat user tangents");
    // Handles (0.5, 1) and (0.5, -1): y = 3u^2 t + t^3 where
    // 1.5t - 1.5t^2 + t^3 is the time.
    CHECK(Near(At(12, 0.25), 0.39410745697498223, 1e-6), "weighted tangents");
    // The middle key's slope: half the chord's (0) and half the
    // time-weighted one-sided slopes (2/3 of 1, 1/3 of -1/2): 1/4; the
    // first key's the one-sided 1. y: 0, 1/3, 11/12, 1.
    CHECK(Near(At(13, 0.5), 0.59375, 1e-6), "auto tangents");
    // The last key's left slope one-sided: -1/2. y over 1 to 3: 1, 7/6,
    // 1/3, 0.
    CHECK(Near(At(13, 2.0), 0.6875, 1e-6), "an end key's auto tangent");
    // Tension 0.5 halves Catmull-Rom's half: slope 1/4 out of the key,
    // flat into the next. y: 1, 7/6, 1, 1 over 1 to 3.
    CHECK(Near(At(14, 2.0), 1.0625, 1e-6), "TCB tangents");
}

static void TestExtrapolations(void)
{
    CHECK(At(15, -0.25) == 0.5 && At(15, -1.25) == 1.5, "mirrored before");
    CHECK(At(15, 1.5) == 3.0 && At(15, 2.25) == 4.5, "repeated relative after");
    CHECK(At(16, -1.0) == -2.0, "along the slope before");
    CHECK(At(16, 1.5) == 1.0 && At(16, 3.5) == 2.0, "repeated once after, then held");
}

static void Replace(const char* from, const char* to)
{
    const char* at = strstr(kScene, from);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)(at - kScene), kScene, to, at + strlen(from));
}

static void ExpectRefusal(int64_t id, maniImportReason reason, const char* at, const char* what)
{
    maniFbxCurve curve;
    maniImportResult r = Read(s_text, id, &curve);
    const char* where = strstr(s_text, at);
    bool right = r.result == mani_errorFormat && r.reason == reason && where != NULL &&
                 r.offset == (uint64_t)(where - s_text);
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu item %u\n", r.result, r.reason,
               (unsigned long long)r.offset, r.item);
    }
}

static void TestRefusals(void)
{
    Replace("KeyValueFloat: *4 {\n   a: 0,10,4,8", "KeyValueFloat: *3 {\n   a: 0,10,4");
    ExpectRefusal(10, maniImportReasonProperty, "AnimationCurve: 10", "values not one a key");
    Replace("KeyAttrFlags: *4 {\n   a: 4,2,258,4", "KeyAttrFlags: *3 {\n   a: 4,2,258");
    ExpectRefusal(10, maniImportReasonProperty, "AnimationCurve: 10", "flags not one a run");
    Replace("a: 1,1,1,1", "a: 1,1,0,1");
    ExpectRefusal(10, maniImportReasonProperty, "AnimationCurve: 10", "an empty run");
    Replace("a: 1,1,1,1", "a: 1,1,1,0");
    ExpectRefusal(10, maniImportReasonProperty, "AnimationCurve: 10", "runs short of the keys");
    Replace("a: 0,46186158000,92372316000", "a: 0,92372316000,46186158000");
    ExpectRefusal(10, maniImportReasonProperty, "AnimationCurve: 10", "times going back");
    Replace("KeyAttrRefCount: *4", "KeyAttrRefCounts: *4");
    ExpectRefusal(10, maniImportReasonProperty, "AnimationCurve: 10", "an array missing");
    Replace("a: 0,10,4,8", "a: 0,10,4,1e999");
    ExpectRefusal(10, maniImportReasonData, "1e999", "a value past binary64");
}

// Before 7.2 an ASCII file writes the attribute words as floats.
static void TestOldWords(void)
{
    Replace("7.4.0", "7.1.0");
    char* at = strstr(s_text, "a: 1073741824,1073741824,327685000");
    CHECK(at != NULL, "the weighted curve");
    if (at != NULL)
    {
        // 327685000's bits as a float.
        float weights = 0.0f;
        uint32_t bits = 327685000u;
        memcpy(&weights, &bits, sizeof weights);
        char rest[8192];
        snprintf(rest, sizeof rest, "a: 2,2,%.9g%s", (double)weights,
                 at + strlen("a: 1073741824,1073741824,327685000"));
        snprintf(at, sizeof s_text - (size_t)(at - s_text), "%s", rest);
    }
    maniFbxCurve curve;
    CHECK(Read(s_text, 12, &curve).result == mani_success &&
              Near(maniFbxEvaluateCurve(&curve, 0.25, 0.0), 0.39410745697498223, 1e-6),
          "float words");
}

static int64_t Ticks(const char* header)
{
    char text[512];
    snprintf(text, sizeof text, "; FBX 7.4.0 project file\nFBXHeaderExtension: {\n%s}\n", header);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxSettings settings = {0};
    CHECK(maniOpenFbx(builder, text, strlen(text), &fbx).result == mani_success &&
              maniFbxReadSettings(fbx, &settings) == mani_success,
          "a header");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return settings.ticksPerSecond;
}

static void TestTicks(void)
{
    const char* newer = " FBXHeaderVersion: 1004\n OtherFlags: {\n  TCDefinition: 0\n }\n";
    const char* older = " FBXHeaderVersion: 1003\n OtherFlags: {\n  TCDefinition: 0\n }\n";
    const char* classic = " FBXHeaderVersion: 1004\n OtherFlags: {\n  TCDefinition: 127\n }\n";
    CHECK(Ticks(newer) == 141120000 && Ticks(older) == 46186158000 &&
              Ticks(classic) == 46186158000 && Ticks(" FBXHeaderVersion: 1004\n") == 46186158000,
          "the tick unit");
}

int main(void)
{
    TestInterpolations();
    TestExtrapolations();
    TestRefusals();
    TestOldWords();
    TestTicks();
    return s_failures == 0 ? 0 : 1;
}
