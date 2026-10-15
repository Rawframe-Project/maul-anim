// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX files before 6.0 (mani-0017): top-level models and their Children,
// rest values from the last channel driving them in file order, the
// legacy frame rate, against what ufbx gives for the same file.

#include "test_fbx_sweep.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Hips (a limb, its own channels giving T 1 2 3) above Spine (its own
// channels turning it about z from 10); Walk's channels on Hips come
// later in the file, so Hips' rest is theirs, 5 6 0 (Z has no Default).
// Settings give 24 frames a second.
static const char kScene[] = "; FBX 5.0.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1002\n"
                             " FBXVersion: 5000\n"
                             "}\n"
                             "Model: \"Model::Hips\" {\n"
                             " Type: \"LimbNode\"\n"
                             " Children: \"Model::Spine\"\n"
                             " Channel: \"Transform\" {\n"
                             "  Channel: \"T\" {\n"
                             "   Channel: \"X\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "   Channel: \"Y\" {\n"
                             "    Default: 2\n"
                             "   }\n"
                             "   Channel: \"Z\" {\n"
                             "    Default: 3\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Model: \"Model::Spine\" {\n"
                             " Type: \"LimbNode\"\n"
                             " Channel: \"Transform\" {\n"
                             "  Channel: \"R\" {\n"
                             "   Channel: \"X\" {\n"
                             "    Default: 0\n"
                             "   }\n"
                             "   Channel: \"Y\" {\n"
                             "    Default: 0\n"
                             "   }\n"
                             "   Channel: \"Z\" {\n"
                             "    Default: 10\n"
                             "    KeyVer: 4004\n"
                             "    KeyCount: 2\n"
                             "    Key: 0,10,L,46186158000,50,L\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"Walk\" {\n"
                             "  LocalTime: 0,46186158000\n"
                             "  Model: \"Model::Hips\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 5\n"
                             "      KeyVer: 4004\n"
                             "      KeyCount: 2\n"
                             "      Key: 0,5,L,46186158000,9,L\n"
                             "     }\n"
                             "     Channel: \"Y\" {\n"
                             "      Default: 6\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Settings: {\n"
                             " FrameRate: \"24\"\n"
                             "}\n";

static _Alignas(16) uint8_t s_skeletonFile[16384];

static bool Near(double a, double b)
{
    return fabs(a - b) <= 1e-5;
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static void TestSkeleton(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, kScene, strlen(kScene), &fbx);
    CHECK(r.result == mani_success && maniFbxVersion(fbx) == 5000 && maniFbxModelCount(fbx) == 2,
          "opens with two models");
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    bool built =
        fbx != NULL && maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success;
    uint32_t hips = built ? maniSkeletonFindBone(&skeleton, "Hips", 4) : MANI_NO_BONE;
    uint32_t spine = built ? maniSkeletonFindBone(&skeleton, "Spine", 5) : MANI_NO_BONE;
    CHECK(hips != MANI_NO_BONE && spine != MANI_NO_BONE &&
              maniSkeletonParent(&skeleton, spine) == hips,
          "Children make the hierarchy");
    maniTransform h = built ? maniSkeletonRest(&skeleton, hips) : (maniTransform){0};
    maniTransform s = built ? maniSkeletonRest(&skeleton, spine) : (maniTransform){0};
    double half = 5.0 * 3.14159265358979323846 / 180.0;
    CHECK(built && Near((double)h.translation.x, 5) && Near((double)h.translation.y, 6) &&
              Near((double)h.translation.z, 0) && Near((double)s.rotation.z, sin(half)) &&
              Near((double)s.rotation.w, cos(half)),
          "rests from the last channel driving them");
    maniFbxSettings settings;
    CHECK(fbx != NULL && maniFbxReadSettings(fbx, &settings) == mani_success &&
              settings.timeMode == 14 && settings.customRate == 24.0,
          "the legacy frame rate");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// The settings of a text.
static bool Settings(const char* text, maniFbxSettings* out)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    bool read = maniOpenFbx(builder, text, strlen(text), &fbx).result == mani_success &&
                maniFbxReadSettings(fbx, out) == mani_success;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return read;
}

static void TestVersion5Settings(void)
{
    // A 6.x file's Version5 Settings, where GlobalSettings has no rate.
    maniFbxSettings s;
    CHECK(Settings("; FBX 6.1.0 project file\nVersion5: {\n Settings: {\n  FrameRate: \"25\"\n"
                   " }\n}\n",
                   &s) &&
              s.timeMode == 14 && s.customRate == 25.0,
          "Version5's frame rate");
    CHECK(Settings("; FBX 6.1.0 project file\nGlobalSettings: {\n Properties60: {\n"
                   "  Property: \"TimeMode\", \"enum\", \"\",11\n }\n}\nVersion5: {\n"
                   " Settings: {\n  FrameRate: 25\n }\n}\n",
                   &s) &&
              s.timeMode == 11 && s.rateNumerator == 24,
          "GlobalSettings' rate first");
}

// Each text refused at open (Property).
static void Refused(const char* text, const char* label)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, text, strlen(text), &fbx);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonProperty, label);
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    Refused("; FBX 5.0.0 project file\nModel: {\n}\n", "a model without a name");
    Refused("; FBX 5.0.0 project file\nModel: \"Model::Mesh\" {\n Vertices: 0,0,0\n"
            " PolygonVertexIndex: -1\n Link: {\n }\n}\n",
            "a link without a name");
    Refused("; FBX 5.0.0 project file\nModel: \"Model::A\" {\n}\nModel: \"Model::A\" {\n}\n",
            "a name given twice");
}

static void TestAllocationFailures(void)
{
    FbxSweep sweep = {kScene, UINT32_MAX, UINT32_MAX};
    CHECK(TestSweep(FbxSweepWork, &sweep, 4096), "allocation failures handled, nothing kept");
}

int main(void)
{
    TestSkeleton();
    TestVersion5Settings();
    TestRefusals();
    TestAllocationFailures();
    return s_failures == 0 ? 0 : 1;
}
