// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX worlds at rest: each inherit type and chains of them,
// against worlds derived by hand from ufbx's rules, under a parent of
// non-uniform scale; and a refused local stopping the evaluation.

#include "fbx_document.h"
#include "fbx_transform.h"
#include "fbx_world.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// A at the root: T (1 0 0), S (2 3 4). Under A: B RrSs, T (1 1 1),
// R z 90; C Rrs, T (1 0 0), S 5; D normal, T (1 0 0), R z 90. E Rrs
// under C, T (1 0 0); F RrSs under B, T (0 1 0).
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Model: 1, \"Model::A\", \"Null\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n"
    "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",2,3,4\n  }\n }\n"
    " Model: 2, \"Model::B\", \"Null\" {\n  Properties70: {\n"
    "   P: \"InheritType\", \"enum\", \"\", \"\",0\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,1,1\n"
    "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,90\n  }\n }\n"
    " Model: 3, \"Model::C\", \"Null\" {\n  Properties70: {\n"
    "   P: \"InheritType\", \"enum\", \"\", \"\",2\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n"
    "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",5,5,5\n  }\n }\n"
    " Model: 4, \"Model::D\", \"Null\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n"
    "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,90\n  }\n }\n"
    " Model: 5, \"Model::E\", \"Null\" {\n  Properties70: {\n"
    "   P: \"InheritType\", \"enum\", \"\", \"\",2\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n  }\n }\n"
    " Model: 6, \"Model::F\", \"Null\" {\n  Properties70: {\n"
    "   P: \"InheritType\", \"enum\", \"\", \"\",0\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n  }\n }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",6,2\n C: \"OO\",5,3\n C: \"OO\",1,0\n C: \"OO\",2,1\n"
    " C: \"OO\",3,1\n C: \"OO\",4,1\n"
    "}\n";

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static bool Near(const maniFbxMatrix* a, const double expected[12])
{
    for (int i = 0; i < 12; ++i)
    {
        if (!(fabs(a->m[i] - expected[i]) <= 1e-12))
        {
            return false;
        }
    }
    return true;
}

static void TestWorlds(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success, "opens");
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    maniFbxWorld worlds[6];
    CHECK(maniFbxEvaluateWorlds(fbx, NULL, worlds).result == mani_success, "evaluates");
    static const double kA[12] = {2, 0, 0, 0, 3, 0, 0, 0, 4, 1, 0, 0};
    // RrSs: A's unscaled world, translation and scale times A's (2 3 4).
    static const double kB[12] = {0, 2, 0, -3, 0, 0, 0, 0, 4, 3, 3, 4};
    // Rrs: A's scale ignored but in the translation.
    static const double kC[12] = {5, 0, 0, 0, 5, 0, 0, 0, 5, 3, 0, 0};
    // Normal: A's world times D's, a shear-free product here.
    static const double kD[12] = {0, 3, 0, -2, 0, 0, 0, 0, 4, 3, 0, 0};
    // Rrs under Rrs: no scale source; translation times C's scale.
    static const double kE[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 8, 0, 0};
    // RrSs under RrSs: B's inherited scale (2 3 4), translation as well.
    static const double kF[12] = {0, 2, 0, -3, 0, 0, 0, 0, 4, 0, 3, 4};
    CHECK(Near(&worlds[0].world, kA), "a normal root");
    CHECK(Near(&worlds[1].world, kB), "RrSs under a non-uniform scale");
    CHECK(Near(&worlds[2].world, kC), "Rrs ignoring its parent's scale");
    CHECK(Near(&worlds[3].world, kD), "a normal child");
    CHECK(Near(&worlds[4].world, kE), "Rrs under Rrs");
    CHECK(Near(&worlds[5].world, kF), "RrSs under RrSs");
    static const double kCUnscaled[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 3, 0, 0};
    CHECK(Near(&worlds[2].unscaled, kCUnscaled), "an unscaled world");
    CHECK(worlds[1].scaleSource == 0 && worlds[2].scaleSource == MANI_FBX_NONE &&
              worlds[4].scaleSource == MANI_FBX_NONE && worlds[5].scaleSource == 1 &&
              worlds[0].scaleSource == MANI_FBX_NONE,
          "scale sources");
    CHECK(worlds[1].inheritScale[0] == 2 && worlds[1].inheritScale[1] == 3 &&
              worlds[1].inheritScale[2] == 4 && worlds[3].inheritScale[0] == 1,
          "inherited scales");
    // The world is the unscaled world times the inherited scale.
    bool split = true;
    for (int v = 0; v < 6; ++v)
    {
        maniFbxTrs s = {{0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}};
        memcpy(s.scale, worlds[v].inheritScale, sizeof s.scale);
        maniFbxMatrix scale = maniFbxTrsMatrix(&s);
        maniFbxMatrix product = maniFbxMatrixMul(&worlds[v].unscaled, &scale);
        split = split && Near(&product, worlds[v].world.m);
    }
    CHECK(split, "each world its unscaled world times its inherited scale");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestRefusal(void)
{
    static const char kBad[] = "; FBX 7.4.0 project file\n"
                               "Objects: {\n"
                               " Model: 1, \"Model::A\", \"Null\" {\n }\n"
                               " Model: 2, \"Model::B\", \"Null\" {\n  Properties70: {\n"
                               "   P: \"RotationOrder\", \"enum\", \"\", \"\",9\n  }\n }\n"
                               "}\n"
                               "Connections: {\n C: \"OO\",2,1\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kBad, strlen(kBad), &fbx).result == mani_success, "opens");
    if (fbx != NULL)
    {
        maniFbxWorld worlds[2];
        maniImportResult r = maniFbxEvaluateWorlds(fbx, NULL, worlds);
        CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
                  r.offset == (uint64_t)(strstr(kBad, "9\n") - kBad) && r.item == 1,
              "a refused local refuses the evaluation");
        uint8_t needed[2] = {1, 0};
        CHECK(maniFbxEvaluateWorlds(fbx, needed, worlds).result == mani_success,
              "a model not needed is not evaluated");
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestWorlds();
    TestRefusal();
    return s_failures == 0 ? 0 : 1;
}
