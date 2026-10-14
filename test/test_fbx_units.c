// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skeletons found at open (mani-0014): skins' linked models and limb
// chains as units, a skin joining two chains under their common
// ancestor, limbs by subclass or by node attribute, a mesh below a limb
// left out, a skin linking nothing, the 6.x form, and the refusal of a
// skin whose models share no ancestor.

#include "fbx_document.h"
#include "fbx_units.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stdio.h>
#include <string.h>

// Models: 0 Armature, 1 Hips, 2 Spine, 3 Head (a limb by its
// attribute), 4 HeadProp (a mesh), 5 Body, 6 Gun, 7 Trigger, 8 Tail.
// Skins: 0 links Hips and Tail; 1 links nothing (a cluster tied to it
// by a property, a blend shape channel); 2 links Trigger (and an
// attribute that is no model). A video of subclass Skin is no skin.
static const char kRig[] = "; FBX 7.4.0 project file\n"
                           "Objects: {\n"
                           " Model: 10, \"Model::Armature\", \"Null\" {\n }\n"
                           " Model: 11, \"Model::Hips\", \"LimbNode\" {\n }\n"
                           " Model: 12, \"Model::Spine\", \"LimbNode\" {\n }\n"
                           " Model: 13, \"Model::Head\", \"Null\" {\n }\n"
                           " NodeAttribute: 30, \"NodeAttribute::\", \"LimbNode\" {\n }\n"
                           " Model: 14, \"Model::HeadProp\", \"Mesh\" {\n }\n"
                           " Model: 15, \"Model::Body\", \"Mesh\" {\n }\n"
                           " Model: 16, \"Model::Gun\", \"Null\" {\n }\n"
                           " Model: 17, \"Model::Trigger\", \"Null\" {\n }\n"
                           " Model: 18, \"Model::Tail\", \"Root\" {\n }\n"
                           " Deformer: 40, \"Deformer::\", \"Skin\" {\n }\n"
                           " Deformer: 41, \"SubDeformer::\", \"Cluster\" {\n }\n"
                           " Deformer: 42, \"SubDeformer::\", \"Cluster\" {\n }\n"
                           " Deformer: 43, \"Deformer::\", \"Skin\" {\n }\n"
                           " Deformer: 44, \"Deformer::\", \"Skin\" {\n }\n"
                           " Deformer: 45, \"SubDeformer::\", \"Cluster\" {\n }\n"
                           " Deformer: 46, \"SubDeformer::\", \"Cluster\" {\n }\n"
                           " Deformer: 47, \"SubDeformer::\", \"BlendShapeChannel\" {\n }\n"
                           " NodeAttribute: 31, \"NodeAttribute::\", \"Null\" {\n }\n"
                           " Video: 51, \"Video::\", \"Skin\" {\n }\n"
                           "}\n"
                           "Connections: {\n"
                           " C: \"OO\",10,0\n C: \"OO\",11,10\n C: \"OO\",12,11\n C: \"OO\",13,12\n"
                           " C: \"OO\",30,13\n C: \"OO\",14,13\n C: \"OO\",15,0\n C: \"OO\",16,0\n"
                           " C: \"OO\",17,16\n C: \"OO\",18,10\n"
                           " C: \"OO\",40,15\n C: \"OO\",41,40\n C: \"OO\",42,40\n"
                           " C: \"OO\",11,41\n C: \"OO\",18,42\n"
                           " C: \"OO\",45,44\n C: \"OO\",17,45\n"
                           " C: \"PO\",46, \"Visibility\",43\n C: \"OO\",15,46\n"
                           " C: \"OO\",47,43\n C: \"OO\",15,47\n C: \"OO\",31,45\n"
                           "}\n";

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static bool MarkedAre(const maniFbx* fbx, uint32_t skeleton, const char* expected)
{
    uint8_t marked[16] = {0};
    maniFbxMarkSkeleton(fbx, skeleton, marked);
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        if ((marked[m] != 0) != (expected[m] == '1'))
        {
            return false;
        }
    }
    return true;
}

static void TestRig(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kRig, strlen(kRig), &fbx).result == mani_success, "opens");
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniFbxSkinCount(fbx) == 3 && maniFbxSkeletonCount(fbx) == 2,
          "three skins, two skeletons");
    CHECK(maniFbxSkinSkeleton(fbx, 0) == 0 && maniFbxSkinSkeleton(fbx, 1) == UINT32_MAX &&
              maniFbxSkinSkeleton(fbx, 2) == 1 && maniFbxSkinSkeleton(fbx, 3) == UINT32_MAX,
          "each skin's skeleton");
    // Units: skin 0 (root Armature), skin 2 (root Trigger), the chains
    // at Hips and at Tail.
    CHECK(fbx->unitCount == 4 && fbx->unitRoots[0] == 0 && fbx->unitRoots[1] == 7 &&
              fbx->unitRoots[2] == 1 && fbx->unitRoots[3] == 8,
          "units: skins, then chains in model order");
    uint32_t hips[3] = {1, 2, 3};
    CHECK(fbx->unitStarts[3] - fbx->unitStarts[2] == 3 &&
              memcmp(fbx->unitSeeds + fbx->unitStarts[2], hips, sizeof hips) == 0,
          "a chain's seeds: its limbs, the mesh below left out");
    CHECK(fbx->skeletonRoots[0] == 0 && fbx->skeletonRoots[1] == 7,
          "the skin joins both chains under the armature");
    CHECK(MarkedAre(fbx, 0, "111100001"), "skeleton 0's models");
    CHECK(MarkedAre(fbx, 1, "000000010"), "skeleton 1's: one rigid model");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    CHECK(maniFbxSkeletonCount(NULL) == 0 && maniFbxSkinCount(NULL) == 0 &&
              maniFbxSkinSkeleton(NULL, 0) == UINT32_MAX,
          "NULL documents");
}

static void TestChains(void)
{
    // Two chains with no skin: two skeletons; a limb under a null under
    // a limb belongs to the upper chain, the null between them too.
    static const char kChains[] = "; FBX 7.4.0 project file\n"
                                  "Objects: {\n"
                                  " Model: 1, \"Model::a\", \"LimbNode\" {\n }\n"
                                  " Model: 2, \"Model::n\", \"Null\" {\n }\n"
                                  " Model: 3, \"Model::b\", \"LimbNode\" {\n }\n"
                                  " Model: 4, \"Model::c\", \"Limb\" {\n }\n"
                                  "}\n"
                                  "Connections: {\n C: \"OO\",1,0\n C: \"OO\",2,1\n"
                                  " C: \"OO\",3,2\n C: \"OO\",4,0\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kChains, strlen(kChains), &fbx).result == mani_success, "opens");
    if (fbx != NULL)
    {
        CHECK(maniFbxSkeletonCount(fbx) == 2 && fbx->skeletonRoots[0] == 0 &&
                  fbx->skeletonRoots[1] == 3,
              "two chains");
        CHECK(MarkedAre(fbx, 0, "1110") && MarkedAre(fbx, 1, "0001"),
              "the null between limbs is a bone");
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
}

static void TestSix(void)
{
    static const char kSix[] = "; FBX 6.1.0 project file\n"
                               "Objects: {\n"
                               " Model: \"Model::hip\", \"Limb\" {\n }\n"
                               " Model: \"Model::leg\", \"Limb\" {\n }\n"
                               " Model: \"Model::mesh\", \"Mesh\" {\n }\n"
                               " Deformer: \"Deformer::skin\", \"Skin\" {\n }\n"
                               " Deformer: \"SubDeformer::c\", \"Cluster\" {\n }\n"
                               "}\n"
                               "Connections: {\n"
                               " Connect: \"OO\", \"Model::hip\", \"Model::Scene\"\n"
                               " Connect: \"OO\", \"Model::leg\", \"Model::hip\"\n"
                               " Connect: \"OO\", \"Model::mesh\", \"Model::Scene\"\n"
                               " Connect: \"OO\", \"Deformer::skin\", \"Model::mesh\"\n"
                               " Connect: \"OO\", \"SubDeformer::c\", \"Deformer::skin\"\n"
                               " Connect: \"OO\", \"Model::leg\", \"SubDeformer::c\"\n"
                               "}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kSix, strlen(kSix), &fbx).result == mani_success, "opens");
    if (fbx != NULL)
    {
        CHECK(maniFbxSkinCount(fbx) == 1 && maniFbxSkeletonCount(fbx) == 1 &&
                  maniFbxSkinSkeleton(fbx, 0) == 0 && fbx->skeletonRoots[0] == 0 &&
                  MarkedAre(fbx, 0, "110"),
              "6.x skins and limbs by name");
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
}

static void TestRefusal(void)
{
    static const char kApart[] =
        "; FBX 7.4.0 project file\n"
        "Objects: {\n"
        " Model: 1, \"Model::a\", \"Null\" {\n }\n"
        " Model: 2, \"Model::b\", \"Null\" {\n }\n"
        " Deformer: 3, \"Deformer::\", \"Skin\" {\n }\n"
        " Deformer: 4, \"SubDeformer::\", \"Cluster\" {\n }\n"
        " Deformer: 5, \"SubDeformer::\", \"Cluster\" {\n }\n"
        "}\n"
        "Connections: {\n C: \"OO\",1,0\n C: \"OO\",2,0\n"
        " C: \"OO\",4,3\n C: \"OO\",5,3\n C: \"OO\",1,4\n C: \"OO\",2,5\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, kApart, strlen(kApart), &fbx);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonHierarchy &&
              r.offset == (uint64_t)(strstr(kApart, "Deformer: 3") - kApart) && r.item == 0 &&
              fbx == NULL,
          "a skin over two trees");
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestRig();
    TestChains();
    TestSix();
    TestRefusal();
    return s_failures == 0 ? 0 : 1;
}
