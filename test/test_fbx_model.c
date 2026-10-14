// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX models (mani-0014): parents from `OO` connections only, children
// in connection order with unconnected models under the root, bone
// names renamed when empty or repeated, the 6.x form, the public
// queries, and the refusals of two parents and of a cycle.

#include "fbx_document.h"
#include "fbx_model.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stdio.h>
#include <string.h>

static const char kSeven[] = "; FBX 7.4.0 project file\n"
                             "Objects: {\n"
                             " Model: 100, \"Model::hip\", \"LimbNode\" {\n }\n"
                             " NodeAttribute: 150, \"NodeAttribute::\", \"Skeleton\" {\n }\n"
                             " Model: 200, \"Model::spine\", \"LimbNode\" {\n }\n"
                             " Model: 300, \"Model::leg\", \"LimbNode\" {\n }\n"
                             " Model: 400, \"Model::spine\", \"Null\" {\n }\n"
                             " Model: 500, \"Model::\", \"Null\" {\n }\n"
                             " Model: 600, \"Model::lone\", \"Mesh\" {\n }\n"
                             "}\n"
                             "Connections: {\n"
                             " C: \"OO\",100,0\n"
                             " C: \"OO\",150,100\n"
                             " C: \"OO\",300,100\n"
                             " C: \"OO\",200,100\n"
                             " C: \"OO\",200,100\n"
                             " C: \"OP\",400,100, \"Lcl Translation\"\n"
                             " C: \"OO\",400,200\n"
                             " C: \"OO\",500,0\n"
                             "}\n";

static const char kSix[] = "; FBX 6.1.0 project file\n"
                           "Objects: {\n"
                           " Model: \"Model::arm\", \"Limb\" {\n }\n"
                           " Model: \"Model::hip\", \"Limb\" {\n }\n"
                           "}\n"
                           "Connections: {\n"
                           " Connect: \"OO\", \"Model::hip\", \"Model::Scene\"\n"
                           " Connect: \"OO\", \"Model::arm\", \"Model::hip\"\n"
                           "}\n";

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static bool NameIs(const maniFbx* fbx, uint32_t model, const char* name)
{
    uint32_t length = 0;
    const char* bone = maniFbxModelBoneName(fbx, model, &length);
    return bone != NULL && length == strlen(name) && memcmp(bone, name, length) == 0;
}

static bool ChildrenAre(const maniFbx* fbx, uint32_t model, const uint32_t* expected, uint32_t n)
{
    const uint32_t* list = NULL;
    uint32_t count = maniFbxModelChildren(fbx, model, &list);
    return count == n && (n == 0 || memcmp(list, expected, n * sizeof(uint32_t)) == 0);
}

static void TestSeven(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kSeven, strlen(kSeven), &fbx).result == mani_success, "opens");
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniFbxModelCount(fbx) == 6, "six models; the attribute is none");
    CHECK(maniFbxModelOf(fbx, 1) == MANI_FBX_NONE && maniFbxModelOf(fbx, 2) == 1 &&
              maniFbxModelOf(fbx, 99) == MANI_FBX_NONE,
          "objects to models");
    // hip 0, spine 1, leg 2, spine 3, unnamed 4, lone 5.
    uint32_t root = fbx->modelCount;
    const uint32_t* parents = fbx->modelParents;
    CHECK(parents[0] == root && parents[1] == 0 && parents[2] == 0 && parents[3] == 1 &&
              parents[4] == root && parents[5] == root,
          "parents from object connections only");
    uint32_t hip[2] = {2, 1};
    uint32_t top[3] = {0, 4, 5};
    uint32_t spine[1] = {3};
    CHECK(ChildrenAre(fbx, 0, hip, 2), "children in connection order, a repeat once");
    CHECK(ChildrenAre(fbx, root, top, 3), "the root's, then models with no parent connection");
    CHECK(ChildrenAre(fbx, 1, spine, 1) && ChildrenAre(fbx, 5, NULL, 0), "and the rest");
    CHECK(NameIs(fbx, 0, "hip") && NameIs(fbx, 1, "spine") && NameIs(fbx, 3, "spine#3") &&
              NameIs(fbx, 4, "#4") && NameIs(fbx, 5, "lone"),
          "bone names, renamed when repeated or empty");
    CHECK(maniFbxRenamedCount(fbx) == 2 && maniFbxRenamedModel(fbx, 0) == 3 &&
              maniFbxRenamedModel(fbx, 1) == 4 && maniFbxRenamedModel(fbx, 2) == UINT32_MAX,
          "the renamed models");
    uint32_t length = 7;
    CHECK(maniFbxModelBoneName(fbx, 6, &length) == NULL && length == 0 &&
              maniFbxModelBoneName(fbx, 0, NULL) != NULL,
          "past the count; no length wanted");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    CHECK(maniFbxModelCount(NULL) == 0 && maniFbxRenamedCount(NULL) == 0 &&
              maniFbxRenamedModel(NULL, 0) == UINT32_MAX &&
              maniFbxModelBoneName(NULL, 0, &length) == NULL && length == 0,
          "NULL documents");
}

static void TestSix(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kSix, strlen(kSix), &fbx).result == mani_success, "opens");
    if (fbx != NULL)
    {
        uint32_t top[1] = {1};
        uint32_t hip[1] = {0};
        CHECK(NameIs(fbx, 0, "arm") && NameIs(fbx, 1, "hip") &&
                  ChildrenAre(fbx, fbx->modelCount, top, 1) && ChildrenAre(fbx, 1, hip, 1),
              "6.x models by name");
        maniCloseFbx(fbx);
    }
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(const char* text, const char* at, uint32_t item, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, text, strlen(text), &fbx);
    bool right = r.result == mani_errorFormat && r.reason == maniImportReasonHierarchy &&
                 r.offset == (uint64_t)(strstr(text, at) - text) && r.item == item && fbx == NULL;
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu item %u\n", (int)r.result, r.reason,
               (unsigned long long)r.offset, r.item);
    }
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    ExpectRefusal("; FBX 7.4.0 project file\n"
                  "Objects: {\n Model: 1, \"Model::a\", \"Null\" {\n }\n"
                  " Model: 2, \"Model::b\", \"Null\" {\n }\n}\n"
                  "Connections: {\n C: \"OO\",2,0\n C: \"OO\",2,1\n}\n",
                  "Model: 2", 1, "two parents");
    ExpectRefusal("; FBX 7.4.0 project file\n"
                  "Objects: {\n Model: 1, \"Model::a\", \"Null\" {\n }\n"
                  " Model: 2, \"Model::b\", \"Null\" {\n }\n"
                  " Model: 3, \"Model::c\", \"Null\" {\n }\n}\n"
                  "Connections: {\n C: \"OO\",1,0\n C: \"OO\",2,3\n C: \"OO\",3,2\n}\n",
                  "Model: 2", 1, "a cycle");
    ExpectRefusal("; FBX 7.4.0 project file\n"
                  "Objects: {\n Model: 1, \"Model::a\", \"Null\" {\n }\n}\n"
                  "Connections: {\n C: \"OO\",1,1\n}\n",
                  "Model: 1", 0, "its own parent");
}

int main(void)
{
    TestSeven();
    TestSix();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
