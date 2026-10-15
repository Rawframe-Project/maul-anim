// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX objects, connections, properties and settings (mani-0013): a 7.x
// and a 6.x document read into objects found by id or name, their
// connections from both ends, properties falling back to templates,
// and GlobalSettings' axes, units and exact frame rates; then each
// refusal.

#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_value.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stdio.h>
#include <string.h>

static const char kSeven[] =
    "; FBX 7.4.0 project file\n"
    "FBXHeaderExtension: {\n FBXVersion: 7400\n}\n"
    "GlobalSettings: {\n Properties70: {\n"
    "  P: \"UpAxis\", \"int\", \"Integer\", \"\",2\n"
    "  P: \"UpAxisSign\", \"int\", \"Integer\", \"\",1\n"
    "  P: \"FrontAxis\", \"int\", \"Integer\", \"\",1\n"
    "  P: \"FrontAxisSign\", \"int\", \"Integer\", \"\",-1\n"
    "  P: \"CoordAxis\", \"int\", \"Integer\", \"\",0\n"
    "  P: \"CoordAxisSign\", \"int\", \"Integer\", \"\",1\n"
    "  P: \"UnitScaleFactor\", \"double\", \"Number\", \"\",2.54\n"
    "  P: \"TimeMode\", \"enum\", \"\", \"\",9\n"
    "  P: \"TimeSpanStop\", \"KTime\", \"Time\", \"\",46186158000\n"
    " }\n}\n"
    "Definitions: {\n"
    " ObjectType: \"Model\" {\n  PropertyTemplate: \"FbxNode\" {\n   Properties70: {\n"
    "    P: \"RotationOrder\", \"enum\", \"\", \"\",4\n"
    "    P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",1,1,1\n"
    "   }\n  }\n }\n"
    " ObjectType: \"NodeAttribute\" {\n  PropertyTemplate: \"FbxCamera\" {\n"
    "   Properties70: {\n    P: \"Size\", \"double\", \"Number\", \"\",7\n   }\n  }\n"
    "  PropertyTemplate: \"FbxSkeleton\" {\n"
    "   Properties70: {\n    P: \"Size\", \"double\", \"Number\", \"\",33\n   }\n  }\n }\n"
    "}\n"
    "Objects: {\n"
    " Model: 100, \"Model::hip\", \"LimbNode\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,2,0\n"
    "  }\n }\n"
    " Model: 200, \"Model::spine\", \"LimbNode\" {\n }\n"
    " NodeAttribute: 300, \"NodeAttribute::\", \"Skeleton\" {\n }\n"
    " NodeAttribute: 301, \"NodeAttribute::\", \"Camera\" {\n }\n"
    " NodeAttribute: 302, \"NodeAttribute::\", \"Light\" {\n }\n"
    " Weird: \"no id\"\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",100,0\n"
    " C: \"OO\",200,100\n"
    " C: \"OO\",300,100\n"
    " C: \"OP\",301,200, \"Lcl Translation\"\n"
    " C: \"PP\",302, \"Intensity\",200, \"Visibility\"\n"
    " C: \"OO\",999,100\n"
    " C: \"XX\",200,100\n"
    "}\n";

static const char kSix[] = "; FBX 6.1.0 project file\n"
                           "Objects: {\n"
                           " Model: \"Model::hip\", \"Limb\" {\n  Properties60: {\n"
                           "   Property: \"Lcl Translation\", \"Lcl Translation\", \"A\",1,2,3\n"
                           "  }\n }\n"
                           " Model: \"Model::arm\", \"Limb\" {\n }\n"
                           "}\n"
                           "Connections: {\n"
                           " Connect: \"OO\", \"Model::hip\", \"Model::Scene\"\n"
                           " Connect: \"OO\", \"Model::arm\", \"Model::hip\"\n"
                           "}\n";

static char s_text[8192];

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static maniFbx* Open(maniBuilder* builder, const char* text)
{
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, text, strlen(text), &fbx);
    CHECK(r.result == mani_success, "opens");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu near: %.30s\n", r.reason, (unsigned long long)r.offset,
               text + r.offset);
    }
    return fbx;
}

static bool NameIs(const maniFbx* fbx, uint32_t object, const char* name)
{
    uint8_t out[64];
    size_t n = maniFbxObjectName(fbx, object, out, sizeof out);
    return n == strlen(name) && memcmp(out, name, n) == 0;
}

static double Value(const maniFbx* fbx, uint32_t object, const char* property, uint32_t k)
{
    uint32_t entry = maniFbxFindProperty(fbx, object, property, strlen(property));
    double d = -999;
    if (entry == MANI_FBX_NONE || !maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, k), &d))
    {
        return -999;
    }
    return d;
}

static void TestSeven(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kSeven);
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(fbx->objectCount == 5 && fbx->skippedObjects == 1, "five objects, one without an id");
    uint32_t hip = maniFbxFindObject(fbx, 100);
    uint32_t spine = maniFbxFindObject(fbx, 200);
    uint32_t skeleton = maniFbxFindObject(fbx, 300);
    uint32_t camera = maniFbxFindObject(fbx, 301);
    uint32_t light = maniFbxFindObject(fbx, 302);
    CHECK(hip == 0 && spine == 1 && skeleton == 2 && camera == 3 && light == 4, "found by id");
    CHECK(maniFbxFindObject(fbx, 999) == MANI_FBX_NONE, "an id no object has");
    CHECK(NameIs(fbx, hip, "hip") && NameIs(fbx, skeleton, ""), "names without their class");
    CHECK(maniFbxObjectKindIs(fbx, hip, "Model", 5) &&
              maniFbxObjectSubclassIs(fbx, hip, "LimbNode", 8),
          "kind and subclass");
    CHECK(Value(fbx, hip, "Lcl Translation", 1) == 2.0, "a name given twice: the later");
    CHECK(Value(fbx, hip, "RotationOrder", 0) == 4.0, "a template's default");
    CHECK(Value(fbx, spine, "Lcl Scaling", 2) == 1.0, "the template for every model");
    CHECK(Value(fbx, skeleton, "Size", 0) == 33.0 && Value(fbx, camera, "Size", 0) == 7.0,
          "the template whose class is the subclass");
    CHECK(maniFbxFindProperty(fbx, light, "Size", 4) == MANI_FBX_NONE, "no template for a light");
    CHECK(maniFbxFindProperty(fbx, hip, "Nothing", 7) == MANI_FBX_NONE, "no such property");
    CHECK(maniFbxPropertyValue(fbx, MANI_FBX_NONE, 0) == MANI_FBX_NONE, "no entry, no value");
    // Connections.
    CHECK(fbx->connectionCount == 5 && fbx->skippedConnections == 2,
          "five connections; an unknown id and an unknown type skipped");
    const uint32_t* list = NULL;
    uint32_t n = maniFbxChildConnections(fbx, fbx->objectCount, &list);
    CHECK(n == 1 && fbx->connections[list[0]].child == hip, "the root's child");
    n = maniFbxChildConnections(fbx, hip, &list);
    CHECK(n == 2 && fbx->connections[list[0]].child == spine &&
              fbx->connections[list[1]].child == skeleton,
          "a parent's children in file order");
    n = maniFbxChildConnections(fbx, spine, &list);
    CHECK(n == 2 && fbx->connections[list[0]].child == camera &&
              maniFbxStringEquals(fbx, fbx->connections[list[0]].parentProperty, "Lcl Translation",
                                  15),
          "a connection to a property");
    const maniFbxConnection* pp = &fbx->connections[list[1]];
    CHECK(pp->child == light && maniFbxStringEquals(fbx, pp->childProperty, "Intensity", 9) &&
              maniFbxStringEquals(fbx, pp->parentProperty, "Visibility", 10),
          "property to property");
    n = maniFbxParentConnections(fbx, spine, &list);
    CHECK(n == 1 && fbx->connections[list[0]].parent == hip, "a child's parent");
    CHECK(maniFbxParentConnections(fbx, fbx->objectCount, &list) == 0, "the root has none");
    // Settings.
    const maniFbxGlobalSettings* s = &fbx->settings;
    CHECK(s->axes[0] == 0 && s->signs[0] == 1 && s->axes[1] == 2 && s->signs[1] == 1 &&
              s->axes[2] == 1 && s->signs[2] == -1,
          "X right, Z up, -Y front");
    CHECK(s->unitScale == 2.54 && s->originalUnitScale == 2.54, "inches");
    CHECK(s->timeMode == 9 && s->rateNumerator == 30000 && s->rateDenominator == 1001,
          "NTSC, exactly");
    CHECK(s->spanStart == 0 && s->spanStop == 46186158000LL, "the span");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestSix(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kSix);
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniFbxVersion(fbx) == 6100 && fbx->objectCount == 2, "two 6.x objects");
    CHECK(NameIs(fbx, 0, "hip") && maniFbxObjectSubclassIs(fbx, 0, "Limb", 4),
          "named and subclassed");
    CHECK(Value(fbx, 0, "Lcl Translation", 1) == 2.0, "Properties60, no label");
    const uint32_t* list = NULL;
    uint32_t n = maniFbxChildConnections(fbx, fbx->objectCount, &list);
    CHECK(n == 1 && fbx->connections[list[0]].child == 0, "Model::Scene is the root");
    n = maniFbxChildConnections(fbx, 0, &list);
    CHECK(n == 1 && fbx->connections[list[0]].child == 1, "connected by name");
    const maniFbxGlobalSettings* s = &fbx->settings;
    CHECK(s->axes[1] == 1 && s->signs[1] == 1 && s->unitScale == 1.0 && s->rateNumerator == 30 &&
              s->rateDenominator == 1,
          "FBX's defaults with no GlobalSettings");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// kSeven with one setting line replaced.
static void Setting(const char* from, const char* to)
{
    const char* at = strstr(kSeven, from);
    size_t head = (size_t)(at - kSeven);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)head, kSeven, to, at + strlen(from));
}

static void ExpectRefusal(const char* at, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, s_text, strlen(s_text), &fbx);
    const char* found = at != NULL ? strstr(s_text, at) : NULL;
    bool right = r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
                 (found == NULL || r.offset == (uint64_t)(found - s_text));
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu near: %.20s\n", (int)r.result, r.reason,
               (unsigned long long)r.offset, s_text + (r.offset < strlen(s_text) ? r.offset : 0));
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// An id given twice is kept by its first object, as ufbx keeps it: the
// later one stays an object that no id finds, counted.
// Names made valid UTF-8 as ufbx makes them (mani-0012): valid
// sequences of two to four bytes kept; a byte of no sequence (C1), an
// overlong form, a surrogate, a code past U+10FFFF and a cut sequence
// each a U+FFFD per byte; whole characters up to a capacity; counted
// with no output; compared, hashed and matched made valid.
static void TestInvalidUtf8(void)
{
    static const char kText[] =
        "; FBX 7.4.0 project file\n"
        "Objects: {\n"
        " Model: 1, \"Model::A\xC1"
        "B\", \"Null\" {\n }\n"
        " Model: 2, \"Model::\xC3\xA9\xE2\x82\xAC\xF0\x9F\x98\x80\", \"Null\" {\n }\n"
        " Model: 3, \"Model::\xC0\x80\", \"Null\" {\n }\n"
        " Model: 4, \"Model::\xED\xA0\x80\", \"Null\" {\n }\n"
        " Model: 5, \"Model::\xF4\x90\x80\x80\", \"Null\" {\n }\n"
        " Model: 6, \"Model::x\xE2\x82\", \"Null\" {\n }\n"
        " Model: 7, \"Model::A\xC1\", \"Null\" {\n }\n"
        " Model: 8, \"Model::A\xC2\", \"Null\" {\n }\n"
        " Model: 9, \"Model::a::b\", \"Null\" {\n }\n"
        " Model: 10, \"Model::a\0b\", \"Null\" {\n }\n"
        " Model: 11, \"Model::x\xC3"
        "A\", \"Null\" {\n }\n"
        " Model: 12, \"Model::\xC3\xA9\xC3\", \"Null\" {\n }\n"
        " Model: 13, \"Model::\xE0\x80\x80\", \"Null\" {\n }\n"
        " Model: 14, \"Model::\xF0\x8F\xBF\xBF\", \"Null\" {\n }\n"
        " Model: 15, \"Model::a::c\", \"Null\" {\n }\n"
        "}\n";
    static const char kR[] = "\xEF\xBF\xBD";
    char twice[16];
    char thrice[16];
    char four[16];
    snprintf(twice, sizeof twice, "%s%s", kR, kR);
    snprintf(thrice, sizeof thrice, "%s%s%s", kR, kR, kR);
    snprintf(four, sizeof four, "%s%s%s%s", kR, kR, kR, kR);
    char cut[16];
    snprintf(cut, sizeof cut, "x%s%s", kR, kR);
    maniBuilder* builder = NewBuilder();
    // The text holds a NUL: opened by its size.
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kText, sizeof kText - 1, &fbx).result == mani_success, "opens");
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    uint32_t o[16];
    for (int64_t id = 1; id <= 15; ++id)
    {
        o[id] = maniFbxFindObject(fbx, id);
    }
    CHECK(NameIs(fbx, o[1],
                 "A\xEF\xBF\xBD"
                 "B"),
          "a byte of no sequence");
    CHECK(NameIs(fbx, o[2], "\xC3\xA9\xE2\x82\xAC\xF0\x9F\x98\x80"), "valid sequences kept");
    CHECK(NameIs(fbx, o[3], twice), "an overlong form");
    CHECK(NameIs(fbx, o[4], thrice), "a surrogate");
    CHECK(NameIs(fbx, o[5], four), "past U+10FFFF");
    CHECK(NameIs(fbx, o[6], cut), "a cut sequence");
    CHECK(NameIs(fbx, o[9], "a::b"), "the class cut at the first ::");
    CHECK(NameIs(fbx, o[10],
                 "a\xEF\xBF\xBD"
                 "b"),
          "a NUL");
    CHECK(NameIs(fbx, o[11],
                 "x\xEF\xBF\xBD"
                 "A"),
          "a lead byte with no continuation");
    CHECK(NameIs(fbx, o[12], "\xC3\xA9\xEF\xBF\xBD"), "a lead byte at the end");
    CHECK(NameIs(fbx, o[13], thrice), "an overlong form of three bytes");
    CHECK(NameIs(fbx, o[14], four), "an overlong form of four bytes");
    uint8_t out[8];
    CHECK(maniFbxObjectName(fbx, o[5], NULL, 0) == 12 && maniFbxObjectName(fbx, o[2], out, 4) == 2,
          "counted with no output; whole characters up to a capacity");
    uint32_t a1 = maniFbxNodeProperty(fbx, fbx->objects[o[7]].node, 1);
    uint32_t a2 = maniFbxNodeProperty(fbx, fbx->objects[o[8]].node, 1);
    uint32_t longer = maniFbxNodeProperty(fbx, fbx->objects[o[1]].node, 1);
    uint32_t b = maniFbxNodeProperty(fbx, fbx->objects[o[9]].node, 1);
    uint32_t c = maniFbxNodeProperty(fbx, fbx->objects[o[15]].node, 1);
    CHECK(!maniFbxSameString(fbx, a1, longer) && !maniFbxSameString(fbx, b, c),
          "one name a prefix of another, or of the same length, not the same");
    static const char kMade[] = "Model::A\xEF\xBF\xBD";
    CHECK(maniFbxStringEquals(fbx, a1, kMade, strlen(kMade)) &&
              !maniFbxStringEquals(fbx, a1, "Model::A\xC1", 9) && maniFbxSameString(fbx, a1, a2) &&
              maniFbxStringHash(fbx, a1) == maniFbxNameHash(kMade, strlen(kMade)),
          "compared, matched and hashed made valid");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestDuplicateId(void)
{
    Setting("Model: 200,", "Model: 100,");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    bool open = maniOpenFbx(builder, s_text, strlen(s_text), &fbx).result == mani_success;
    uint32_t found = open ? maniFbxFindObject(fbx, 100) : MANI_FBX_NONE;
    uint32_t later = MANI_FBX_NONE;
    for (uint32_t o = found != MANI_FBX_NONE ? found + 1 : UINT32_MAX;
         open && o < fbx->objectCount && later == MANI_FBX_NONE; ++o)
    {
        later = fbx->objects[o].id == 100 ? o : MANI_FBX_NONE;
    }
    CHECK(open && found != MANI_FBX_NONE && later != MANI_FBX_NONE &&
              maniFbxObjectKindIs(fbx, found, "Model", 5) && fbx->duplicateObjects == 1,
          "an id twice: the first object found, the later kept and counted");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    Setting("\"\",2\n", "\"\",3\n");
    ExpectRefusal("3\n  P: \"UpAxisSign", "an axis past z");
    Setting("\"\",-1\n", "\"\",0\n");
    ExpectRefusal("0\n  P: \"CoordAxis\"", "a sign of 0");
    Setting("\"FrontAxis\", \"int\", \"Integer\", \"\",1",
            "\"FrontAxis\", \"int\", \"Integer\", \"\",2");
    ExpectRefusal("2\n  P: \"FrontAxisSign", "up and front the same axis");
    Setting("\"\",2.54\n", "\"\",0\n");
    ExpectRefusal("0\n  P: \"TimeMode", "a unit of nothing");
    Setting("\"\",9\n", "\"\",19\n");
    ExpectRefusal("19\n", "a time mode FBX lacks");
    Setting("\"\",9\n", "\"\",14\n");
    ExpectRefusal(NULL, "a custom rate not given");
    Setting("\"\",9\n", "\"\",14\n  P: \"CustomFrameRate\", \"double\", \"Number\", \"\",-12.5\n");
    ExpectRefusal("-12.5", "a custom rate below zero");
    Setting("\"\",9\n", "\"\",14\n  P: \"CustomFrameRate\", \"double\", \"Number\", \"\",12.5\n");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, s_text);
    CHECK(fbx != NULL && fbx->settings.rateDenominator == 0 && fbx->settings.customRate == 12.5,
          "a custom rate");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestSeven();
    TestSix();
    TestDuplicateId();
    TestInvalidUtf8();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
