// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX's local transform: sines exact at right angles, Euler
// rotations in each order against ufbx's closed forms, the pivot and
// offset chain against the product of its factors, the rotation space
// switches, matrix inverses, and each refusal.

#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_transform.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static uint64_t s_seed = 0x9E3779B97F4A7C15u;

static double Random(double low, double high)
{
    s_seed = s_seed * 6364136223846793005u + 1442695040888963407u;
    return low + (high - low) * (double)(s_seed >> 11) / 9007199254740992.0;
}

static void TestSinCos(void)
{
    static const double kRight[][3] = {{0, 0, 1},    {90, 1, 0},  {180, 0, -1}, {270, -1, 0},
                                       {-90, -1, 0}, {450, 1, 0}, {720, 0, 1},  {-540, 0, -1}};
    bool exact = true;
    for (size_t i = 0; i < sizeof kRight / sizeof kRight[0]; ++i)
    {
        double s = 9;
        double c = 9;
        maniFbxSinCosDegrees(kRight[i][0], &s, &c);
        exact = exact && s == kRight[i][1] && c == kRight[i][2];
    }
    CHECK(exact, "right angles exactly");
    double tinySin = 9;
    double tinyCos = 9;
    maniFbxSinCosDegrees(-1e-20, &tinySin, &tinyCos);
    CHECK(fabs(tinySin) < 1e-30 && tinyCos == 1.0, "a tiny negative angle rounding to a full turn");
    double worst = 0;
    for (int i = 0; i < 10000; ++i)
    {
        double d = Random(-1000, 1000);
        double s = 0;
        double c = 0;
        maniFbxSinCosDegrees(d, &s, &c);
        double r = d * 0.017453292519943295;
        worst = fmax(worst, fmax(fabs(s - sin(r)), fabs(c - cos(r))));
    }
    // The library's quarter series keeps about 2e-14 (binary32's needs).
    CHECK(worst < 1e-13, "within 1e-13 of libm elsewhere");
}

// ufbx_euler_to_quat's closed forms (ufbx 5955c5c).
static void Ufbx(const double v[3], uint32_t order, double q[4])
{
    double h = 0.5 * 0.017453292519943295;
    double cx = cos(v[0] * h), sx = sin(v[0] * h);
    double cy = cos(v[1] * h), sy = sin(v[1] * h);
    double cz = cos(v[2] * h), sz = sin(v[2] * h);
    double forms[6][4] = {
        {-cx * sy * sz + cy * cz * sx, cx * cz * sy + cy * sx * sz, cx * cy * sz - cz * sx * sy,
         cx * cy * cz + sx * sy * sz},
        {cx * sy * sz + cy * cz * sx, cx * cz * sy + cy * sx * sz, cx * cy * sz - cz * sx * sy,
         cx * cy * cz - sx * sy * sz},
        {-cx * sy * sz + cy * cz * sx, cx * cz * sy - cy * sx * sz, cx * cy * sz + cz * sx * sy,
         cx * cy * cz + sx * sy * sz},
        {-cx * sy * sz + cy * cz * sx, cx * cz * sy + cy * sx * sz, cx * cy * sz + cz * sx * sy,
         cx * cy * cz - sx * sy * sz},
        {cx * sy * sz + cy * cz * sx, cx * cz * sy - cy * sx * sz, cx * cy * sz - cz * sx * sy,
         cx * cy * cz + sx * sy * sz},
        {cx * sy * sz + cy * cz * sx, cx * cz * sy - cy * sx * sz, cx * cy * sz + cz * sx * sy,
         cx * cy * cz - sx * sy * sz},
    };
    memcpy(q, forms[order], sizeof forms[order]);
}

static void TestEuler(void)
{
    for (uint32_t order = 0; order < 6; ++order)
    {
        double worst = 0;
        for (int i = 0; i < 1000; ++i)
        {
            double v[3] = {Random(-400, 400), Random(-400, 400), Random(-400, 400)};
            double q[4];
            double u[4];
            maniFbxEulerQuat(v, order, q);
            Ufbx(v, order, u);
            for (int k = 0; k < 4; ++k)
            {
                worst = fmax(worst, fabs(q[k] - u[k]));
            }
        }
        CHECK(worst < 1e-13, "each order as ufbx's closed form");
    }
}

static bool Near(const maniFbxMatrix* a, const maniFbxMatrix* b, double tolerance)
{
    for (int i = 0; i < 12; ++i)
    {
        if (!(fabs(a->m[i] - b->m[i]) <= tolerance))
        {
            return false;
        }
    }
    return true;
}

static maniFbxMatrix Translate(const double v[3], double sign)
{
    maniFbxTrs t = {{sign * v[0], sign * v[1], sign * v[2]}, {0, 0, 0, 1}, {1, 1, 1}};
    return maniFbxTrsMatrix(&t);
}

static maniFbxMatrix Rotate(const double degrees[3], uint32_t order, bool inverse)
{
    maniFbxTrs t = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    maniFbxEulerQuat(degrees, order, t.rotation);
    for (int k = 0; inverse && k < 3; ++k)
    {
        t.rotation[k] = -t.rotation[k];
    }
    return maniFbxTrsMatrix(&t);
}

static void TestMatrices(void)
{
    double worst = 0;
    bool inverses = true;
    for (int i = 0; i < 200; ++i)
    {
        double v[3] = {Random(-180, 180), Random(-180, 180), Random(-180, 180)};
        maniFbxTrs t = {{Random(-5, 5), Random(-5, 5), Random(-5, 5)},
                        {0, 0, 0, 1},
                        {Random(0.1, 3), Random(-3, -0.1), Random(0.1, 3)}};
        maniFbxEulerQuat(v, (uint32_t)i % 6, t.rotation);
        maniFbxMatrix m = maniFbxTrsMatrix(&t);
        maniFbxMatrix inv;
        inverses = inverses && maniFbxMatrixInverse(&m, &inv);
        maniFbxMatrix product = maniFbxMatrixMul(&m, &inv);
        maniFbxMatrix identity = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
        for (int k = 0; k < 12; ++k)
        {
            worst = fmax(worst, fabs(product.m[k] - identity.m[k]));
        }
        double p[3] = {Random(-2, 2), Random(-2, 2), Random(-2, 2)};
        double rotated[3];
        maniFbxQuatRotate(t.rotation, p, rotated);
        maniFbxTrs pure = {
            {0, 0, 0}, {t.rotation[0], t.rotation[1], t.rotation[2], t.rotation[3]}, {1, 1, 1}};
        maniFbxMatrix r = maniFbxTrsMatrix(&pure);
        for (int k = 0; k < 3; ++k)
        {
            double expected = r.m[k] * p[0] + r.m[3 + k] * p[1] + r.m[6 + k] * p[2];
            worst = fmax(worst, fabs(rotated[k] - expected));
        }
    }
    CHECK(inverses && worst < 1e-12, "inverses, products and rotations agree");
    maniFbxMatrix singular = {{1, 0, 0, 2, 0, 0, 0, 0, 1, 0, 0, 0}};
    maniFbxMatrix out;
    CHECK(!maniFbxMatrixInverse(&singular, &out), "a singular matrix has none");
    maniFbxMatrix huge = {{1e-300, 0, 0, 0, 1e-300, 0, 0, 0, 1e-300, 0, 0, 0}};
    CHECK(!maniFbxMatrixInverse(&huge, &out), "nor one whose determinant underflows");
    maniFbxMatrix subnormal = {{1e-104, 0, 0, 0, 1e-103, 0, 0, 0, 1e-103, 0, 0, 0}};
    CHECK(!maniFbxMatrixInverse(&subnormal, &out), "nor one whose determinant is subnormal");
    maniFbxMatrix far = {{1e-10, 0, 0, 0, 1e-10, 0, 0, 0, 1e-10, 1e305, 0, 0}};
    CHECK(!maniFbxMatrixInverse(&far, &out), "nor one whose translation overflows");
    maniFbxTrs known = {{1, 2, 3}, {0, 0, 0.6, 0.8}, {2, 3, 4}};
    maniFbxMatrix k = maniFbxTrsMatrix(&known);
    // A rotation of 2 atan(0.75) about z: cos 0.28, sin 0.96.
    maniFbxMatrix expected = {{0.56, 1.92, 0, -2.88, 0.84, 0, 0, 0, 4, 1, 2, 3}};
    CHECK(Near(&k, &expected, 1e-15), "a known TRS as its matrix");
}

static char s_text[4096];
static const char* s_template = "    P: \"ScalingPivot\", \"Vector3D\", \"Vector\", \"\",0,0,0\n";

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// A document with one model holding the properties given, its template
// s_template's.
static void Model(const char* properties)
{
    snprintf(
        s_text, sizeof s_text,
        "; FBX 7.4.0 project file\n"
        "Definitions: {\n ObjectType: \"Model\" {\n  PropertyTemplate: \"FbxNode\" {\n"
        "   Properties70: {\n%s   }\n  }\n }\n}\n"
        "Objects: {\n Model: 1, \"Model::m\", \"LimbNode\" {\n  Properties70: {\n%s  }\n }\n}\n",
        s_template, properties);
}

static maniImportResult Local(maniFbxLocal* out)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, s_text, strlen(s_text), &fbx);
    if (r.result == mani_success)
    {
        r = maniFbxModelLocal(fbx, 0, NULL, out);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return r;
}

static void TestChain(void)
{
    double T[3] = {1, 2, 3};
    double Roff[3] = {0.5, -0.25, 2};
    double Rp[3] = {-1, 0.75, 0.5};
    double Rpre[3] = {10, -20, 30};
    double R[3] = {45, 15, -60};
    double Rpost[3] = {-5, 25, 12};
    double Soff[3] = {0.1, 0.2, -0.3};
    double Sp[3] = {0.4, -0.6, 1.5};
    double S[3] = {2, 0.5, -1.25};
    char properties[2048];
    snprintf(properties, sizeof properties,
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,2,3\n"
             "   P: \"RotationOffset\", \"Vector3D\", \"Vector\", \"\",0.5,-0.25,2\n"
             "   P: \"RotationPivot\", \"Vector3D\", \"Vector\", \"\",-1,0.75,0.5\n"
             "   P: \"PreRotation\", \"Vector3D\", \"Vector\", \"\",10,-20,30\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",45,15,-60\n"
             "   P: \"PostRotation\", \"Vector3D\", \"Vector\", \"\",-5,25,12\n"
             "   P: \"ScalingOffset\", \"Vector3D\", \"Vector\", \"\",0.1,0.2,-0.3\n"
             "   P: \"ScalingPivot\", \"Vector3D\", \"Vector\", \"\",0.4,-0.6,1.5\n"
             "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",2,0.5,-1.25\n"
             "   P: \"RotationOrder\", \"enum\", \"\", \"\",%s\n"
             "   P: \"RotationActive\", \"bool\", \"\", \"\",%s\n"
             "   P: \"InheritType\", \"enum\", \"\", \"\",2\n",
             "4", "1");
    Model(properties);
    maniFbxLocal local;
    CHECK(Local(&local).result == mani_success && local.inherit == maniFbxInheritRrs,
          "a model's local");
    // T Roff Rp Rpre R Rpost^-1 Rp^-1 Soff Sp S Sp^-1, factor by factor.
    maniFbxMatrix factors[11] = {Translate(T, 1),     Translate(Roff, 1),
                                 Translate(Rp, 1),    Rotate(Rpre, 0, false),
                                 Rotate(R, 4, false), Rotate(Rpost, 0, true),
                                 Translate(Rp, -1),   Translate(Soff, 1),
                                 Translate(Sp, 1),    {{0}},
                                 Translate(Sp, -1)};
    maniFbxTrs scale = {{0, 0, 0}, {0, 0, 0, 1}, {S[0], S[1], S[2]}};
    factors[9] = maniFbxTrsMatrix(&scale);
    maniFbxMatrix product = factors[0];
    for (int k = 1; k < 11; ++k)
    {
        product = maniFbxMatrixMul(&product, &factors[k]);
    }
    maniFbxMatrix got = maniFbxTrsMatrix(&local.trs);
    CHECK(Near(&got, &product, 1e-12), "the chain is one TRS");
    // RotationActive 0: R alone, in XYZ.
    snprintf(properties, sizeof properties,
             "   P: \"PreRotation\", \"Vector3D\", \"Vector\", \"\",10,-20,30\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",45,15,-60\n"
             "   P: \"RotationOrder\", \"enum\", \"\", \"\",4\n"
             "   P: \"RotationActive\", \"bool\", \"\", \"\",0\n");
    Model(properties);
    maniFbxMatrix alone = Rotate(R, 0, false);
    CHECK(Local(&local).result == mani_success && local.inherit == maniFbxInheritRSrs &&
              Near((got = maniFbxTrsMatrix(&local.trs), &got), &alone, 1e-14),
          "outside the rotation space: R alone, XYZ");
    snprintf(properties, sizeof properties,
             "   P: \"PreRotation\", \"Vector3D\", \"Vector\", \"\",10,-20,30\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",45,15,-60\n"
             "   P: \"RotationOrder\", \"enum\", \"\", \"\",4\n"
             "   P: \"RotationSpaceForLimitOnly\", \"bool\", \"\", \"\",1\n");
    Model(properties);
    CHECK(Local(&local).result == mani_success &&
              Near((got = maniFbxTrsMatrix(&local.trs), &got), &alone, 1e-14),
          "the rotation space for limits only");
    Model("   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",45,15,-60\n"
          "   P: \"RotationOrder\", \"enum\", \"\", \"\",6\n");
    CHECK(Local(&local).result == mani_success &&
              Near((got = maniFbxTrsMatrix(&local.trs), &got), &alone, 1e-14),
          "spheric is XYZ");
    Model("");
    maniFbxMatrix identity = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    CHECK(Local(&local).result == mani_success &&
              Near((got = maniFbxTrsMatrix(&local.trs), &got), &identity, 0),
          "no properties: identity");
}

static bool Translation(double x, double y, double z)
{
    maniFbxLocal local;
    return Local(&local).result == mani_success && local.trs.translation[0] == x &&
           local.trs.translation[1] == y && local.trs.translation[2] == z;
}

// Which entry gives a property: the model's own, its later one when
// given twice, else its template's; a name only alike is another.
static void TestLookup(void)
{
    static const char kOwn[] =
        "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,2,3\n";
    s_template = "    P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",4,5,6\n";
    Model("");
    CHECK(Translation(4, 5, 6), "the template's when the model lacks it");
    Model(kOwn);
    CHECK(Translation(1, 2, 3), "the model's own over its template's");
    Model("   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,2,3\n"
          "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",7,8,9\n");
    CHECK(Translation(7, 8, 9), "a name given twice: the later");
    Model("   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,2,3\n"
          "   P: \"Lcl Trans\", \"Lcl Translation\", \"\", \"A\",7,8,9\n"
          "   P: \"Lcl Translations\", \"Lcl Translation\", \"\", \"A\",7,8,9\n");
    CHECK(Translation(1, 2, 3), "names only alike: others");
    s_template = "    P: \"ScalingPivot\", \"Vector3D\", \"Vector\", \"\",0,0,0\n";
}

// One cache through every object twice, of two templates (a property
// in one only) and with a property of its own: each object's entries as
// found without one.
static void TestTemplateCache(void)
{
    snprintf(s_text, sizeof s_text,
             "; FBX 7.4.0 project file\nDefinitions: {\n"
             " ObjectType: \"Model\" {\n  PropertyTemplate: \"FbxNode\" {\n   Properties70: {\n"
             "    P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",4,5,6\n"
             "    P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",2,2,2\n   }\n  }\n }\n"
             " ObjectType: \"Material\" {\n  PropertyTemplate: \"FbxSurfacePhong\" {\n"
             "   Properties70: {\n"
             "    P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",7,8,9\n   }\n  }\n }\n"
             "}\nObjects: {\n Model: 1, \"Model::m\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,45\n  }\n }\n"
             " Material: 2, \"Material::x\", \"\" {\n }\n}\n");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_text, strlen(s_text), &fbx).result == mani_success, "opens");
    static const char* const kNames[3] = {"Lcl Translation", "Lcl Rotation", "Lcl Scaling"};
    maniFbxTemplateCache cache;
    maniFbxEmptyTemplateCache(&cache);
    bool same = fbx != NULL && fbx->objectCount >= 2;
    uint32_t templates = 0;
    uint32_t own = 0;
    for (uint32_t i = 0; same && i < 2 * fbx->objectCount; ++i)
    {
        uint32_t o = i % fbx->objectCount;
        uint32_t entries[3];
        maniFbxFindProperties(fbx, o, kNames, 3, &cache, entries);
        for (uint32_t k = 0; k < 3; ++k)
        {
            same = same && entries[k] == maniFbxFindProperty(fbx, o, kNames[k], strlen(kNames[k]));
        }
        templates += entries[0] != MANI_FBX_NONE ? 1 : 0;
        own += entries[1] != MANI_FBX_NONE ? 1 : 0;
    }
    CHECK(same && templates == 4 && own == 2, "a template cache: the same entries");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(const char* properties, const char* at, const char* what)
{
    Model(properties);
    maniFbxLocal local;
    maniImportResult r = Local(&local);
    const char* found = strstr(s_text, at);
    bool right = r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
                 found != NULL && r.offset == (uint64_t)(found - s_text) && r.item == 0;
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu near: %.20s\n", (int)r.result, r.reason,
               (unsigned long long)r.offset, s_text + (r.offset < strlen(s_text) ? r.offset : 0));
    }
}

static void TestRefusals(void)
{
    ExpectRefusal("   P: \"RotationOrder\", \"enum\", \"\", \"\",7\n", "7\n", "an order past 6");
    ExpectRefusal("   P: \"InheritType\", \"enum\", \"\", \"\",3\n", "3\n",
                  "an inherit type past 2");
    ExpectRefusal("   P: \"RotationActive\", \"bool\", \"\", \"\",-1\n", "-1\n", "a flag below 0");
    ExpectRefusal("   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",1,1.#INF,0\n", "1.#INF",
                  "a value not finite");
    ExpectRefusal("   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",1,1e999,0\n", "1e999",
                  "a value out of range");
    ExpectRefusal("   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,2\n",
                  "Lcl Translation\", \"Lcl", "two values of three");
    ExpectRefusal("   P: \"ScalingPivot\", \"Vector3D\", \"Vector\", \"\",0,\"x\",0\n", "x\",0",
                  "a string for a number");
}

// The same rotation, the quaternions equal up to sign.
static bool SameRotation(const double a[4], const double b[4], double tolerance)
{
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    return fabs(fabs(dot) - 1.0) <= tolerance;
}

// Each order's axes, the first applied first (the middle one's angle
// within a quarter turn).
static const int kAxes[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 2, 0}, {1, 0, 2}, {2, 0, 1}, {2, 1, 0}};

static void TestQuatEuler(void)
{
    double worst = 0.0;
    bool gimbal = true;
    for (uint32_t order = 0; order < 6; ++order)
    {
        for (int i = -170; i <= 170; i += 17)
        {
            for (int j = -85; j <= 85; j += 17)
            {
                for (int k = -170; k <= 170; k += 23)
                {
                    double angles[3];
                    angles[kAxes[order][0]] = (double)i;
                    angles[kAxes[order][1]] = (double)j;
                    angles[kAxes[order][2]] = (double)k;
                    double q[4];
                    double out[3];
                    maniFbxEulerQuat(angles, order, q);
                    maniFbxQuatEuler(q, order, out);
                    for (int c = 0; c < 3; ++c)
                    {
                        double e = fabs(out[c] - angles[c]);
                        worst = e > worst ? e : worst;
                    }
                }
            }
            // A quarter turn about the middle axis: another set of angles,
            // the same rotation.
            // ufbx's convention there: the first angle 0, the middle a
            // quarter turn of the same sign.
            for (int sign = -1; sign <= 1; sign += 2)
            {
                double locked[3] = {(double)i, (double)i, (double)i};
                locked[kAxes[order][1]] = 90.0 * sign;
                double q[4];
                double back[4];
                double out[3];
                maniFbxEulerQuat(locked, order, q);
                maniFbxQuatEuler(q, order, out);
                maniFbxEulerQuat(out, order, back);
                gimbal = gimbal && SameRotation(q, back, 1e-12) && out[kAxes[order][0]] == 0.0 &&
                         fabs(out[kAxes[order][1]] - 90.0 * sign) <= 1e-12;
            }
            // Within 1e-9 of the lock by the sine (89.99999 degrees would be
            // exactly locked already), treated as locked.
            double near[3] = {(double)i, (double)i, (double)i};
            near[kAxes[order][1]] = 89.999;
            // The last angle 0: the first is i, recoverable short of the
            // threshold, so 0 shows the threshold applied.
            near[kAxes[order][2]] = 0.0;
            double q[4];
            double out[3];
            maniFbxEulerQuat(near, order, q);
            maniFbxQuatEuler(q, order, out);
            gimbal = gimbal && out[kAxes[order][0]] == 0.0;
        }
    }
    CHECK(worst <= 1e-9, "Euler angles back from their quaternion, each order");
    CHECK(gimbal, "at gimbal lock, the same rotation");
}

int main(void)
{
    TestSinCos();
    TestEuler();
    TestQuatEuler();
    TestMatrices();
    TestChain();
    TestLookup();
    TestTemplateCache();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
