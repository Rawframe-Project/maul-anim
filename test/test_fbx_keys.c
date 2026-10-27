// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX 6.x keys: each mode, slope mode and weight mode at
// KeyVer 4003, 4004 and 4005, extrapolations given as letters, and what
// is refused, against keys and values ufbx gives for the same takes.

#include "fbx_curve.h"
#include "fbx_document.h"
#include "fbx_keys.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct Expected
{
    double time;
    double value;
    uint32_t interpolation;
    float leftDx;
    float leftDy;
    float rightDx;
    float rightDy;
} Expected;

typedef struct Case
{
    const char* name;
    const char* text;
    // A refusal's reason, maniImportReasonNone when read.
    maniImportReason refusal;
    bool curve;
    uint32_t count;
    const Expected* keys;
    uint32_t pre;
    int32_t preRepeat;
    uint32_t post;
    int32_t postRepeat;
    // Values at -1 s to 3 s in quarters, -7 where there are no keys.
    const double* values;
} Case;

static const char kLinear[] = "; FBX 6.1.0 project file\n"
                              "FBXHeaderExtension: {\n"
                              " FBXHeaderVersion: 1003\n"
                              " FBXVersion: 6100\n"
                              "}\n"
                              "Objects: {\n"
                              " Model: \"Model::A\", \"Null\" {\n"
                              "  Version: 232\n"
                              " }\n"
                              "}\n"
                              "Takes: {\n"
                              " Take: \"T\" {\n"
                              "  LocalTime: 0,92372316000\n"
                              "  Model: \"Model::A\" {\n"
                              "   Channel: \"Transform\" {\n"
                              "    Channel: \"T\" {\n"
                              "     Channel: \"X\" {\n"
                              "      Default: 0.5\n"
                              "      KeyVer: 4005\n"
                              "      KeyCount: 3\n"
                              "      Key: 0,0,L,46186158000,10,L,92372316000,4,L\n"
                              "     }\n"
                              "    }\n"
                              "   }\n"
                              "  }\n"
                              " }\n"
                              "}\n";

static const Expected kLinearKeys[3] = {
    {0.0, 0.0, 2, 0.0f, 0.0f, 0.333332986f, 3.33332992f},
    {1.0, 10.0, 2, 0.333332986f, 3.33332992f, 0.333332986f, -1.99999785f},
    {2.0, 4.0, 2, 0.333332986f, -1.99999785f, 0.0f, 0.0f},
};

static const double kLinearValues[21] = {0.0, 0.0, 0.0, 0.0, 0.0, 2.5, 5.0, 7.5, 10.0, 8.5, 7.0,
                                         5.5, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0,  4.0};

static const char kConstant[] = "; FBX 6.1.0 project file\n"
                                "FBXHeaderExtension: {\n"
                                " FBXHeaderVersion: 1003\n"
                                " FBXVersion: 6100\n"
                                "}\n"
                                "Objects: {\n"
                                " Model: \"Model::A\", \"Null\" {\n"
                                "  Version: 232\n"
                                " }\n"
                                "}\n"
                                "Takes: {\n"
                                " Take: \"T\" {\n"
                                "  LocalTime: 0,92372316000\n"
                                "  Model: \"Model::A\" {\n"
                                "   Channel: \"Transform\" {\n"
                                "    Channel: \"T\" {\n"
                                "     Channel: \"X\" {\n"
                                "      Default: 0.5\n"
                                "      KeyVer: 4005\n"
                                "      KeyCount: 3\n"
                                "      Key: 0,1,C,n,46186158000,2,C,p,92372316000,3,L\n"
                                "     }\n"
                                "    }\n"
                                "   }\n"
                                "  }\n"
                                " }\n"
                                "}\n";

static const Expected kConstantKeys[3] = {
    {0.0, 1.0, 1, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 2.0, 0, 0.333332986f, 0.0f, 0.333332986f, 0.0f},
    {2.0, 3.0, 2, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kConstantValues[21] = {1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                                           2.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

static const char kConstant4003[] = "; FBX 6.1.0 project file\n"
                                    "FBXHeaderExtension: {\n"
                                    " FBXHeaderVersion: 1003\n"
                                    " FBXVersion: 6100\n"
                                    "}\n"
                                    "Objects: {\n"
                                    " Model: \"Model::A\", \"Null\" {\n"
                                    "  Version: 232\n"
                                    " }\n"
                                    "}\n"
                                    "Takes: {\n"
                                    " Take: \"T\" {\n"
                                    "  LocalTime: 0,92372316000\n"
                                    "  Model: \"Model::A\" {\n"
                                    "   Channel: \"Transform\" {\n"
                                    "    Channel: \"T\" {\n"
                                    "     Channel: \"X\" {\n"
                                    "      Default: 0.5\n"
                                    "      KeyVer: 4003\n"
                                    "      KeyCount: 2\n"
                                    "      Key: 0,1,C,46186158000,2,L\n"
                                    "     }\n"
                                    "    }\n"
                                    "   }\n"
                                    "  }\n"
                                    " }\n"
                                    "}\n";

static const Expected kConstant4003Keys[2] = {
    {0.0, 1.0, 0, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 2.0, 2, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kConstant4003Values[21] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                               1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                                               2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

static const char kUser[] = "; FBX 6.1.0 project file\n"
                            "FBXHeaderExtension: {\n"
                            " FBXHeaderVersion: 1003\n"
                            " FBXVersion: 6100\n"
                            "}\n"
                            "Objects: {\n"
                            " Model: \"Model::A\", \"Null\" {\n"
                            "  Version: 232\n"
                            " }\n"
                            "}\n"
                            "Takes: {\n"
                            " Take: \"T\" {\n"
                            "  LocalTime: 0,92372316000\n"
                            "  Model: \"Model::A\" {\n"
                            "   Channel: \"Transform\" {\n"
                            "    Channel: \"T\" {\n"
                            "     Channel: \"X\" {\n"
                            "      Default: 0.5\n"
                            "      KeyVer: 4005\n"
                            "      KeyCount: 4\n"
                            "      Key: "
                            "0,0,U,s,2,-1,n,46186158000,5,U,s,0,0,a,0.5,0.25,92372316000,1,U,s,1,1,"
                            "l,0.4,138558474000,2,U,s,0,0,r,0.6\n"
                            "     }\n"
                            "    }\n"
                            "   }\n"
                            "  }\n"
                            " }\n"
                            "}\n";

static const Expected kUserKeys[4] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.666665971f},
    {1.0, 5.0, 3, 0.333332986f, -0.333332986f, 0.5f, 0.0f},
    {2.0, 1.0, 3, 0.25f, 0.0f, 0.333332986f, 0.333332986f},
    {3.0, 2.0, 3, 0.400000006f, 0.400000006f, 0.0f, 0.0f},
};

static const double kUserValues[21] = {0.0,
                                       0.0,
                                       0.0,
                                       0.0,
                                       0.0,
                                       1.1093752750310975,
                                       2.8749996088445187,
                                       4.453124284918785,
                                       5.0,
                                       4.64978125901672,
                                       3.569336082897155,
                                       1.9780893438524927,
                                       1.0,
                                       1.25,
                                       1.5000000000000002,
                                       1.75,
                                       2.0,
                                       2.0,
                                       2.0,
                                       2.0,
                                       2.0};

static const char kAuto[] =
    "; FBX 6.1.0 project file\n"
    "FBXHeaderExtension: {\n"
    " FBXHeaderVersion: 1003\n"
    " FBXVersion: 6100\n"
    "}\n"
    "Objects: {\n"
    " Model: \"Model::A\", \"Null\" {\n"
    "  Version: 232\n"
    " }\n"
    "}\n"
    "Takes: {\n"
    " Take: \"T\" {\n"
    "  LocalTime: 0,92372316000\n"
    "  Model: \"Model::A\" {\n"
    "   Channel: \"Transform\" {\n"
    "    Channel: \"T\" {\n"
    "     Channel: \"X\" {\n"
    "      Default: 0.5\n"
    "      KeyVer: 4005\n"
    "      KeyCount: 4\n"
    "      Key: 0,0,U,a,n,46186158000,5,U,a,n,92372316000,2,U,a,n,138558474000,8,U,a,n\n"
    "     }\n"
    "    }\n"
    "   }\n"
    "  }\n"
    " }\n"
    "}\n";

static const Expected kAutoKeys[4] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 5.0, 3, 0.333332986f, 0.0f, 0.333332986f, 0.0f},
    {2.0, 2.0, 3, 0.333332986f, 0.0f, 0.333332986f, 0.0f},
    {3.0, 8.0, 3, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kAutoValues[21] = {0.0, 0.0,
                                       0.0, 0.0,
                                       0.0, 0.7812505500623954,
                                       2.5, 4.218749449937604,
                                       5.0, 4.5312496699625635,
                                       3.5, 2.4687503300374374,
                                       2.0, 2.9375006600748748,
                                       5.0, 7.062499339925126,
                                       8.0, 8.0,
                                       8.0, 8.0,
                                       8.0};

static const char kAuto4004[] = "; FBX 6.1.0 project file\n"
                                "FBXHeaderExtension: {\n"
                                " FBXHeaderVersion: 1003\n"
                                " FBXVersion: 6100\n"
                                "}\n"
                                "Objects: {\n"
                                " Model: \"Model::A\", \"Null\" {\n"
                                "  Version: 232\n"
                                " }\n"
                                "}\n"
                                "Takes: {\n"
                                " Take: \"T\" {\n"
                                "  LocalTime: 0,92372316000\n"
                                "  Model: \"Model::A\" {\n"
                                "   Channel: \"Transform\" {\n"
                                "    Channel: \"T\" {\n"
                                "     Channel: \"X\" {\n"
                                "      Default: 0.5\n"
                                "      KeyVer: 4004\n"
                                "      KeyCount: 3\n"
                                "      Key: 0,0,U,a,46186158000,5,U,a,92372316000,1,U,a\n"
                                "     }\n"
                                "    }\n"
                                "   }\n"
                                "  }\n"
                                " }\n"
                                "}\n";

static const Expected kAuto4004Keys[3] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 5.0, 3, 0.333332986f, 0.0f, 0.333332986f, 0.0f},
    {2.0, 1.0, 3, 0.333332986f, -0.0f, 0.0f, 0.0f},
};

static const double kAuto4004Values[21] = {0.0, 0.0,
                                           0.0, 0.0,
                                           0.0, 0.7812505500623954,
                                           2.5, 4.218749449937604,
                                           5.0, 4.3749995599500835,
                                           3.0, 1.6250004400499165,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0};

static const char kOthers[] =
    "; FBX 6.1.0 project file\n"
    "FBXHeaderExtension: {\n"
    " FBXHeaderVersion: 1003\n"
    " FBXVersion: 6100\n"
    "}\n"
    "Objects: {\n"
    " Model: \"Model::A\", \"Null\" {\n"
    "  Version: 232\n"
    " }\n"
    "}\n"
    "Takes: {\n"
    " Take: \"T\" {\n"
    "  LocalTime: 0,92372316000\n"
    "  Model: \"Model::A\" {\n"
    "   Channel: \"Transform\" {\n"
    "    Channel: \"T\" {\n"
    "     Channel: \"X\" {\n"
    "      Default: 0.5\n"
    "      KeyVer: 4005\n"
    "      KeyCount: 4\n"
    "      Key: "
    "0,0,U,p,1,2,n,n,46186158000,3,U,q,1,2,c,n,92372316000,1,U,t,1,2,3,138558474000,4,U,d,0,n\n"
    "     }\n"
    "    }\n"
    "   }\n"
    "  }\n"
    " }\n"
    "}\n";

static const Expected kOthersKeys[4] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 3.0, 3, 0.333332986f, 0.0f, 0.333332986f, 0.0f},
    {2.0, 1.0, 3, 0.333332986f, 0.0f, 0.333332986f, 0.0f},
    {3.0, 4.0, 3, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kOthersValues[21] = {0.0, 0.0,
                                         0.0, 0.0,
                                         0.0, 0.46875033003743727,
                                         1.5, 2.5312496699625626,
                                         3.0, 2.6874997799750417,
                                         2.0, 1.3125002200249583,
                                         1.0, 1.4687503300374374,
                                         2.5, 3.531249669962563,
                                         4.0, 4.0,
                                         4.0, 4.0,
                                         4.0};

static const char kOthers4004[] = "; FBX 6.1.0 project file\n"
                                  "FBXHeaderExtension: {\n"
                                  " FBXHeaderVersion: 1003\n"
                                  " FBXVersion: 6100\n"
                                  "}\n"
                                  "Objects: {\n"
                                  " Model: \"Model::A\", \"Null\" {\n"
                                  "  Version: 232\n"
                                  " }\n"
                                  "}\n"
                                  "Takes: {\n"
                                  " Take: \"T\" {\n"
                                  "  LocalTime: 0,92372316000\n"
                                  "  Model: \"Model::A\" {\n"
                                  "   Channel: \"Transform\" {\n"
                                  "    Channel: \"T\" {\n"
                                  "     Channel: \"X\" {\n"
                                  "      Default: 0.5\n"
                                  "      KeyVer: 4004\n"
                                  "      KeyCount: 2\n"
                                  "      Key: 0,0,U,p,1,2,n,46186158000,3,U,q,1,2,n\n"
                                  "     }\n"
                                  "    }\n"
                                  "   }\n"
                                  "  }\n"
                                  " }\n"
                                  "}\n";

static const Expected kOthers4004Keys[2] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 3.0, 3, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kOthers4004Values[21] = {0.0, 0.0,
                                             0.0, 0.0,
                                             0.0, 0.46875033003743727,
                                             1.5, 2.5312496699625626,
                                             3.0, 3.0,
                                             3.0, 3.0,
                                             3.0, 3.0,
                                             3.0, 3.0,
                                             3.0, 3.0,
                                             3.0, 3.0,
                                             3.0};

static const char kSlopes4003[] = "; FBX 6.1.0 project file\n"
                                  "FBXHeaderExtension: {\n"
                                  " FBXHeaderVersion: 1003\n"
                                  " FBXVersion: 6100\n"
                                  "}\n"
                                  "Objects: {\n"
                                  " Model: \"Model::A\", \"Null\" {\n"
                                  "  Version: 232\n"
                                  " }\n"
                                  "}\n"
                                  "Takes: {\n"
                                  " Take: \"T\" {\n"
                                  "  LocalTime: 0,92372316000\n"
                                  "  Model: \"Model::A\" {\n"
                                  "   Channel: \"Transform\" {\n"
                                  "    Channel: \"T\" {\n"
                                  "     Channel: \"X\" {\n"
                                  "      Default: 0.5\n"
                                  "      KeyVer: 4003\n"
                                  "      KeyCount: 2\n"
                                  "      Key: 0,0,U,s,1,2,46186158000,3,U,b,0,0\n"
                                  "     }\n"
                                  "    }\n"
                                  "   }\n"
                                  "  }\n"
                                  " }\n"
                                  "}\n";

static const Expected kSlopes4003Keys[2] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.333332986f},
    {1.0, 3.0, 3, 0.333332986f, 0.666665971f, 0.0f, 0.0f},
};

static const double kSlopes4003Values[21] = {0.0,
                                             0.0,
                                             0.0,
                                             0.0,
                                             0.0,
                                             0.5156252383604049,
                                             1.3750001303851604,
                                             2.296874908322968,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0,
                                             3.0};

static const char kWeights[] = "; FBX 6.1.0 project file\n"
                               "FBXHeaderExtension: {\n"
                               " FBXHeaderVersion: 1003\n"
                               " FBXVersion: 6100\n"
                               "}\n"
                               "Objects: {\n"
                               " Model: \"Model::A\", \"Null\" {\n"
                               "  Version: 232\n"
                               " }\n"
                               "}\n"
                               "Takes: {\n"
                               " Take: \"T\" {\n"
                               "  LocalTime: 0,92372316000\n"
                               "  Model: \"Model::A\" {\n"
                               "   Channel: \"Transform\" {\n"
                               "    Channel: \"T\" {\n"
                               "     Channel: \"X\" {\n"
                               "      Default: 0.5\n"
                               "      KeyVer: 4005\n"
                               "      KeyCount: 2\n"
                               "      Key: 0,0,U,s,1,1,c,46186158000,2,U,s,1,1,r,0.7\n"
                               "     }\n"
                               "    }\n"
                               "   }\n"
                               "  }\n"
                               " }\n"
                               "}\n";

static const Expected kWeightsKeys[2] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.333332986f},
    {1.0, 2.0, 3, 0.333332986f, 0.333332986f, 0.0f, 0.0f},
};

static const double kWeightsValues[21] = {0.0, 0.0,
                                          0.0, 0.0,
                                          0.0, 0.406250110012479,
                                          1.0, 1.5937498899875209,
                                          2.0, 2.0,
                                          2.0, 2.0,
                                          2.0, 2.0,
                                          2.0, 2.0,
                                          2.0, 2.0,
                                          2.0, 2.0,
                                          2.0};

static const char kLate[] = "; FBX 6.1.0 project file\n"
                            "FBXHeaderExtension: {\n"
                            " FBXHeaderVersion: 1003\n"
                            " FBXVersion: 6100\n"
                            "}\n"
                            "Objects: {\n"
                            " Model: \"Model::A\", \"Null\" {\n"
                            "  Version: 232\n"
                            " }\n"
                            "}\n"
                            "Takes: {\n"
                            " Take: \"T\" {\n"
                            "  LocalTime: 0,92372316000\n"
                            "  Model: \"Model::A\" {\n"
                            "   Channel: \"Transform\" {\n"
                            "    Channel: \"T\" {\n"
                            "     Channel: \"X\" {\n"
                            "      Default: 0.5\n"
                            "      KeyCount: 2\n"
                            "      Key: 46186158000,1,U,s,1,1,n,92372316000,3,L\n"
                            "      Pre-Extrapolation: {\n"
                            "       Type: R\n"
                            "       Repetition: 2\n"
                            "      }\n"
                            "      Post-Extrapolation: {\n"
                            "       Type: A\n"
                            "      }\n"
                            "     }\n"
                            "    }\n"
                            "   }\n"
                            "  }\n"
                            " }\n"
                            "}\n";

static const Expected kLateKeys[2] = {
    {1.0, 1.0, 3, 0.333332986f, 0.0f, 0.333332986f, 0.333332986f},
    {2.0, 3.0, 2, 0.333332986f, 0.333332986f, 0.0f, 0.0f},
};

static const double kLateValues[21] = {
    1.0, 1.4062501100124791, 2.0, 2.593749889987521,  3.0, 1.4062501100124791,
    2.0, 2.593749889987521,  1.0, 1.4062501100124791, 2.0, 2.593749889987521,
    3.0, 3.406250110012479,  4.0, 4.593749889987521,  5.0, 5.406250110012479,
    6.0, 6.593749889987521,  7.0};

static const char kMirror[] = "; FBX 6.1.0 project file\n"
                              "FBXHeaderExtension: {\n"
                              " FBXHeaderVersion: 1003\n"
                              " FBXVersion: 6100\n"
                              "}\n"
                              "Objects: {\n"
                              " Model: \"Model::A\", \"Null\" {\n"
                              "  Version: 232\n"
                              " }\n"
                              "}\n"
                              "Takes: {\n"
                              " Take: \"T\" {\n"
                              "  LocalTime: 0,92372316000\n"
                              "  Model: \"Model::A\" {\n"
                              "   Channel: \"Transform\" {\n"
                              "    Channel: \"T\" {\n"
                              "     Channel: \"X\" {\n"
                              "      Default: 0.5\n"
                              "      KeyVer: 4005\n"
                              "      KeyCount: 2\n"
                              "      Key: 0,0,L,46186158000,2,L\n"
                              "      Pre-Extrapolation: {\n"
                              "       Type: M\n"
                              "      }\n"
                              "      Post-Extrapolation: {\n"
                              "       Type: K\n"
                              "      }\n"
                              "     }\n"
                              "    }\n"
                              "   }\n"
                              "  }\n"
                              " }\n"
                              "}\n";

static const Expected kMirrorKeys[2] = {
    {0.0, 0.0, 2, 0.0f, 0.0f, 0.333332986f, 0.666665971f},
    {1.0, 2.0, 2, 0.333332986f, 0.666665971f, 0.0f, 0.0f},
};

static const double kMirrorValues[21] = {2.0, 1.5, 1.0, 0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0,
                                         3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0};

static const char kEmpty[] = "; FBX 6.1.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1003\n"
                             " FBXVersion: 6100\n"
                             "}\n"
                             "Objects: {\n"
                             " Model: \"Model::A\", \"Null\" {\n"
                             "  Version: 232\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"T\" {\n"
                             "  LocalTime: 0,92372316000\n"
                             "  Model: \"Model::A\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 0.5\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 0\n"
                             "      Key: \n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n";

static const double kEmptyValues[21] = {-7.0, -7.0, -7.0, -7.0, -7.0, -7.0, -7.0,
                                        -7.0, -7.0, -7.0, -7.0, -7.0, -7.0, -7.0,
                                        -7.0, -7.0, -7.0, -7.0, -7.0, -7.0, -7.0};

static const char kRising[] =
    "; FBX 6.1.0 project file\n"
    "FBXHeaderExtension: {\n"
    " FBXHeaderVersion: 1003\n"
    " FBXVersion: 6100\n"
    "}\n"
    "Objects: {\n"
    " Model: \"Model::A\", \"Null\" {\n"
    "  Version: 232\n"
    " }\n"
    "}\n"
    "Takes: {\n"
    " Take: \"T\" {\n"
    "  LocalTime: 0,92372316000\n"
    "  Model: \"Model::A\" {\n"
    "   Channel: \"Transform\" {\n"
    "    Channel: \"T\" {\n"
    "     Channel: \"X\" {\n"
    "      Default: 0.5\n"
    "      KeyVer: 4005\n"
    "      KeyCount: 4\n"
    "      Key: 0,0,U,a,n,46186158000,1,U,a,n,92372316000,3,U,p,1,2,n,n,138558474000,6,U,a,n\n"
    "     }\n"
    "    }\n"
    "   }\n"
    "  }\n"
    " }\n"
    "}\n";

static const Expected kRisingKeys[4] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 1.0, 3, 0.333332986f, 0.499999464f, 0.333332986f, 0.499999464f},
    {2.0, 3.0, 3, 0.333332986f, 0.833332479f, 0.333332986f, 0.833332479f},
    {3.0, 6.0, 3, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kRisingValues[21] = {0.0,
                                         0.0,
                                         0.0,
                                         0.0,
                                         0.0,
                                         0.08593763961112624,
                                         0.3125002011656761,
                                         0.6328125887958378,
                                         1.0,
                                         1.4062500649597809,
                                         1.8750001192092896,
                                         2.406250064959781,
                                         3.0,
                                         3.820312515454051,
                                         4.812499679625034,
                                         5.648437126219423,
                                         6.0,
                                         6.0,
                                         6.0,
                                         6.0,
                                         6.0};

static const char kRightweight[] =
    "; FBX 6.1.0 project file\n"
    "FBXHeaderExtension: {\n"
    " FBXHeaderVersion: 1003\n"
    " FBXVersion: 6100\n"
    "}\n"
    "Objects: {\n"
    " Model: \"Model::A\", \"Null\" {\n"
    "  Version: 232\n"
    " }\n"
    "}\n"
    "Takes: {\n"
    " Take: \"T\" {\n"
    "  LocalTime: 0,92372316000\n"
    "  Model: \"Model::A\" {\n"
    "   Channel: \"Transform\" {\n"
    "    Channel: \"T\" {\n"
    "     Channel: \"X\" {\n"
    "      Default: 0.5\n"
    "      KeyVer: 4005\n"
    "      KeyCount: 3\n"
    "      Key: 0,0,U,s,1,1,r,0.7,46186158000,2,U,s,1,1,l,0.2,92372316000,1,L\n"
    "     }\n"
    "    }\n"
    "   }\n"
    "  }\n"
    " }\n"
    "}\n";

static const Expected kRightweightKeys[3] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.699999988f, 0.699999988f},
    {1.0, 2.0, 3, 0.333332986f, 0.333332986f, 0.333332986f, 0.333332986f},
    {2.0, 1.0, 2, 0.200000003f, 0.200000003f, 0.0f, 0.0f},
};

static const double kRightweightValues[21] = {0.0,
                                              0.0,
                                              0.0,
                                              0.0,
                                              0.0,
                                              0.30151229725167705,
                                              0.7642313673457248,
                                              1.4946304005717002,
                                              2.0,
                                              1.9742045941226716,
                                              1.6349857414380513,
                                              1.2049148446945295,
                                              1.0,
                                              1.0,
                                              1.0,
                                              1.0,
                                              1.0,
                                              1.0,
                                              1.0,
                                              1.0,
                                              1.0};

static const char kConstant4004[] = "; FBX 6.1.0 project file\n"
                                    "FBXHeaderExtension: {\n"
                                    " FBXHeaderVersion: 1003\n"
                                    " FBXVersion: 6100\n"
                                    "}\n"
                                    "Objects: {\n"
                                    " Model: \"Model::A\", \"Null\" {\n"
                                    "  Version: 232\n"
                                    " }\n"
                                    "}\n"
                                    "Takes: {\n"
                                    " Take: \"T\" {\n"
                                    "  LocalTime: 0,92372316000\n"
                                    "  Model: \"Model::A\" {\n"
                                    "   Channel: \"Transform\" {\n"
                                    "    Channel: \"T\" {\n"
                                    "     Channel: \"X\" {\n"
                                    "      Default: 0.5\n"
                                    "      KeyVer: 4004\n"
                                    "      KeyCount: 2\n"
                                    "      Key: 0,1,C,n,46186158000,2,L\n"
                                    "     }\n"
                                    "    }\n"
                                    "   }\n"
                                    "  }\n"
                                    " }\n"
                                    "}\n";

static const Expected kConstant4004Keys[2] = {
    {0.0, 1.0, 1, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 2.0, 2, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kConstant4004Values[21] = {1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0,
                                               2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                                               2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

static const char kConstantother[] = "; FBX 6.1.0 project file\n"
                                     "FBXHeaderExtension: {\n"
                                     " FBXHeaderVersion: 1003\n"
                                     " FBXVersion: 6100\n"
                                     "}\n"
                                     "Objects: {\n"
                                     " Model: \"Model::A\", \"Null\" {\n"
                                     "  Version: 232\n"
                                     " }\n"
                                     "}\n"
                                     "Takes: {\n"
                                     " Take: \"T\" {\n"
                                     "  LocalTime: 0,92372316000\n"
                                     "  Model: \"Model::A\" {\n"
                                     "   Channel: \"Transform\" {\n"
                                     "    Channel: \"T\" {\n"
                                     "     Channel: \"X\" {\n"
                                     "      Default: 0.5\n"
                                     "      KeyVer: 4005\n"
                                     "      KeyCount: 2\n"
                                     "      Key: 0,1,C,x,46186158000,2,L\n"
                                     "     }\n"
                                     "    }\n"
                                     "   }\n"
                                     "  }\n"
                                     " }\n"
                                     "}\n";

static const Expected kConstantotherKeys[2] = {
    {0.0, 1.0, 0, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 2.0, 2, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kConstantotherValues[21] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                                1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                                                2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

static const char kSame[] = "; FBX 6.1.0 project file\n"
                            "FBXHeaderExtension: {\n"
                            " FBXHeaderVersion: 1003\n"
                            " FBXVersion: 6100\n"
                            "}\n"
                            "Objects: {\n"
                            " Model: \"Model::A\", \"Null\" {\n"
                            "  Version: 232\n"
                            " }\n"
                            "}\n"
                            "Takes: {\n"
                            " Take: \"T\" {\n"
                            "  LocalTime: 0,92372316000\n"
                            "  Model: \"Model::A\" {\n"
                            "   Channel: \"Transform\" {\n"
                            "    Channel: \"T\" {\n"
                            "     Channel: \"X\" {\n"
                            "      Default: 0.5\n"
                            "      KeyVer: 4005\n"
                            "      KeyCount: 3\n"
                            "      Key: 0,0,U,s,0,1e40,n,0,1,L,46186158000,2,U,s,1e40,0,n\n"
                            "     }\n"
                            "    }\n"
                            "   }\n"
                            "  }\n"
                            " }\n"
                            "}\n";

static const Expected kSameKeys[3] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.0f, 0.0f},
    {0.0, 1.0, 2, 0.0f, 0.0f, 0.333332986f, 0.333332986f},
    {1.0, 2.0, 3, 0.333332986f, 0.333332986f, 0.0f, 0.0f},
};

static const double kSameValues[21] = {0.0, 0.0, 0.0, 0.0, 1.0, 1.25, 1.5, 1.75, 2.0, 2.0, 2.0,
                                       2.0, 2.0, 2.0, 2.0, 2.0, 2.0,  2.0, 2.0,  2.0, 2.0};

static const char kNegativever[] = "; FBX 6.1.0 project file\n"
                                   "FBXHeaderExtension: {\n"
                                   " FBXHeaderVersion: 1003\n"
                                   " FBXVersion: 6100\n"
                                   "}\n"
                                   "Objects: {\n"
                                   " Model: \"Model::A\", \"Null\" {\n"
                                   "  Version: 232\n"
                                   " }\n"
                                   "}\n"
                                   "Takes: {\n"
                                   " Take: \"T\" {\n"
                                   "  LocalTime: 0,92372316000\n"
                                   "  Model: \"Model::A\" {\n"
                                   "   Channel: \"Transform\" {\n"
                                   "    Channel: \"T\" {\n"
                                   "     Channel: \"X\" {\n"
                                   "      Default: 0.5\n"
                                   "      KeyVer: -1\n"
                                   "      KeyCount: 2\n"
                                   "      Key: 0,1,C,n,46186158000,2,L\n"
                                   "     }\n"
                                   "    }\n"
                                   "   }\n"
                                   "  }\n"
                                   " }\n"
                                   "}\n";

static const Expected kNegativeverKeys[2] = {
    {0.0, 1.0, 1, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 2.0, 2, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kNegativeverValues[21] = {1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                                              2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

static const char kDefaultver[] = "; FBX 6.1.0 project file\n"
                                  "FBXHeaderExtension: {\n"
                                  " FBXHeaderVersion: 1003\n"
                                  " FBXVersion: 6100\n"
                                  "}\n"
                                  "Objects: {\n"
                                  " Model: \"Model::A\", \"Null\" {\n"
                                  "  Version: 232\n"
                                  " }\n"
                                  "}\n"
                                  "Takes: {\n"
                                  " Take: \"T\" {\n"
                                  "  LocalTime: 0,92372316000\n"
                                  "  Model: \"Model::A\" {\n"
                                  "   Channel: \"Transform\" {\n"
                                  "    Channel: \"T\" {\n"
                                  "     Channel: \"X\" {\n"
                                  "      Default: 0.5\n"
                                  "      KeyCount: 2\n"
                                  "      Key: 0,0,U,a,n,46186158000,1,L\n"
                                  "     }\n"
                                  "    }\n"
                                  "   }\n"
                                  "  }\n"
                                  " }\n"
                                  "}\n";

static const Expected kDefaultverKeys[2] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 1.0, 2, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kDefaultverValues[21] = {0.0, 0.0,
                                             0.0, 0.0,
                                             0.0, 0.1562501100124791,
                                             0.5, 0.8437498899875209,
                                             1.0, 1.0,
                                             1.0, 1.0,
                                             1.0, 1.0,
                                             1.0, 1.0,
                                             1.0, 1.0,
                                             1.0, 1.0,
                                             1.0};

static const char kVersion5[] = "; FBX 5.0.0 project file\n"
                                "FBXHeaderExtension: {\n"
                                " FBXHeaderVersion: 1003\n"
                                " FBXVersion: 5000\n"
                                "}\n"
                                "Objects: {\n"
                                " Model: \"Model::A\", \"Null\" {\n"
                                "  Version: 232\n"
                                " }\n"
                                "}\n"
                                "Takes: {\n"
                                " Take: \"T\" {\n"
                                "  LocalTime: 0,92372316000\n"
                                "  Model: \"Model::A\" {\n"
                                "   Channel: \"Transform\" {\n"
                                "    Channel: \"T\" {\n"
                                "     Channel: \"X\" {\n"
                                "      Default: 0.5\n"
                                "      KeyCount: 2\n"
                                "      Key: 0,0,U,a,46186158000,1,L\n"
                                "     }\n"
                                "    }\n"
                                "   }\n"
                                "  }\n"
                                " }\n"
                                "}\n";

static const Expected kVersion5Keys[2] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.0f},
    {1.0, 1.0, 2, 0.333332986f, 0.0f, 0.0f, 0.0f},
};

static const double kVersion5Values[21] = {0.0, 0.0,
                                           0.0, 0.0,
                                           0.0, 0.1562501100124791,
                                           0.5, 0.8437498899875209,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0, 1.0,
                                           1.0};

static const char kVersion4[] = "; FBX 4.0.0 project file\n"
                                "FBXHeaderExtension: {\n"
                                " FBXHeaderVersion: 1003\n"
                                " FBXVersion: 4000\n"
                                "}\n"
                                "Objects: {\n"
                                " Model: \"Model::A\", \"Null\" {\n"
                                "  Version: 232\n"
                                " }\n"
                                "}\n"
                                "Takes: {\n"
                                " Take: \"T\" {\n"
                                "  LocalTime: 0,92372316000\n"
                                "  Model: \"Model::A\" {\n"
                                "   Channel: \"Transform\" {\n"
                                "    Channel: \"T\" {\n"
                                "     Channel: \"X\" {\n"
                                "      Default: 0.5\n"
                                "      KeyCount: 2\n"
                                "      Key: 0,0,U,s,1,1,46186158000,1,L\n"
                                "     }\n"
                                "    }\n"
                                "   }\n"
                                "  }\n"
                                " }\n"
                                "}\n";

static const Expected kVersion4Keys[2] = {
    {0.0, 0.0, 3, 0.0f, 0.0f, 0.333332986f, 0.333332986f},
    {1.0, 1.0, 2, 0.333332986f, 0.333332986f, 0.0f, 0.0f},
};

static const double kVersion4Values[21] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.24999999999999994, 0.5, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0};

static const char kWordvalue[] = "; FBX 6.1.0 project file\n"
                                 "FBXHeaderExtension: {\n"
                                 " FBXHeaderVersion: 1003\n"
                                 " FBXVersion: 6100\n"
                                 "}\n"
                                 "Objects: {\n"
                                 " Model: \"Model::A\", \"Null\" {\n"
                                 "  Version: 232\n"
                                 " }\n"
                                 "}\n"
                                 "Takes: {\n"
                                 " Take: \"T\" {\n"
                                 "  LocalTime: 0,92372316000\n"
                                 "  Model: \"Model::A\" {\n"
                                 "   Channel: \"Transform\" {\n"
                                 "    Channel: \"T\" {\n"
                                 "     Channel: \"X\" {\n"
                                 "      Default: 0.5\n"
                                 "      KeyVer: 4005\n"
                                 "      KeyCount: 1\n"
                                 "      Key: 0,infx,L\n"
                                 "     }\n"
                                 "    }\n"
                                 "   }\n"
                                 "  }\n"
                                 " }\n"
                                 "}\n";

static const Expected kWordvalueKeys[1] = {
    {0.0, 105.0, 2, 0.0f, 0.0f, 0.0f, 0.0f},
};

static const double kWordvalueValues[21] = {105.0, 105.0, 105.0, 105.0, 105.0, 105.0, 105.0,
                                            105.0, 105.0, 105.0, 105.0, 105.0, 105.0, 105.0,
                                            105.0, 105.0, 105.0, 105.0, 105.0, 105.0, 105.0};

static const char kArray[] = "; FBX 6.1.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1003\n"
                             " FBXVersion: 6100\n"
                             "}\n"
                             "Objects: {\n"
                             " Model: \"Model::A\", \"Null\" {\n"
                             "  Version: 232\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"T\" {\n"
                             "  LocalTime: 0,92372316000\n"
                             "  Model: \"Model::A\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 0.5\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 1\n"
                             "      Key: *3 {\n"
                             "       a: 0,0,76\n"
                             "      }\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n";

static const Expected kArrayKeys[1] = {
    {0.0, 0.0, 2, 0.0f, 0.0f, 0.0f, 0.0f},
};

static const double kArrayValues[21] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

static const char kNokey[] = "; FBX 6.1.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1003\n"
                             " FBXVersion: 6100\n"
                             "}\n"
                             "Objects: {\n"
                             " Model: \"Model::A\", \"Null\" {\n"
                             "  Version: 232\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"T\" {\n"
                             "  LocalTime: 0,92372316000\n"
                             "  Model: \"Model::A\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 0.5\n"
                             "      KeyVer: 4005\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n";

static const char kNocount[] = "; FBX 6.1.0 project file\n"
                               "FBXHeaderExtension: {\n"
                               " FBXHeaderVersion: 1003\n"
                               " FBXVersion: 6100\n"
                               "}\n"
                               "Objects: {\n"
                               " Model: \"Model::A\", \"Null\" {\n"
                               "  Version: 232\n"
                               " }\n"
                               "}\n"
                               "Takes: {\n"
                               " Take: \"T\" {\n"
                               "  LocalTime: 0,92372316000\n"
                               "  Model: \"Model::A\" {\n"
                               "   Channel: \"Transform\" {\n"
                               "    Channel: \"T\" {\n"
                               "     Channel: \"X\" {\n"
                               "      Default: 0.5\n"
                               "      KeyVer: 4005\n"
                               "      Key: 0,0,L\n"
                               "     }\n"
                               "    }\n"
                               "   }\n"
                               "  }\n"
                               " }\n"
                               "}\n";

static const char kMode[] = "; FBX 6.1.0 project file\n"
                            "FBXHeaderExtension: {\n"
                            " FBXHeaderVersion: 1003\n"
                            " FBXVersion: 6100\n"
                            "}\n"
                            "Objects: {\n"
                            " Model: \"Model::A\", \"Null\" {\n"
                            "  Version: 232\n"
                            " }\n"
                            "}\n"
                            "Takes: {\n"
                            " Take: \"T\" {\n"
                            "  LocalTime: 0,92372316000\n"
                            "  Model: \"Model::A\" {\n"
                            "   Channel: \"Transform\" {\n"
                            "    Channel: \"T\" {\n"
                            "     Channel: \"X\" {\n"
                            "      Default: 0.5\n"
                            "      KeyVer: 4005\n"
                            "      KeyCount: 1\n"
                            "      Key: 0,0,X\n"
                            "     }\n"
                            "    }\n"
                            "   }\n"
                            "  }\n"
                            " }\n"
                            "}\n";

static const char kSlope[] = "; FBX 6.1.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1003\n"
                             " FBXVersion: 6100\n"
                             "}\n"
                             "Objects: {\n"
                             " Model: \"Model::A\", \"Null\" {\n"
                             "  Version: 232\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"T\" {\n"
                             "  LocalTime: 0,92372316000\n"
                             "  Model: \"Model::A\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 0.5\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 1\n"
                             "      Key: 0,0,U,z,n\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n";

static const char kWeight[] = "; FBX 6.1.0 project file\n"
                              "FBXHeaderExtension: {\n"
                              " FBXHeaderVersion: 1003\n"
                              " FBXVersion: 6100\n"
                              "}\n"
                              "Objects: {\n"
                              " Model: \"Model::A\", \"Null\" {\n"
                              "  Version: 232\n"
                              " }\n"
                              "}\n"
                              "Takes: {\n"
                              " Take: \"T\" {\n"
                              "  LocalTime: 0,92372316000\n"
                              "  Model: \"Model::A\" {\n"
                              "   Channel: \"Transform\" {\n"
                              "    Channel: \"T\" {\n"
                              "     Channel: \"X\" {\n"
                              "      Default: 0.5\n"
                              "      KeyVer: 4005\n"
                              "      KeyCount: 1\n"
                              "      Key: 0,0,U,s,0,0,z\n"
                              "     }\n"
                              "    }\n"
                              "   }\n"
                              "  }\n"
                              " }\n"
                              "}\n";

static const char kLeft[] = "; FBX 6.1.0 project file\n"
                            "FBXHeaderExtension: {\n"
                            " FBXHeaderVersion: 1003\n"
                            " FBXVersion: 6100\n"
                            "}\n"
                            "Objects: {\n"
                            " Model: \"Model::A\", \"Null\" {\n"
                            "  Version: 232\n"
                            " }\n"
                            "}\n"
                            "Takes: {\n"
                            " Take: \"T\" {\n"
                            "  LocalTime: 0,92372316000\n"
                            "  Model: \"Model::A\" {\n"
                            "   Channel: \"Transform\" {\n"
                            "    Channel: \"T\" {\n"
                            "     Channel: \"X\" {\n"
                            "      Default: 0.5\n"
                            "      KeyVer: 4005\n"
                            "      KeyCount: 1\n"
                            "      Key: 0,0,L,5\n"
                            "     }\n"
                            "    }\n"
                            "   }\n"
                            "  }\n"
                            " }\n"
                            "}\n";

static const char kShort[] = "; FBX 6.1.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1003\n"
                             " FBXVersion: 6100\n"
                             "}\n"
                             "Objects: {\n"
                             " Model: \"Model::A\", \"Null\" {\n"
                             "  Version: 232\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"T\" {\n"
                             "  LocalTime: 0,92372316000\n"
                             "  Model: \"Model::A\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 0.5\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 2\n"
                             "      Key: 0,0,L,46186158000\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n";

static const char kMissing[] = "; FBX 6.1.0 project file\n"
                               "FBXHeaderExtension: {\n"
                               " FBXHeaderVersion: 1003\n"
                               " FBXVersion: 6100\n"
                               "}\n"
                               "Objects: {\n"
                               " Model: \"Model::A\", \"Null\" {\n"
                               "  Version: 232\n"
                               " }\n"
                               "}\n"
                               "Takes: {\n"
                               " Take: \"T\" {\n"
                               "  LocalTime: 0,92372316000\n"
                               "  Model: \"Model::A\" {\n"
                               "   Channel: \"Transform\" {\n"
                               "    Channel: \"T\" {\n"
                               "     Channel: \"X\" {\n"
                               "      Default: 0.5\n"
                               "      KeyVer: 4005\n"
                               "      KeyCount: 1\n"
                               "      Key: 0,0,U,s,0\n"
                               "     }\n"
                               "    }\n"
                               "   }\n"
                               "  }\n"
                               " }\n"
                               "}\n";

static const char kFraction[] = "; FBX 6.1.0 project file\n"
                                "FBXHeaderExtension: {\n"
                                " FBXHeaderVersion: 1003\n"
                                " FBXVersion: 6100\n"
                                "}\n"
                                "Objects: {\n"
                                " Model: \"Model::A\", \"Null\" {\n"
                                "  Version: 232\n"
                                " }\n"
                                "}\n"
                                "Takes: {\n"
                                " Take: \"T\" {\n"
                                "  LocalTime: 0,92372316000\n"
                                "  Model: \"Model::A\" {\n"
                                "   Channel: \"Transform\" {\n"
                                "    Channel: \"T\" {\n"
                                "     Channel: \"X\" {\n"
                                "      Default: 0.5\n"
                                "      KeyVer: 4005\n"
                                "      KeyCount: 1\n"
                                "      Key: 0.5,0,L\n"
                                "     }\n"
                                "    }\n"
                                "   }\n"
                                "  }\n"
                                " }\n"
                                "}\n";

static const char kDecrease[] = "; FBX 6.1.0 project file\n"
                                "FBXHeaderExtension: {\n"
                                " FBXHeaderVersion: 1003\n"
                                " FBXVersion: 6100\n"
                                "}\n"
                                "Objects: {\n"
                                " Model: \"Model::A\", \"Null\" {\n"
                                "  Version: 232\n"
                                " }\n"
                                "}\n"
                                "Takes: {\n"
                                " Take: \"T\" {\n"
                                "  LocalTime: 0,92372316000\n"
                                "  Model: \"Model::A\" {\n"
                                "   Channel: \"Transform\" {\n"
                                "    Channel: \"T\" {\n"
                                "     Channel: \"X\" {\n"
                                "      Default: 0.5\n"
                                "      KeyVer: 4005\n"
                                "      KeyCount: 2\n"
                                "      Key: 46186158000,0,L,0,1,L\n"
                                "     }\n"
                                "    }\n"
                                "   }\n"
                                "  }\n"
                                " }\n"
                                "}\n";

static const char kString[] = "; FBX 6.1.0 project file\n"
                              "FBXHeaderExtension: {\n"
                              " FBXHeaderVersion: 1003\n"
                              " FBXVersion: 6100\n"
                              "}\n"
                              "Objects: {\n"
                              " Model: \"Model::A\", \"Null\" {\n"
                              "  Version: 232\n"
                              " }\n"
                              "}\n"
                              "Takes: {\n"
                              " Take: \"T\" {\n"
                              "  LocalTime: 0,92372316000\n"
                              "  Model: \"Model::A\" {\n"
                              "   Channel: \"Transform\" {\n"
                              "    Channel: \"T\" {\n"
                              "     Channel: \"X\" {\n"
                              "      Default: 0.5\n"
                              "      KeyVer: 4005\n"
                              "      KeyCount: 1\n"
                              "      Key: 0,0,\"L\"\n"
                              "     }\n"
                              "    }\n"
                              "   }\n"
                              "  }\n"
                              " }\n"
                              "}\n";

static const char kNan[] = "; FBX 6.1.0 project file\n"
                           "FBXHeaderExtension: {\n"
                           " FBXHeaderVersion: 1003\n"
                           " FBXVersion: 6100\n"
                           "}\n"
                           "Objects: {\n"
                           " Model: \"Model::A\", \"Null\" {\n"
                           "  Version: 232\n"
                           " }\n"
                           "}\n"
                           "Takes: {\n"
                           " Take: \"T\" {\n"
                           "  LocalTime: 0,92372316000\n"
                           "  Model: \"Model::A\" {\n"
                           "   Channel: \"Transform\" {\n"
                           "    Channel: \"T\" {\n"
                           "     Channel: \"X\" {\n"
                           "      Default: 0.5\n"
                           "      KeyVer: 4005\n"
                           "      KeyCount: 1\n"
                           "      Key: 0,nan,L\n"
                           "     }\n"
                           "    }\n"
                           "   }\n"
                           "  }\n"
                           " }\n"
                           "}\n";

static const char kMany[] = "; FBX 6.1.0 project file\n"
                            "FBXHeaderExtension: {\n"
                            " FBXHeaderVersion: 1003\n"
                            " FBXVersion: 6100\n"
                            "}\n"
                            "Objects: {\n"
                            " Model: \"Model::A\", \"Null\" {\n"
                            "  Version: 232\n"
                            " }\n"
                            "}\n"
                            "Takes: {\n"
                            " Take: \"T\" {\n"
                            "  LocalTime: 0,92372316000\n"
                            "  Model: \"Model::A\" {\n"
                            "   Channel: \"Transform\" {\n"
                            "    Channel: \"T\" {\n"
                            "     Channel: \"X\" {\n"
                            "      Default: 0.5\n"
                            "      KeyVer: 4005\n"
                            "      KeyCount: 2\n"
                            "      Key: 0,0,L\n"
                            "     }\n"
                            "    }\n"
                            "   }\n"
                            "  }\n"
                            " }\n"
                            "}\n";

static const char kWrapped[] = "; FBX 6.1.0 project file\n"
                               "FBXHeaderExtension: {\n"
                               " FBXHeaderVersion: 1003\n"
                               " FBXVersion: 6100\n"
                               "}\n"
                               "Objects: {\n"
                               " Model: \"Model::A\", \"Null\" {\n"
                               "  Version: 232\n"
                               " }\n"
                               "}\n"
                               "Takes: {\n"
                               " Take: \"T\" {\n"
                               "  LocalTime: 0,92372316000\n"
                               "  Model: \"Model::A\" {\n"
                               "   Channel: \"Transform\" {\n"
                               "    Channel: \"T\" {\n"
                               "     Channel: \"X\" {\n"
                               "      Default: 0.5\n"
                               "      KeyVer: 4005\n"
                               "      KeyCount: 1\n"
                               "      Key: 0,0,332\n"
                               "     }\n"
                               "    }\n"
                               "   }\n"
                               "  }\n"
                               " }\n"
                               "}\n";

static const char kWeightshort[] = "; FBX 6.1.0 project file\n"
                                   "FBXHeaderExtension: {\n"
                                   " FBXHeaderVersion: 1003\n"
                                   " FBXVersion: 6100\n"
                                   "}\n"
                                   "Objects: {\n"
                                   " Model: \"Model::A\", \"Null\" {\n"
                                   "  Version: 232\n"
                                   " }\n"
                                   "}\n"
                                   "Takes: {\n"
                                   " Take: \"T\" {\n"
                                   "  LocalTime: 0,92372316000\n"
                                   "  Model: \"Model::A\" {\n"
                                   "   Channel: \"Transform\" {\n"
                                   "    Channel: \"T\" {\n"
                                   "     Channel: \"X\" {\n"
                                   "      Default: 0.5\n"
                                   "      KeyVer: 4005\n"
                                   "      KeyCount: 1\n"
                                   "      Key: 0,0,U,s,0,0,a\n"
                                   "     }\n"
                                   "    }\n"
                                   "   }\n"
                                   "  }\n"
                                   " }\n"
                                   "}\n";

static const char kModeextra[] = "; FBX 6.1.0 project file\n"
                                 "FBXHeaderExtension: {\n"
                                 " FBXHeaderVersion: 1003\n"
                                 " FBXVersion: 6100\n"
                                 "}\n"
                                 "Objects: {\n"
                                 " Model: \"Model::A\", \"Null\" {\n"
                                 "  Version: 232\n"
                                 " }\n"
                                 "}\n"
                                 "Takes: {\n"
                                 " Take: \"T\" {\n"
                                 "  LocalTime: 0,92372316000\n"
                                 "  Model: \"Model::A\" {\n"
                                 "   Channel: \"Transform\" {\n"
                                 "    Channel: \"T\" {\n"
                                 "     Channel: \"X\" {\n"
                                 "      Default: 0.5\n"
                                 "      KeyVer: 4005\n"
                                 "      KeyCount: 1\n"
                                 "      Key: 0,0,X,n\n"
                                 "     }\n"
                                 "    }\n"
                                 "   }\n"
                                 "  }\n"
                                 " }\n"
                                 "}\n";

static const Case kCases[] = {
    {"linear", kLinear, maniImportReasonNone, true, 3, kLinearKeys, 0, -1, 0, -1, kLinearValues},
    {"constant", kConstant, maniImportReasonNone, true, 3, kConstantKeys, 0, -1, 0, -1,
     kConstantValues},
    {"constant4003", kConstant4003, maniImportReasonNone, true, 2, kConstant4003Keys, 0, -1, 0, -1,
     kConstant4003Values},
    {"user", kUser, maniImportReasonNone, true, 4, kUserKeys, 0, -1, 0, -1, kUserValues},
    {"auto", kAuto, maniImportReasonNone, true, 4, kAutoKeys, 0, -1, 0, -1, kAutoValues},
    {"auto4004", kAuto4004, maniImportReasonNone, true, 3, kAuto4004Keys, 0, -1, 0, -1,
     kAuto4004Values},
    {"others", kOthers, maniImportReasonNone, true, 4, kOthersKeys, 0, -1, 0, -1, kOthersValues},
    {"others4004", kOthers4004, maniImportReasonNone, true, 2, kOthers4004Keys, 0, -1, 0, -1,
     kOthers4004Values},
    {"slopes4003", kSlopes4003, maniImportReasonNone, true, 2, kSlopes4003Keys, 0, -1, 0, -1,
     kSlopes4003Values},
    {"weights", kWeights, maniImportReasonNone, true, 2, kWeightsKeys, 0, -1, 0, -1,
     kWeightsValues},
    {"late", kLate, maniImportReasonNone, true, 2, kLateKeys, 1, 2, 2, -1, kLateValues},
    {"mirror", kMirror, maniImportReasonNone, true, 2, kMirrorKeys, 3, -1, 4, -1, kMirrorValues},
    {"empty", kEmpty, maniImportReasonNone, true, 0, NULL, 0, -1, 0, -1, kEmptyValues},
    {"rising", kRising, maniImportReasonNone, true, 4, kRisingKeys, 0, -1, 0, -1, kRisingValues},
    {"rightweight", kRightweight, maniImportReasonNone, true, 3, kRightweightKeys, 0, -1, 0, -1,
     kRightweightValues},
    {"constant4004", kConstant4004, maniImportReasonNone, true, 2, kConstant4004Keys, 0, -1, 0, -1,
     kConstant4004Values},
    {"constantother", kConstantother, maniImportReasonNone, true, 2, kConstantotherKeys, 0, -1, 0,
     -1, kConstantotherValues},
    {"same", kSame, maniImportReasonNone, true, 3, kSameKeys, 0, -1, 0, -1, kSameValues},
    {"negativever", kNegativever, maniImportReasonNone, true, 2, kNegativeverKeys, 0, -1, 0, -1,
     kNegativeverValues},
    {"defaultver", kDefaultver, maniImportReasonNone, true, 2, kDefaultverKeys, 0, -1, 0, -1,
     kDefaultverValues},
    {"version5", kVersion5, maniImportReasonNone, true, 2, kVersion5Keys, 0, -1, 0, -1,
     kVersion5Values},
    {"version4", kVersion4, maniImportReasonNone, true, 2, kVersion4Keys, 0, -1, 0, -1,
     kVersion4Values},
    {"wordvalue", kWordvalue, maniImportReasonNone, true, 1, kWordvalueKeys, 0, -1, 0, -1,
     kWordvalueValues},
    {"array", kArray, maniImportReasonNone, true, 1, kArrayKeys, 0, -1, 0, -1, kArrayValues},
    {"nokey", kNokey, maniImportReasonNone, false, 0, NULL, 0, -1, 0, -1, NULL},
    {"nocount", kNocount, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"mode", kMode, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"slope", kSlope, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"weight", kWeight, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"left", kLeft, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"short", kShort, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"missing", kMissing, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"fraction", kFraction, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"decrease", kDecrease, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"string", kString, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"nan", kNan, maniImportReasonData, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"many", kMany, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"wrapped", kWrapped, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"weightshort", kWeightshort, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
    {"modeextra", kModeextra, maniImportReasonProperty, false, 0, NULL, 0, 0, 0, 0, NULL},
};

// The first node holding a Default child: the channel.
static uint32_t ChannelNode(const maniFbx* fbx)
{
    for (uint32_t n = 0; n < fbx->nodeCount; ++n)
    {
        if (maniFbxFindChild(fbx, n, "Default", 7) != MANI_FBX_NONE)
        {
            return n;
        }
    }
    return MANI_FBX_NONE;
}

static bool SameKeys(const maniFbxCurve* curve, const Case* c)
{
    bool right = curve->count == c->count;
    for (uint32_t k = 0; right && k < c->count; ++k)
    {
        const maniFbxKey* a = &curve->keys[k];
        const Expected* b = &c->keys[k];
        right = a->time == b->time && a->value == b->value &&
                a->interpolation == b->interpolation && a->leftDx == (double)b->leftDx &&
                a->leftDy == (double)b->leftDy && a->rightDx == (double)b->rightDx &&
                a->rightDy == (double)b->rightDy;
    }
    return right;
}

static bool SameValues(const maniFbxCurve* curve, const Case* c)
{
    bool right = true;
    for (int s = 0; right && s < 21; ++s)
    {
        double v = maniFbxEvaluateCurve(curve, (s - 4) * 0.25, -7.0);
        right = fabs(v - c->values[s]) <= 1e-9 * (1.0 + fabs(c->values[s]));
    }
    return right;
}

static void Run(const Case* c)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    maniFbx* fbx = NULL;
    bool opened = maniCreateBuilder(&def, &builder) == mani_success &&
                  maniOpenFbx(builder, c->text, strlen(c->text), &fbx).result == mani_success;
    uint32_t node = opened ? ChannelNode(fbx) : MANI_FBX_NONE;
    maniFbxCurve curve;
    maniImportResult r = node != MANI_FBX_NONE ? maniFbxReadKeys(fbx, node, 9, &curve)
                                               : (maniImportResult){mani_errorInvalid, 0, 0, 0};
    bool right = false;
    if (c->refusal != maniImportReasonNone)
    {
        right = r.result == mani_errorFormat && r.reason == c->refusal &&
                (c->refusal != maniImportReasonProperty || r.item == 9);
    }
    else if (r.result == mani_success)
    {
        right = c->curve ? SameKeys(&curve, c) && curve.pre.mode == c->pre &&
                               curve.pre.repeat == c->preRepeat && curve.post.mode == c->post &&
                               curve.post.repeat == c->postRepeat && SameValues(&curve, c)
                         : curve.count == 0 && curve.keys == NULL;
        maniFbxReleaseCurve(fbx, &curve);
    }
    if (!right)
    {
        printf("case %s: result %d reason %d\n", c->name, (int)r.result, (int)r.reason);
    }
    CHECK(right, c->name);
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(void)
{
    for (size_t k = 0; k < sizeof kCases / sizeof kCases[0]; ++k)
    {
        Run(&kCases[k]);
    }
    return s_failures == 0 ? 0 : 1;
}
