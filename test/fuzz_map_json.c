// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Reading the maps' JSON forms against map_json_assets.h's
// rigs: any text is refused or built, as a retargeting map and as a
// mirror map, a refusal of the text points inside it, and a map built
// opens. Run as fuzz_map_json <corpus dir>; the seeds come from
// test_retarget_json --seed <file> and test_mirror_json --seed <file>.

#include "map_json_assets.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static _Alignas(16) uint8_t s_rigFiles[3][4096];
static _Alignas(16) uint8_t s_file[65536];
static maniSkeleton s_rigs[3];
static maniBuilder* s_builder;

static void Setup(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    if (!OpenRig(kSourceRig, 8, s_rigFiles[0], &s_rigs[0]) ||
        !OpenRig(kTargetRig, 10, s_rigFiles[1], &s_rigs[1]) ||
        !OpenRig(kMirrorRig, kBones, s_rigFiles[2], &s_rigs[2]) ||
        maniCreateBuilder(&def, &s_builder) != mani_success)
    {
        abort();
    }
}

// A refusal of the text points inside it; a map built opens.
static void Check(maniBuildResult r, size_t size, bool mirror)
{
    if (r.result != mani_success)
    {
        bool text = r.reason == maniBuildReasonJson || r.reason == maniBuildReasonForm;
        if (r.result == mani_errorInvalid && text && r.item > size)
        {
            abort();
        }
        return;
    }
    size_t fileSize = 0;
    maniRetargetMap retarget;
    maniMirrorMap map;
    if (maniCopyBuilt(s_builder, s_file, sizeof s_file, &fileSize) == mani_success &&
        (mirror ? maniOpenMirrorMap(s_file, fileSize, &map).result
                : maniOpenRetargetMap(s_file, fileSize, &retarget).result) != mani_success)
    {
        abort();
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_builder == NULL)
    {
        Setup();
    }
    const char* text = (const char*)data;
    Check(maniBuildRetargetMapJson(s_builder, text, size, &s_rigs[0], &s_rigs[1]), size, false);
    Check(maniBuildMirrorMapJson(s_builder, text, size, &s_rigs[2]), size, true);
    return 0;
}
