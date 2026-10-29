// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Reading a graph's JSON form against graph_json_assets.h's
// skeleton, clips, database, model and spring set: any text is refused
// or built, and a graph built opens. Run as fuzz_graph_json <corpus
// dir>; the seeds come from test_graph_json --seed <file> and its
// kin, nested subgraphs from test_graph_nested.

#include "graph_json_assets.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_builder == NULL)
    {
        SetupGraphAssets();
    }
    maniBuildResult r =
        maniBuildGraphJson(s_builder, (const char*)data, size,
                           &(maniGraphNamedAssets){&s_skeleton, kHost, kHostDatabase, kHostModel,
                                                   kHostSet, 3, 1, 1, 1, NULL});
    if (r.result != mani_success)
    {
        bool text = r.reason == maniBuildReasonJson || r.reason == maniBuildReasonForm;
        if (r.result == mani_errorInvalid && text && r.item > size)
        {
            abort();
        }
        return 0;
    }
    size_t fileSize = 0;
    maniGraph graph;
    if (maniBuilder_CopyBuilt(s_builder, s_file, sizeof s_file, &fileSize) == mani_success &&
        maniOpenGraph(s_file, fileSize, &graph).result != mani_success)
    {
        abort();
    }
    return 0;
}
