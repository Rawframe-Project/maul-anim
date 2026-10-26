#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Runs fuzz targets of a build, and minimizes their corpora.
#
#   tools/fuzz.sh run <build dir> <seconds> [target ...]
#   tools/fuzz.sh merge <build dir> [target ...]
#
# Without a target name, every fuzz_* program of the build. `run` fuzzes
# each target for the same time, four at a time, through the family's
# tools/fuzz_target.sh. A target grows its corpus in out/corpus/<target>
# and also reads its committed seeds in test/seeds/<target> and, for the
# importers, the sample files of test/data. Its log goes to
# out/fuzz-logs/<target>.log and its findings to
# out/fuzz-crashes/<target>/; on a finding, its report, which never holds
# the input, is printed once every target has finished.
#
# `merge` minimizes each corpus with libFuzzer's merge: the seeds and the
# corpus are read together, and only the files that keep every feature
# either reaches stay, seeds among them. The nightly run merges before it
# saves a corpus to its cache. A merge reruns every input, so one that
# crashes is dumped as libFuzzer dumps a finding: its output goes only to
# the target's log, and a failed merge prints the target and its status.
set -euo pipefail

usage() {
    echo "usage: tools/fuzz.sh run <build dir> <seconds> [target ...]" >&2
    echo "       tools/fuzz.sh merge <build dir> [target ...]" >&2
    exit 2
}

[ $# -ge 2 ] || usage
mode=$1
build=$2
shift 2
case "$mode" in
run)
    [ $# -ge 1 ] || usage
    case "$1" in '' | *[!0-9]* | 0) usage ;; esac
    export FUZZ_SECONDS=$1
    shift
    ;;
merge) ;;
*) usage ;;
esac

cd "$(dirname "$0")/.."
if [ $# -gt 0 ]; then
    targets=()
    for name in "$@"; do targets+=("$build/$name"); done
else
    mapfile -t targets < <(find "$build" -maxdepth 1 -type f -name 'fuzz_*' -perm -u+x | sort)
fi
if [ ${#targets[@]} -eq 0 ]; then
    echo "no fuzz target in $build" >&2
    exit 1
fi
for target in "${targets[@]}"; do
    [ -x "$target" ] || {
        echo "$target was not built" >&2
        exit 1
    }
done
mkdir -p out/fuzz-logs

# The directories a target reads besides its own corpus.
inputs() {
    [ -d "test/seeds/$1" ] && echo "test/seeds/$1"
    case "$1" in
    fuzz_gltf) echo test/data/gltf ;;
    fuzz_draco) echo test/data/draco ;;
    fuzz_fbx) echo test/data/fbx ;;
    esac
}

fuzz_one() {
    local name
    name=$(basename "$1")
    mkdir -p "out/corpus/$name"
    mapfile -t read_also < <(inputs "$name")
    tools/fuzz_target.sh "$1" -max_total_time="$FUZZ_SECONDS" "out/corpus/$name" "${read_also[@]}"
}

# The reports are cleared first, so that only this run's targets report,
# and printed once they have all finished, so that none interleaves with
# another.
if [ "$mode" = run ]; then
    export -f fuzz_one inputs
    rm -rf out/fuzz-reports
    status=0
    printf '%s\n' "${targets[@]}" | xargs -P 4 -I {} bash -c 'fuzz_one "$1"' _ {} || status=$?
    cat out/fuzz-reports/*.txt 2>/dev/null || true
    exit "$status"
fi

for target in "${targets[@]}"; do
    name=$(basename "$target")
    mapfile -t read_also < <(inputs "$name")
    mkdir -p "out/corpus/$name"
    rm -rf "out/corpus-merged/$name"
    mkdir -p "out/corpus-merged/$name"
    mkdir -p "out/fuzz-crashes/$name"
    status=0
    "$target" -merge=1 -artifact_prefix="out/fuzz-crashes/$name/" "out/corpus-merged/$name" \
        "${read_also[@]}" "out/corpus/$name" >>"out/fuzz-logs/$name.log" 2>&1 || status=$?
    if [ "$status" -ne 0 ]; then
        echo "::error::$name: the merge failed (exit $status)"
        exit 1
    fi
    rm -rf "out/corpus/$name"
    mv "out/corpus-merged/$name" "out/corpus/$name"
    echo "$name: $(find "out/corpus/$name" -type f | wc -l) files kept"
done
