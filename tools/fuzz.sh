#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Runs every fuzz target of a build for the same time, four at a time.
# Each target grows its corpus in out/corpus/<target> and also reads its
# committed seeds in test/seeds/<target> and, for the importers, the
# sample files of test/data. A target's output goes to
# out/fuzz-logs/<target>.log and is printed when it finds something; the
# finding's input is written to the working directory.
#
#   tools/fuzz.sh <build dir> <seconds> [--merge]
#
# With --merge, each corpus is then minimized: only the inputs that reach
# code no other input reaches are kept, the seeds' coverage counted first.
# The nightly run merges before it saves the corpus to its cache.
set -euo pipefail

if [ $# -lt 2 ] || [ $# -gt 3 ] || { [ $# -eq 3 ] && [ "$3" != --merge ]; }; then
    echo "usage: tools/fuzz.sh <build dir> <seconds> [--merge]" >&2
    exit 2
fi
build=$1
export FUZZ_SECONDS=$2
merge=${3:-}

cd "$(dirname "$0")/.."
mapfile -t targets < <(find "$build" -maxdepth 1 -type f -name 'fuzz_*' -perm -u+x | sort)
if [ ${#targets[@]} -eq 0 ]; then
    echo "no fuzz target in $build" >&2
    exit 1
fi
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
    if "$1" -max_total_time="$FUZZ_SECONDS" "out/corpus/$name" "${read_also[@]}" \
        >"out/fuzz-logs/$name.log" 2>&1; then
        echo "$name: no finding"
    else
        tail -n 60 "out/fuzz-logs/$name.log"
        echo "::error::$name found something"
        return 1
    fi
}
export -f fuzz_one inputs
printf '%s\n' "${targets[@]}" | xargs -P 4 -I {} bash -c 'fuzz_one "$1"' _ {}

[ "$merge" = --merge ] || exit 0
for target in "${targets[@]}"; do
    name=$(basename "$target")
    mapfile -t read_also < <(inputs "$name")
    rm -rf "out/corpus-merged/$name"
    mkdir -p "out/corpus-merged/$name"
    # The seeds are read first and kept; a grown input stays only for the
    # code they miss.
    "$target" -merge=1 "out/corpus-merged/$name" "${read_also[@]}" "out/corpus/$name" \
        >>"out/fuzz-logs/$name.log" 2>&1
    rm -rf "out/corpus/$name"
    mv "out/corpus-merged/$name" "out/corpus/$name"
    echo "$name: $(find "out/corpus/$name" -type f | wc -l) inputs kept"
done
