#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The size the core is held to: ozz-animation's runtime, its base and
# animation runtime libraries (the sources of its ozz_base and
# ozz_animation targets), compiled as the core's size builds are (GCC 14,
# Release, -O2 -DNDEBUG, ozz's own flags adding only warnings) and counted
# as tools/size_report.py counts a part: size(1)'s text, which holds the
# read-only data and unwind tables too. ozz's own CMake asks for 3.30, so
# the sources are compiled here directly.
#
#   tools/ozz_runtime_size.sh <ozz source> [level]
#
# The source is ozz-animation at the commit the differentials pin
# (out/linux-differential/_deps/ozz-src after configuring that preset);
# the level is O2 (the default) or O3. Prints each library's text and
# their sum.

set -euo pipefail

usage() {
    echo "usage: tools/ozz_runtime_size.sh <ozz source> [O2|O3]" >&2
    exit 2
}

[ $# -ge 1 ] || usage
src=$1
level=${2:-O2}
case $level in O2 | O3) ;; *) usage ;; esac
[ -f "$src/src/base/platform.cc" ] || usage

base=(memory/allocator.cc platform.cc log.cc containers/string_archive.cc encode/group_varint.cc
    io/archive.cc io/stream.cc maths/box.cc maths/simd_math.cc maths/math_archive.cc
    maths/soa_math_archive.cc maths/simd_math_archive.cc)
animation=(animation.cc animation_utils.cc blending_job.cc ik_aim_job.cc ik_two_bone_job.cc
    local_to_model_job.cc motion_blending_job.cc sampling_job.cc skeleton.cc skeleton_utils.cc
    track.cc track_sampling_job.cc track_triggering_job.cc)

out=$(mktemp -d)
trap 'rm -r -- "$out"' EXIT

compile() {
    local dir=$1
    shift
    for file in "$@"; do
        g++-14 -"$level" -DNDEBUG -std=c++17 -w -I "$src/include" -I "$src/src" -c "$src/$dir/$file" \
            -o "$out/$(echo "$dir/$file" | tr / _).o"
    done
}

text() {
    size "$@" | awk 'NR > 1 { sum += $1 } END { print sum }'
}

compile src/base "${base[@]}"
compile src/animation/runtime "${animation[@]}"
b=$(text "$out"/src_base_*.o)
a=$(text "$out"/src_animation_runtime_*.o)
echo "ozz-animation runtime, GCC 14 -$level: base $b, animation $a, total $((b + a)) bytes of text"
