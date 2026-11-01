#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The checks, in three tiers. Each tier includes the one before it.
#
#   tools/check.sh changed   the files changed against origin/main: the
#                            source rules, formatting, the documentation
#                            checks, clang-tidy in Release and in Debug on
#                            the changed sources, and the
#                            linux-clang-20-asan preset (assertions on),
#                            the whole suite or the suites the changes
#                            reach. Under two minutes; run it before every
#                            push.
#   tools/check.sh pr        what the pull request check runs on this
#                            machine: every check, the generated files,
#                            the documentation site, clang-tidy, the
#                            linux-clang-20-asan and linux-gcc-14 presets,
#                            the nightly's differential tools built,
#                            and a short fuzzing pass.
#   tools/check.sh full      also the other Linux presets (ThreadSanitizer,
#                            shared with the package test, Clang Release),
#                            the size budget and longer fuzzing.
#
# The builds are the CI cells' CMake presets (CMakePresets.json), in
# out/<preset>; the other trees go to out/check-*. Needs gcc-14, clang-20,
# clang-tidy-20, ninja, pkg-config, the clang-format version CI pins and
# Python's markdown package: the organization's CI image has them all. The
# organization's documentation check runs when its .github checkout is at
# ORG_DEFAULTS (default: ../../.github from the repository). On the shared
# development machine, run the pr and full tiers through the workspace's
# heavy-job wrapper.
set -euo pipefail

tier=${1:-changed}
case "$tier" in
changed | pr | full) ;;
*)
    echo "usage: tools/check.sh [changed|pr|full]" >&2
    exit 2
    ;;
esac

cd "$(dirname "$0")/.."
root=$PWD
out=$root/out
mkdir -p "$out"
org=${ORG_DEFAULTS:-$root/../../.github}
jobs=${JOBS:-8}

step() { printf '\n== %s\n' "$*"; }

# A tree outside the presets (the size builds).
configure() { # name, then cmake arguments
    local name=check-$1
    shift
    cmake -S "$root" -B "$out/$name" -G Ninja -DMAUL_ANIM_WERROR=ON "$@" >"$out/$name.log"
}

build() { cmake --build "$out/check-$1" -j "$jobs" >>"$out/check-$1.log"; }

# A CI cell: its configure, build and test presets. More ctest arguments
# (a label filter) follow the preset's name.
preset() {
    local name=$1
    shift
    step "preset $name"
    cmake --preset "$name" >"$out/$name.log"
    python3 tools/check_test_hygiene.py --ctest "out/$name"
    cmake --build --preset "$name" -j "$jobs" >>"$out/$name.log"
    ctest --preset "$name" -j "$jobs" "$@"
}

# The ctest labels a changed path reaches: a suite's own file reaches its
# suite, the user guide reaches the guide's suites, and anything else the
# library is built from reaches every suite ("all").
labels_for() {
    case "$1" in
    *.md | LICENSE | .github/* | tools/check_*.py | tools/build_docs.py | tools/gen_api.py | tools/*.sh) ;;
    tools/size-budget.txt | tools/size_report.py | tools/fbx-deviations.txt) ;;
    test/fuzz_* | test/seeds/* | bench/*) ;;
    docs/user/*) echo "guide guide_import" ;;
    test/test_*.c) basename "$1" .c | sed 's/^test_//' ;;
    *) echo all ;;
    esac
}

source_checks() {
    step "source, module, length, documentation, snippet and test hygiene rules"
    python3 tools/check_source.py
    python3 tools/check_modules.py
    python3 tools/check_lengths.py
    python3 tools/check_docs.py
    python3 tools/check_float_env.py
    python3 tools/check_guide.py
    python3 tools/check_test_hygiene.py --self-test
    python3 tools/check_test_hygiene.py
    if [ -f "$org/scripts/docs_check.py" ]; then
        python3 "$org/scripts/docs_check.py" --exclude test/data
    else
        echo "skipped the organization's documentation check: no checkout at $org"
    fi
}

all_c_files() { git ls-files 'include/*.h' 'src/*.[ch]' 'test/*.[ch]' 'tools/*.[ch]' 'bench/*.[ch]'; }

# clang-tidy on the files given, as CI's checks job runs it: on the
# Release preset, where the assertions compile out, and with them live.
# Each hides findings the other shows.
tidy() { # files
    local build
    for build in Release Debug; do
        step "clang-tidy, $build, on $# files"
        cmake --preset linux-clang-20 -B "$out/check-tidy-$build" -DCMAKE_BUILD_TYPE=$build \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DMAUL_ANIM_BUILD_TESTS=OFF -DMAUL_ANIM_BUILD_BENCH=OFF \
            >"$out/check-tidy-$build.log"
        printf '%s\n' "$@" | xargs -P "$jobs" -n 4 clang-tidy-20 -p "$out/check-tidy-$build" --quiet \
            --warnings-as-errors='*' 2>/dev/null
    done
}

if [ "$tier" = changed ]; then
    base=$(git merge-base HEAD origin/main 2>/dev/null || echo HEAD)
    mapfile -t changed < <({
        git diff --name-only "$base"
        git ls-files --others --exclude-standard
    } | sort -u)
    if [ ${#changed[@]} -eq 0 ]; then
        echo "nothing changed against origin/main"
        exit 0
    fi
    source_checks
    mapfile -t c_files < <(printf '%s\n' "${changed[@]}" | grep -E '\.[ch]$' | grep -v '^src/generated/' || true)
    present=()
    for f in "${c_files[@]+"${c_files[@]}"}"; do [ -f "$f" ] && present+=("$f"); done
    if [ ${#present[@]} -gt 0 ]; then
        step "clang-format on ${#present[@]} changed files"
        clang-format --dry-run -Werror "${present[@]}"
    fi
    # The pr tier's set, git ls-files 'src/*.c', narrowed to the changed
    # files still present.
    sources=()
    for f in "${changed[@]}"; do [[ $f == src/*.c && -f $f ]] && sources+=("$f"); done
    if [ ${#sources[@]} -gt 0 ]; then
        tidy "${sources[@]}"
    fi
    labels=""
    for f in "${changed[@]}"; do labels="$labels $(labels_for "$f")"; done
    if [ -z "${labels// /}" ]; then
        echo "no test reached by the changes"
        exit 0
    fi
    if [[ " $labels " == *" all "* ]]; then
        preset linux-clang-20-asan
    else
        regex=$(printf '%s\n' $labels | sort -u | paste -sd '|')
        echo "tests labeled $regex"
        preset linux-clang-20-asan -L "^($regex)\$"
    fi
    exit 0
fi

source_checks

step "clang-format on every file"
mapfile -t files < <(all_c_files)
[ ${#files[@]} -gt 0 ] || {
    echo "no C file found" >&2
    exit 1
}
clang-format --dry-run -Werror "${files[@]}"

step "generated files match their generators"
for generator in gen_math_constants gen_decimal_powers gen_inflate_tables gen_number_cases gen_format_cases \
    gen_gltf_expect; do
    python3 "tools/$generator.py" --check
done

step "the documentation site builds"
python3 tools/build_docs.py

mapfile -t sources < <(git ls-files 'src/*.c')
[ ${#sources[@]} -gt 0 ] || {
    echo "no source file for clang-tidy" >&2
    exit 1
}
tidy "${sources[@]}"

preset linux-clang-20-asan
preset linux-gcc-14

# The nightly runs these against ufbx, ACL, ozz-animation and Godot; a
# change to the headers must still compile them.
step "the nightly's differential tools build"
cmake --preset linux-differential >"$out/linux-differential.log"
cmake --build --preset linux-differential -j "$jobs" --target fbx_differential acl_differential \
    ozz_differential ik_differential retarget_godot >>"$out/linux-differential.log"

fuzz_seconds=10
[ "$tier" = full ] && fuzz_seconds=60
step "fuzzing, $fuzz_seconds seconds per target"
cmake --preset linux-fuzz >"$out/linux-fuzz.log"
cmake --build --preset linux-fuzz -j "$jobs" >>"$out/linux-fuzz.log"
tools/fuzz.sh run out/linux-fuzz "$fuzz_seconds"

[ "$tier" = full ] || exit 0

for name in linux-clang-20-tsan linux-shared linux-clang-20; do
    preset "$name"
done

step "the runtime's size and allocators"
for level in O2 O3; do
    for simd in scalar auto; do
        configure "size-$level-$simd" -DCMAKE_C_COMPILER=gcc-14 -DCMAKE_BUILD_TYPE=Release \
            "-DCMAKE_C_FLAGS_RELEASE=-$level -DNDEBUG" -DMAUL_ANIM_OFFLINE=OFF -DMAUL_ANIM_SIMD=$simd \
            -DMAUL_ANIM_BUILD_TESTS=OFF -DMAUL_ANIM_BUILD_BENCH=OFF
        build "size-$level-$simd"
    done
    python3 tools/size_report.py "$out/check-size-$level-scalar/libmaul-anim.a" --check "$level" >/dev/null
    python3 tools/size_report.py "$out/check-size-$level-auto/libmaul-anim.a" --check "$level-avx2" >/dev/null
done
configure size-clang -DCMAKE_C_COMPILER=clang-20 -DCMAKE_BUILD_TYPE=Release -DMAUL_ANIM_OFFLINE=OFF \
    -DMAUL_ANIM_BUILD_TESTS=OFF -DMAUL_ANIM_BUILD_BENCH=OFF
build size-clang
python3 tools/size_report.py "$out/check-size-clang/libmaul-anim.a" --allocators

step "all checks passed"
