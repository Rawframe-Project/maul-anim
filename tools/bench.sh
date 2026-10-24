#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Runs the benchmark program of a build and records where it ran.
#
#   tools/bench.sh <build dir> [case ...]
#
# Prints the machine (CPU, kernel, logical CPUs) and its load average
# before and after, the program's timings (nanoseconds a call, the best
# of five runs) and, where `perf` can count user instructions, each
# case's instructions a call: the count of 1,001 calls less that of one,
# over 1,000, so the scenes' build drops out. Without a case name, every
# case. Run it through the workspace's heavy-job wrapper, on a quiet
# machine when the timings are to be compared.
set -euo pipefail

usage() {
    echo "usage: tools/bench.sh <build dir> [case ...]" >&2
    exit 2
}

[ $# -ge 1 ] || usage
build=$1
shift
bench=$build/maul-anim_bench
[ -x "$bench" ] || {
    echo "no $bench: configure with MAUL_ANIM_BUILD_BENCH and MAUL_ANIM_OFFLINE on" >&2
    exit 1
}
if [ $# -eq 0 ]; then
    mapfile -t cases < <("$bench" --list)
else
    cases=("$@")
fi

load() { cut -d' ' -f1-3 /proc/loadavg 2>/dev/null || uptime; }

cpu=$(grep -m1 'model name' /proc/cpuinfo 2>/dev/null | cut -d: -f2- | sed 's/^ *//' || true)
echo "# machine: ${cpu:-unknown CPU}, $(nproc) logical CPUs, $(uname -sr)"
echo "# load before: $(load)"
"$bench" "${cases[@]}"
echo "# load after: $(load)"

count() {
    perf stat -x, -e instructions:u "$bench" --count "$1" "$2" 2>&1 >/dev/null | tail -1 | cut -d, -f1
}

if ! command -v perf >/dev/null || ! [[ $(count "${cases[0]}" 1) =~ ^[0-9]+$ ]]; then
    echo "# instructions: perf cannot count user instructions here"
    exit 0
fi
echo "# instructions a call"
for name in "${cases[@]}"; do
    one=$(count "$name" 1)
    many=$(count "$name" 1001)
    printf '%-28s %12d\n' "$name" $(((many - one) / 1000))
done
