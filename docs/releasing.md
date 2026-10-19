# Releasing Maul Anim

A release follows the family's release checklist (`docs/conventions.md`,
section 15) in order. Maul Anim's own steps below are taken between its
steps 4 and 5: on the `release: X.Y.Z` commit, once CI is green on it,
and before the tag.

## Maul Anim's own steps

1. **The size budget** (record mani-0036): on the release commit, a
   runtime-only Release build (`-DMAUL_ANIM_OFFLINE=OFF`) at `-O2` and
   at `-O3` with GCC 14 passes `tools/size_report.py <archive> --check
   O2` and `--check O3`: every part within its ceiling in
   `tools/size-budget.txt`, no module unplaced, no allocator symbol
   (CI's size and Windows jobs check the Clang 20 and clang-cl
   archives too).
   The two tables go into the release's notes. A part over its ceiling
   blocks the release until it shrinks or a decision in mani-0036
   raises the ceiling.
2. **The differential jobs**: the `differential` workflow, dispatched
   on the release commit, is green in every job (the ufbx corpus, the
   ACL compression, ozz's blending, ozz's IK and Godot's retargeting).
   Its sizes, errors and times go into the notes.
3. **The targets** (record mani-0036): the notes carry the table of
   the library's targets against ACL and ozz-animation, each met or
   not with its measured ratio, from the differential jobs and the
   benchmarks. Before 1.0.0 a target not met is listed, never left out;
   1.0.0 waits on all of them.
4. **Fuzzing**: each fuzz target (`fuzz_*`, built with
   `-DMAUL_ANIM_FUZZ=ON`) runs for at least ten minutes from its seed
   corpus (the CI fuzz job's seeds) without a finding.
5. **The guide and the API reference** are current:
   `tools/check_docs.py` and `tools/check_guide.py` pass, the guide's
   tests (`test_guide`, `test_guide_import`) pass, and `docs/api.md` is
   regenerated.
