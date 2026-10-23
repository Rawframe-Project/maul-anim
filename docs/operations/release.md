# Release

Releases are made by release-please from the conventional commit titles on `main`. Versions follow Semantic Versioning; before 1.0.0 a breaking change bumps the minor version and a feature or fix the patch. Tags are `vX.Y.Z`.

## Steps

1. release-please keeps a draft pull request titled `chore: release X.Y.Z` open. It moves the version between the markers in `include/maul-anim/base.h`, where CMake reads it, updates `.release-please-manifest.json`, and adds an entry to `CHANGELOG.md` from the `feat`, `fix` and `perf` titles.
2. Rewrite that entry in user language before merging: what changed for someone using the library, grouped as Added, Changed, Fixed, Removed and Security. Leave out refactors, tests, CI and documentation. Say what a caller must change for a breaking change, and name every change of results (a pose, a compressed clip's bytes, a file format version), since programs pin results by hash and store built files.
3. On the release pull request's head, dispatch the `differential` workflow and check that every job passes: the ufbx corpus, the ACL compression, ozz-animation's blending and IK, Godot's retargeting.
4. Check that the size job passed: every part within its ceiling in `tools/size-budget.txt`, no allocator symbol in the runtime.
5. Check that the `required` CI job passed, mark the pull request ready and merge it. release-please tags the merge commit and publishes the GitHub release with the changelog entry as its notes. The Pages workflow publishes the documentation from `main`.
6. The `assets` job of `.github/workflows/release.yml` attaches the source archive `maul-anim-X.Y.Z.tar.gz`, its `SHA256SUMS`, an SPDX SBOM and a build provenance attestation. A user verifies a download with `sha256sum -c SHA256SUMS` and `gh attestation verify maul-anim-X.Y.Z.tar.gz -R Rawframe-Project/maul-anim`.

## Notes

- A change to a file format bumps that kind's format version in the same pull request ([file-formats.md](../internals/file-formats.md#versions)); the release notes name it, since built files must be rebuilt.
- The shared family files must match their canonical copies in the organization's `.github` repository before a release.
- A library that pins this one moves its pin in a pull request of its own.
