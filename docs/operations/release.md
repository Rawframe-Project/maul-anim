# Release

Releases are made by release-please from the conventional commit titles on `main`. Versions follow Semantic Versioning; before 1.0.0 a breaking change bumps the minor version, a feature the minor and a fix the patch. Tags are `vX.Y.Z`. While the major version is 0, the shared library's SOVERSION is the whole version and the CMake package accepts only its exact version, since any release may change the ABI.

## Steps

1. release-please keeps a draft pull request titled `chore: release X.Y.Z` open. It moves the version between the markers in `include/maul-anim/base.h`, where CMake reads it, updates `.release-please-manifest.json`, and adds an entry to `CHANGELOG.md` from the `feat`, `fix` and `perf` titles and the `BREAKING CHANGE:` footers.
2. Rewrite that entry in user language, in `CHANGELOG.md` on the release pull request's branch (`release-please--branches--main--components--maul-anim`): what changed for someone using the library, grouped as Added, Changed, Fixed, Removed and Security.
   - A breaking change comes first under Changed, prefixed **Breaking:**, and says what a caller must change.
   - Name every change of results (a pose's bits, a compressed clip's bytes, a file format version), since programs pin results by hash and store built files.
   - User documentation (the README and `docs/user/`) gets one short line under Changed when it changes what a user can find or do.
   - Leave out refactors, tests, CI, and the internals and operations documentation.
3. Put the same entry in the pull request's body, in place of the notes between release-please's `---` lines: release-please publishes the GitHub release notes from the body, not from `CHANGELOG.md`. Keep its header line, both `---` lines and its footer around the entry. On merge, release-please parses the body: one it cannot parse gets no tag and no release, and release-please opens a new release pull request with a wrong version. release-please rewrites both whenever anything else lands on `main`, so the curation is the last change before the merge, and nothing else merges to `main` until the release is published. Curate only after the `release` run for `main`'s latest push has finished, and do not dispatch `release.yml` afterwards: a run in flight force-updates the release branch and writes the body again over the curation.
4. Dispatch the `nightly` workflow on the release pull request's branch (`gh workflow run nightly --ref <branch>`) and check that every job passes: the ufbx corpus, the ACL compression, ozz-animation's blending, model pass and IK, Godot's retargeting, and the long fuzzing.
5. Check that the `required` CI job passed on the release pull request, including the size job (every part within its ceiling in `tools/size-budget.txt`, no allocator symbol in the runtime), and that the tag `vX.Y.Z` does not exist yet (`git ls-remote --tags origin vX.Y.Z` prints nothing).
6. Mark the pull request ready and merge it. release-please tags the merge commit and publishes the GitHub release with the body's notes. The Pages workflow publishes the documentation from `main`.
7. The `assets` job of `.github/workflows/release.yml` then attaches the source archive `maul-anim-X.Y.Z.tar.gz` (the tagged tree, generated sources and test fixtures included), its `SHA256SUMS`, an SPDX SBOM and a build provenance attestation. Check that they are on the release; a user verifies a download with `sha256sum -c SHA256SUMS` and `gh attestation verify maul-anim-X.Y.Z.tar.gz -R Rawframe-Project/maul-anim`.

## Notes

- A change of results is never a patch release. A pull request that gives existing inputs a different result, or breaks the API or ABI, carries `!` in its title and a `BREAKING CHANGE:` footer naming what a host sees, whatever its type.
- A change to a file format bumps that kind's format version in the same pull request ([file-formats.md](../internals/file-formats.md#versions)); the release notes name it, since built files must be rebuilt.
- The shared family files must match their canonical copies in the organization's `.github` repository before a release.
- A library that pins this one moves its pin in a pull request of its own.
- If a merged release pull request's body could not be parsed, put the header, `---` lines and footer back around its notes, close the release pull request release-please opened after it, and dispatch `release.yml` on `main`: release-please then tags the merge commit and publishes the release.
