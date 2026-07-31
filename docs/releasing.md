# Publishing `a1z` to PyPI

The CI workflow builds a wheel and source distribution for every pull request
and every push to `main`. A tag that starts with `v` also publishes those
artifacts to PyPI through Trusted Publishing.

## One-time setup

The `a1z` name had no project page or simple-index entry on PyPI when checked
on July 30, 2026. PyPI does not reserve a name until the first release, so
complete these steps before announcing the package name:

1. Sign in to the PyPI account that will own the project and open
   **Account settings > Publishing**.
2. Add a pending GitHub Actions publisher with these exact values:

   | Field | Value |
   | --- | --- |
   | PyPI project name | `a1z` |
   | Owner | `userguide-galaxea` |
   | Repository | `GALAXEA-A1Z` |
   | Workflow name | `ci.yml` |
   | Environment name | `pypi` |

3. In the GitHub repository, open **Settings > Environments** and create an
   environment named `pypi`.
4. Add required reviewers or other deployment protection rules to the `pypi`
   environment if releases need manual approval.

The pending publisher creates the PyPI project on its first successful
release. It does not reserve `a1z` before then. See PyPI's
[pending publisher instructions][pending-publisher] for the security and
name-reservation details.

## Release

1. Merge the release changes into `main`.
2. Choose a PEP 440 version. The tag supplies the package version, so a
   `v0.1.0` tag builds version `0.1.0`.
3. Create and push an annotated tag:

   ```bash
   git switch main
   git pull --ff-only
   git tag -a v0.1.0 -m "v0.1.0"
   git push origin v0.1.0
   ```

4. Approve the `pypi` deployment if the environment requires approval.
5. Confirm the release at <https://pypi.org/project/a1z/> and install it in a
   clean environment:

   ```bash
   python -m venv /tmp/a1z-release-check
   /tmp/a1z-release-check/bin/python -m pip install a1z
   /tmp/a1z-release-check/bin/python -c "import a1z; print(a1z.__file__)"
   ```

PyPI releases are immutable. If a release is wrong, fix the problem and
publish a new version; do not reuse the tag or version.

[pending-publisher]: https://docs.pypi.org/trusted-publishers/creating-a-project-through-oidc/
