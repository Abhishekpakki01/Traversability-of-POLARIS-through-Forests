# Editing and uploading this project

The repository checkout is `~/Traversability-of-POLARIS-through-Forests`.
The existing working ROS workspace remains separately at `~/semantic_map_ws`.
Editing one folder does not automatically edit the other. Use this checkout for
future Git-tracked changes and keep the old workspace as the working comparison.

## Edit on GitHub

Open a source or Markdown file on GitHub, choose the pencil/Edit action, make a
small change, and commit it. Source files are in `src/semantic_mapping/`; all
earlier uploaded files are preserved under `legacy/original_upload/`.

Then retrieve the change locally before building:

```bash
cd ~/Traversability-of-POLARIS-through-Forests
git status
git pull --ff-only
```

If you already have local edits, inspect and commit them before pulling; do not
discard them or force-reset the checkout.

## Edit locally, test, then upload

```bash
cd ~/Traversability-of-POLARIS-through-Forests
git status
git pull --ff-only
```

Edit the intended file in your editor. Preserve working versions before
significant changes. To test package changes using this PC's existing dependency
workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/semantic_map_ws/install/setup.bash
colcon build --packages-select semantic_mapping --symlink-install
source install/setup.bash
colcon test --packages-select semantic_mapping --return-code-on-test-failure
colcon test-result --test-result-base build/semantic_mapping/test_results --verbose
```

Use the README playback commands to inspect visual behavior. Unit tests do not
establish correct terrain labels. When satisfied, select the files you changed:

```bash
git diff
git add src/semantic_mapping
git diff --cached --stat
git commit -m "Describe the specific mapping change"
git push origin main
```

For a documentation-only change, use `git add README.md` or the exact guide path
instead. No source-code retest is needed just to correct prose. Avoid `git add .`
until you have inspected what it would include. Never force-push to replace the
existing history.

## Authentication

The GitHub CLI is installed locally at `~/.local/bin/gh`. Sign in through
GitHub's browser flow if credentials are absent:

```bash
~/.local/bin/gh auth login --hostname github.com --git-protocol https --web
```

Do not paste a password or access token into a chat or commit one into a file.
The repository-local credential helper uses the authenticated CLI. Git identity
was configured only in this checkout, matching the previous repository author;
the user's global Git configuration was not changed.

## Large files

Sensor payloads and learned weights are excluded by `.gitignore`. The demo video
and portable handoff archive are suitable release assets rather than ordinary
source changes. Uploading datasets is a separate data-distribution task; the
README/provenance records how the current files can be located or recovered.
Do not remove the ignore rules just to add multi-GB recordings to a commit.

Build products, environments and credentials also remain outside Git. To share
results, prefer the report, relevant image, saved voxel/pose arrays and provenance
needed to interpret them, with an explanation of the run's limitations.

