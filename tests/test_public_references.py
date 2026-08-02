"""Every path the repository mentions must exist, or be produced by a command.

A release that references a file it does not ship is worse than one that omits
it: the reader follows the instruction, it fails, and there is no way to tell
whether the file was forgotten or the instruction was wrong. This scans the
whole tree and fails on the first dead reference.

It also refuses private paths, host names and the filenames of tools that were
deliberately left out of the extraction.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

TEXT_SUFFIXES = {".md", ".py", ".yaml", ".yml", ".sdf", ".cff", ".toml",
                 ".rviz", ".example", ".txt", ".sh"}
SKIP_DIRS = {".git", "__pycache__", ".venv", ".pytest_cache", "figures"}

# Things that must never appear in a public file.
FORBIDDEN = [
    (r"/home/[a-z]", "an absolute home path"),
    (r"\bdragonlfy\b", "a developer user name"),
    (r"\bDragonflyJr\b", "a developer host name"),
    (r"\bthesis_config\b", "the private repository name"),
    (r"scripts/pane_\w+\.sh", "a private launcher script"),
    (r"start_x500_stack_tmux", "a private launcher script"),
    (r"\bartifacts/[a-z0-9_]+", "a private artifact directory"),
    (r"plan_a_star_(policy|clearance)\.py", "a tool that was not extracted"),
    (r"refine_raw_astar_bspline\.py", "a tool that was not extracted"),
    (r"smooth_clearance_preserving\.py", "a tool that was not extracted"),
    (r"build_mission_reference\.py", "a tool that was not extracted"),
    (r"gate\d[a-d]?_[a-z_]+\.py", "a gate-named private tool"),
]

# Referenced paths that are produced by a documented command rather than
# shipped. Keyed by the exact string as written.
GENERATED = {
    "prior/map.yaml", "patched/map.yaml", "setpoints_ned.csv",
    "obstacle.yaml", "mission_reference.csv", "spawn_record.yaml",
    "map.pgm", "map.yaml", "hashes.txt", "manifest.yaml",
}


def public_files():
    for path in REPO.rglob("*"):
        if not path.is_file():
            continue
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        if path.suffix in TEXT_SUFFIXES:
            yield path


def test_no_private_paths_or_omitted_tools():
    """The scan exempts this file, which has to name the patterns it forbids."""
    problems = []
    for path in public_files():
        if path.name == Path(__file__).name:
            continue
        text = path.read_text(errors="replace")
        for pattern, why in FORBIDDEN:
            for m in re.finditer(pattern, text):
                line = text[:m.start()].count("\n") + 1
                problems.append(
                    f"{path.relative_to(REPO)}:{line}: {m.group(0)!r} is {why}")
    assert problems == [], "\n" + "\n".join(problems)


def test_referenced_repository_paths_exist():
    """Any repo-relative path written in a public file must resolve.

    Only paths that look like real repository locations are checked: a leading
    known top-level directory. That keeps the scan free of false positives on
    prose while still catching the references that matter.
    """
    tops = ("src/", "tools/", "tests/", "docs/", "sim/", "results/", "ros2/",
            ".github/")
    pattern = re.compile(
        r"(?<![\w/])((?:" + "|".join(re.escape(t) for t in tops) +
        r")[\w./-]*[\w/])")
    problems = []
    for path in public_files():
        text = path.read_text(errors="replace")
        for m in pattern.finditer(text):
            ref = m.group(1).rstrip("./")
            if ref.endswith("/") or "*" in ref:
                continue
            target = REPO / ref
            if target.exists():
                continue
            if Path(ref).name in GENERATED:
                continue
            line = text[:m.start()].count("\n") + 1
            problems.append(f"{path.relative_to(REPO)}:{line}: {ref}")
    assert problems == [], (
        "referenced paths that do not exist:\n" + "\n".join(sorted(set(problems))))


def test_markdown_links_resolve():
    link = re.compile(r"\[[^\]]*\]\(([^)#]+?)(?:#[^)]*)?\)")
    problems = []
    for path in REPO.rglob("*.md"):
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        for m in link.finditer(path.read_text(errors="replace")):
            target = m.group(1).strip()
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            resolved = (path.parent / target).resolve()
            if not resolved.exists():
                problems.append(f"{path.relative_to(REPO)} -> {target}")
    assert problems == [], "broken links:\n" + "\n".join(problems)


def test_no_empty_directories():
    empties = [str(d.relative_to(REPO)) for d in REPO.rglob("*")
               if d.is_dir()
               and not any(part in SKIP_DIRS for part in d.parts)
               and not any(d.iterdir())]
    assert empties == [], f"empty directories: {empties}"


def test_every_result_bundle_has_verifiable_hashes():
    import hashlib
    for bundle in sorted((REPO / "results").iterdir()):
        if not bundle.is_dir():
            continue
        hashes = bundle / "hashes.txt"
        if not hashes.is_file():
            pytest.fail(f"{bundle.name} has no hashes.txt")
        for line in hashes.read_text().splitlines():
            if not line.strip():
                continue
            digest, name = line.split(None, 1)
            target = bundle / name.strip()
            assert target.is_file(), f"{bundle.name}: missing {name}"
            actual = hashlib.sha256(target.read_bytes()).hexdigest()
            assert actual == digest, f"{bundle.name}: {name} changed"


def test_shell_scripts_are_syntactically_valid():
    import shutil
    import subprocess
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash not available")
    for script in REPO.rglob("*.sh"):
        if any(part in SKIP_DIRS for part in script.parts):
            continue
        proc = subprocess.run([bash, "-n", str(script)],
                              capture_output=True, text=True)
        assert proc.returncode == 0, f"{script.name}: {proc.stderr}"
