"""Artifact writers: deterministic CSV, YAML and SHA-256 manifests.

Every run of a pipeline stage writes a self-describing bundle:

``*.csv`` / ``*.png``
    The deterministic content. Identical inputs produce identical bytes.
``metrics.yaml``
    Quantitative results.
``manifest.yaml``
    Inputs, parameters and tool versions needed to regenerate the bundle.
``hashes.txt``
    ``sha256sum``-compatible digests of the deterministic content files only.

``manifest.yaml`` carries a timestamp and is therefore deliberately *excluded*
from ``hashes.txt``, as is any metrics file containing wall-clock timings.
Otherwise ``sha256sum -c hashes.txt`` would fail on a correct regeneration and
the check would be worthless.

Numeric CSV output is written with a fixed decimal count rather than ``repr``
so the text is stable across platforms.
"""

from __future__ import annotations

import csv
import hashlib
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import yaml

__all__ = [
    "sha256_of_file", "write_csv", "write_yaml", "write_hashes",
    "verify_hashes", "format_float",
]


def sha256_of_file(path: Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def format_float(value: float, decimals: int = 6) -> str:
    """Fixed-decimal float formatting for stable CSV text."""
    return f"{float(value):.{decimals}f}"


def write_csv(path: Path, header: Sequence[str], rows: Iterable[Sequence]) -> None:
    """Write a CSV with LF line endings regardless of platform."""
    with Path(path).open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(list(header))
        for row in rows:
            writer.writerow(list(row))


def write_yaml(path: Path, payload: Dict) -> None:
    """Write YAML preserving insertion order and using block style."""
    with Path(path).open("w", encoding="utf-8") as f:
        yaml.safe_dump(
            payload, f, sort_keys=False, default_flow_style=False, allow_unicode=True,
        )


def write_hashes(out_path: Path, content_files: List[Path]) -> None:
    """Write a ``sha256sum``-compatible file over deterministic content only.

    Verify with ``cd <dir> && sha256sum -c hashes.txt``.
    """
    lines: List[str] = []
    for p in content_files:
        lines.append(f"{sha256_of_file(p)}  {Path(p).name}")
    Path(out_path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def verify_hashes(hashes_path: Path) -> List[str]:
    """Re-check a ``hashes.txt``. Returns a list of failures; empty means OK."""
    hashes_path = Path(hashes_path)
    base = hashes_path.parent
    failures: List[str] = []
    for line in hashes_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        digest, _, name = line.partition("  ")
        target = base / name
        if not target.exists():
            failures.append(f"{name}: MISSING")
            continue
        actual = sha256_of_file(target)
        if actual != digest:
            failures.append(f"{name}: expected {digest[:16]}… got {actual[:16]}…")
    return failures
