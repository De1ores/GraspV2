#!/usr/bin/env python3
"""Validate that an ARM64 offline bundle can be built before staging it."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import sys


EXTRA_WHEELS = {
    "vision": (
        ("ultralytics", "8.4.90"),
        ("ultralytics-thop", "2.0.18"),
    ),
}
CHECKSUMS = {
    "offline/runtime/libcusparse_lt-linux-aarch64-0.7.1.0-archive.tar.xz": (
        "d3b0a660fd552e0bd9a4491b15299d968674833483d5f164cfea35e70646136c"
    ),
    "mobileclip2_b.ts": (
        "35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f"
    ),
    "yoloe-26s-seg.pt": (
        "48f24206bc8680d60cbbfa296b0140da849669b9515058b72f5a945142df0654"
    ),
}


def _canonical_distribution(value: str) -> str:
    return re.sub(r"[-_.]+", "-", value).casefold()


def _requirements(path: Path) -> list[tuple[str, str]]:
    requirements: list[tuple[str, str]] = []
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        requirement = line.split(";", 1)[0].strip()
        match = re.fullmatch(r"([A-Za-z0-9_.-]+)==([^\s]+)", requirement)
        if match is None:
            raise ValueError(
                f"{path}:{line_number}: offline requirements must use exact == pins"
            )
        requirements.append((match.group(1), match.group(2)))
    return requirements


def _available_wheels(directory: Path) -> set[tuple[str, str]]:
    available: set[tuple[str, str]] = set()
    if not directory.is_dir():
        return available
    for wheel in directory.glob("*.whl"):
        parts = wheel.name.removesuffix(".whl").split("-")
        if len(parts) >= 2 and wheel.stat().st_size > 0:
            available.add((_canonical_distribution(parts[0]), parts[1].casefold()))
    return available


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate(root: Path) -> list[str]:
    failures: list[str] = []
    for environment in ("planning", "vision"):
        requirement_path = root / "offline" / f"requirements-{environment}.txt"
        wheel_dir = root / "offline" / "wheelhouse" / "aarch64" / environment
        if not requirement_path.is_file():
            failures.append(f"missing requirements file: {requirement_path}")
            continue
        available = _available_wheels(wheel_dir)
        required = _requirements(requirement_path) + list(
            EXTRA_WHEELS.get(environment, ())
        )
        for distribution, version in required:
            identity = (_canonical_distribution(distribution), version.casefold())
            if identity not in available:
                failures.append(
                    f"missing {environment} wheel: {distribution}=={version}"
                )

    for relative_path, expected in CHECKSUMS.items():
        path = root / relative_path
        if not path.is_file() or path.stat().st_size == 0:
            failures.append(f"missing offline asset: {relative_path}")
            continue
        actual = _sha256(path)
        if actual != expected:
            failures.append(
                f"checksum mismatch: {relative_path} (expected {expected}, got {actual})"
            )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="GraspV2 source root",
    )
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    try:
        failures = validate(root)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if failures:
        print("Offline bundle inputs are incomplete:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        print(
            "Run ./tools/fetch_offline_dependencies.sh on a networked host, "
            "then retry.",
            file=sys.stderr,
        )
        return 1
    print("Offline bundle inputs: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
