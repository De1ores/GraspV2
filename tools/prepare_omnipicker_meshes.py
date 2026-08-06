#!/usr/bin/env python3
"""Create MuJoCo-compatible caches for oversized binary OmniPicker STL files."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct

import numpy as np


MAX_MUJOCO_FACES = 200_000
TARGET_FACES = 180_000


def _read_binary_stl(path: Path) -> tuple[bytes, np.ndarray]:
    with path.open("rb") as stream:
        header = stream.read(80)
        count_data = stream.read(4)
        if len(header) != 80 or len(count_data) != 4:
            raise ValueError(f"invalid binary STL header: {path}")
        count = struct.unpack("<I", count_data)[0]
        records = np.fromfile(
            stream,
            dtype=np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("vertices", "<f4", (3, 3)),
                    ("attribute", "<u2"),
                ]
            ),
            count=count,
        )
    if len(records) != count or path.stat().st_size != 84 + count * 50:
        raise ValueError(f"not a supported binary STL: {path}")
    return header, records["vertices"].astype(np.float64)


def _cluster_faces(triangles: np.ndarray, tolerance: float) -> np.ndarray:
    flat = triangles.reshape(-1, 3)
    origin = flat.min(axis=0)
    keys = np.rint((flat - origin) / tolerance).astype(np.int64)
    unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    valid = (
        (faces[:, 0] != faces[:, 1])
        & (faces[:, 1] != faces[:, 2])
        & (faces[:, 0] != faces[:, 2])
    )
    faces = faces[valid]
    canonical = np.sort(faces, axis=1)
    _, unique_indices = np.unique(canonical, axis=0, return_index=True)
    faces = faces[np.sort(unique_indices)]
    vertices = origin + unique_keys.astype(np.float64) * tolerance
    return vertices[faces]


def _decimate(triangles: np.ndarray) -> tuple[np.ndarray, float]:
    extent = np.ptp(triangles.reshape(-1, 3), axis=0)
    diagonal = float(np.linalg.norm(extent))
    if diagonal <= 0.0:
        raise ValueError("mesh has zero extent")
    tolerance = diagonal * 1e-6
    result = triangles
    for _ in range(32):
        result = _cluster_faces(triangles, tolerance)
        if 0 < len(result) <= TARGET_FACES:
            return result, tolerance
        tolerance *= 1.45
    raise ValueError(
        f"could not reduce mesh below {TARGET_FACES} faces; last count={len(result)}"
    )


def _write_binary_stl(path: Path, header: bytes, triangles: np.ndarray) -> None:
    first = triangles[:, 1] - triangles[:, 0]
    second = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(first, second)
    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 1e-12
    normals[nonzero] /= lengths[nonzero, None]
    normals[~nonzero] = 0.0
    records = np.zeros(
        len(triangles),
        dtype=np.dtype(
            [
                ("normal", "<f4", (3,)),
                ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ]
        ),
    )
    records["normal"] = normals.astype(np.float32)
    records["vertices"] = triangles.astype(np.float32)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(header[:80].ljust(80, b" "))
        stream.write(struct.pack("<I", len(records)))
        records.tofile(stream)


def prepare(mesh_directory: Path) -> None:
    output_directory = mesh_directory / "mujoco"
    for source in sorted(mesh_directory.glob("*.STL")):
        header, triangles = _read_binary_stl(source)
        if len(triangles) <= MAX_MUJOCO_FACES:
            print(f"OK   {source.name}: {len(triangles)} faces")
            continue
        reduced, tolerance = _decimate(triangles)
        output = output_directory / source.name
        _write_binary_stl(output, header, reduced)
        print(
            f"CACHE {source.name}: {len(triangles)} -> {len(reduced)} faces, "
            f"tolerance={tolerance:.8g}, output={output}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    default_meshes = Path(__file__).resolve().parents[1] / "robot_description" / "meshes"
    parser.add_argument("--meshes", type=Path, default=default_meshes)
    args = parser.parse_args()
    prepare(args.meshes.expanduser().resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
