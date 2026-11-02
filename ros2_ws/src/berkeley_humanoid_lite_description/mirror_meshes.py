#!/usr/bin/env python3
"""Mirror the DexHand's right-hand STLs into a left-hand set.

dexhandv2_description ships `right` and `cobot_right` meshes and no left variant,
so a left hand has to be produced here.  The mirror plane is Y: the fingers
extend along +Z and spread along Y, with the palm normal along X, so negating Y
turns a right hand into a left one.

Mirroring reverses handedness, which means the triangle winding has to be
reversed too or every face ends up inside-out -- renderers and collision
checkers both care.  Normals are recomputed from the rewound vertices rather
than mirrored, so they cannot disagree with the winding.

Writing real files, not `<mesh scale="-1 1 1">`: negative scale in URDF is
handled inconsistently across RViz, FCL and the various mesh loaders, and a
silently inside-out collision mesh is a bad thing to debug.

    python3 mirror_meshes.py
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "meshes_dexhand" / "right"
OUTPUT = HERE / "meshes_dexhand_left"

RECORD = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
MIRROR = np.array([1.0, -1.0, 1.0])


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) < 84:
        raise SystemExit(f"{path}: too short to be an STL")
    if data[:5].lower() == b"solid" and b"facet" in data[:2048].lower():
        raise SystemExit(f"{path}: ASCII STL is not supported")
    count = struct.unpack("<I", data[80:84])[0]
    expected = 84 + count * RECORD.itemsize
    if len(data) < expected:
        raise SystemExit(f"{path}: truncated (want {expected}, have {len(data)})")
    return np.frombuffer(data[84:expected], dtype=RECORD, count=count)["v"].astype(
        np.float64
    )


def write_binary_stl(path: Path, triangles: np.ndarray, header: str) -> None:
    count = len(triangles)
    records = np.zeros(count, dtype=RECORD)
    records["v"] = triangles

    edge1 = triangles[:, 1] - triangles[:, 0]
    edge2 = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edge1, edge2)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    # Degenerate triangles keep a zero normal; STL viewers fall back to winding.
    records["normal"] = np.divide(
        normals, lengths, out=np.zeros_like(normals), where=lengths > 0
    )

    with path.open("wb") as handle:
        handle.write(header.encode("ascii", "replace")[:80].ljust(80, b"\0"))
        handle.write(struct.pack("<I", count))
        handle.write(records.tobytes())


def signed_volume(triangles: np.ndarray) -> float:
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    return np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0


def main() -> None:
    if not SOURCE.is_dir():
        raise SystemExit(f"source meshes not found: {SOURCE}")
    OUTPUT.mkdir(parents=True, exist_ok=True)

    sources = sorted(SOURCE.glob("*.stl")) + sorted(SOURCE.glob("*.STL"))
    if not sources:
        raise SystemExit(f"no STLs under {SOURCE}")

    for path in sources:
        triangles = read_binary_stl(path)
        before = signed_volume(triangles)

        mirrored = triangles * MIRROR
        # Reverse winding so the mirrored faces still point outward.
        mirrored = mirrored[:, ::-1, :]
        after = signed_volume(mirrored)

        # A correct mirror preserves both the magnitude and the sign of the
        # signed volume: negating one axis flips it, reversing winding flips it
        # back. A sign flip here means the rewind did not happen.
        if not np.isclose(before, after, rtol=1e-6, atol=1e-9):
            raise SystemExit(
                f"{path.name}: signed volume {before:.6g} -> {after:.6g}; "
                "mirror did not preserve orientation"
            )

        target = OUTPUT / path.name
        write_binary_stl(target, mirrored, f"mirrored from {path.name}")
        # STL units here are millimetres (the URDF scales by 0.001).
        print(f"  {path.name:44s} {len(triangles):7d} tris  "
              f"volume {abs(before) / 1e3:9.2f} cm3")

    print(f"\nwrote {len(sources)} mirrored meshes to {OUTPUT.relative_to(HERE)}")


if __name__ == "__main__":
    main()
