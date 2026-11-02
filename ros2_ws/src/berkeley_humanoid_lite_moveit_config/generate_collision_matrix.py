#!/usr/bin/env python3
"""Compute the SRDF self-collision matrix by sampling, like the Setup Assistant.

The Setup Assistant is GUI-only on Humble, so this reproduces the part of it we
need.  Every collision body is reduced to an oriented bounding box -- primitives
exactly, meshes via the axis-aligned bounds of their vertices -- and pairs are
tested with the separating-axis theorem.  Boxing a mesh is conservative: it can
report a collision the true geometry would not have, never the reverse, so a
pair this script calls "never in collision" really never is.

Arm and finger joints are sampled uniformly inside their URDF limits.  Leg
joints are held at zero because these configurations plan for the arms only and
the legs never move, so every leg/leg pair is either always or never colliding.

A pair is disabled when it collides in every sample (the links overlap by
construction, so testing it only ever blocks planning) or in none (testing it is
wasted work).  Pairs that collide some of the time stay enabled, since those are
the checks that carry information.

Intra-hand pairs are not recomputed.  The DexHand ships an SRDF whose matrix the
Setup Assistant already produced against the real mesh geometry; --import-srdf
folds those in verbatim, which is both faster and more accurate than boxing 22
finger meshes against each other.

    python3 generate_collision_matrix.py \
        --urdf ../berkeley_humanoid_lite_description/urdf/berkeley_humanoid_lite_dexhand.urdf \
        --import-srdf ~/Desktop/dexhand_moveit_ws/src/dexhand_moveit_config/config/dexhandv2_right_8servo.srdf \
        --samples 10000

Prints an SRDF fragment on stdout.
"""

from __future__ import annotations

import argparse
import itertools
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_URDF = (
    HERE.parent
    / "berkeley_humanoid_lite_description/urdf/berkeley_humanoid_lite.urdf"
)
PACKAGE_ROOTS = {
    "berkeley_humanoid_lite_description": HERE.parent
    / "berkeley_humanoid_lite_description",
}


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    k = 1.0 - c
    return np.array([
        [c + x * x * k, x * y * k - z * s, x * z * k + y * s],
        [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
        [z * x * k - y * s, z * y * k + x * s, c + z * z * k],
    ])


def transform(xyz: np.ndarray, rot: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = xyz
    return t


def parse_origin(element: ET.Element | None) -> np.ndarray:
    if element is None:
        return np.eye(4)
    origin = element.find("origin")
    if origin is None:
        return np.eye(4)
    xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
    rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
    return transform(xyz, rpy_to_matrix(rpy))


def resolve_package_uri(uri: str) -> Path:
    if not uri.startswith("package://"):
        raise SystemExit(f"unsupported mesh URI: {uri}")
    package, _, relative = uri[len("package://"):].partition("/")
    root = PACKAGE_ROOTS.get(package)
    if root is None:
        raise SystemExit(f"no source path known for package '{package}'")
    return root / relative


def stl_bounds(path: Path, scale: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned bounds of a binary STL, in metres."""
    data = path.read_bytes()
    if len(data) < 84:
        raise SystemExit(f"{path}: too short to be an STL")
    if data[:5] == b"solid" and b"facet" in data[:512]:
        raise SystemExit(f"{path}: ASCII STL is not supported")

    count = struct.unpack("<I", data[80:84])[0]
    record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    expected = 84 + count * record.itemsize
    if len(data) < expected:
        raise SystemExit(f"{path}: truncated (want {expected} bytes, have {len(data)})")

    triangles = np.frombuffer(data[84:expected], dtype=record, count=count)
    vertices = triangles["v"].reshape(-1, 3).astype(np.float64) * scale
    return vertices.min(axis=0), vertices.max(axis=0)


class Model:
    """Minimal URDF kinematics: joint tree, FK, and collision bounding boxes."""

    def __init__(self, path: Path) -> None:
        root = ET.parse(path).getroot()

        self.joints: dict[str, dict] = {}
        self.parent_of: dict[str, tuple[str, str]] = {}
        for joint in root.findall("joint"):
            name = joint.get("name")
            limit = joint.find("limit")
            lower = upper = 0.0
            if limit is not None:
                lower = float(limit.get("lower", 0.0))
                upper = float(limit.get("upper", 0.0))
            axis_el = joint.find("axis")
            axis = (
                np.fromstring(axis_el.get("xyz"), sep=" ")
                if axis_el is not None
                else np.array([0.0, 0.0, 1.0])
            )
            child = joint.find("child").get("link")
            self.joints[name] = {
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": child,
                "origin": parse_origin(joint),
                "axis": axis,
                "lower": lower,
                "upper": upper,
            }
            self.parent_of[child] = (name, self.joints[name]["parent"])

        children = {j["child"] for j in self.joints.values()}
        roots = [l.get("name") for l in root.findall("link")
                 if l.get("name") not in children]
        if len(roots) != 1:
            raise SystemExit(f"expected exactly one root link, found {roots}")
        self.root = roots[0]

        self.bodies: list[tuple[str, np.ndarray, np.ndarray]] = []
        for link in root.findall("link"):
            link_name = link.get("name")
            for collision in link.findall("collision"):
                local = parse_origin(collision)
                geometry = collision.find("geometry")
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                mesh = geometry.find("mesh")
                if box is not None:
                    half = np.fromstring(box.get("size"), sep=" ") / 2.0
                    centre = np.zeros(3)
                elif cylinder is not None:
                    radius = float(cylinder.get("radius"))
                    length = float(cylinder.get("length"))
                    # URDF cylinders are z-aligned; circumscribe with a box.
                    half = np.array([radius, radius, length / 2.0])
                    centre = np.zeros(3)
                elif mesh is not None:
                    scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
                    lower, upper = stl_bounds(
                        resolve_package_uri(mesh.get("filename")), scale
                    )
                    half = (upper - lower) / 2.0
                    centre = (upper + lower) / 2.0
                else:
                    tags = [child.tag for child in geometry]
                    raise SystemExit(
                        f"{link_name}: unsupported collision geometry {tags}"
                    )
                # Fold the bounds centre into the transform so `half` is centred.
                local = local @ transform(centre, np.eye(3))
                self.bodies.append((link_name, local, half))

        self.movable = [n for n, j in self.joints.items()
                        if j["type"] in ("revolute", "continuous", "prismatic")]

    def link_poses(self, q: dict[str, float]) -> dict[str, np.ndarray]:
        poses: dict[str, np.ndarray] = {self.root: np.eye(4)}

        def pose_of(link: str) -> np.ndarray:
            if link in poses:
                return poses[link]
            joint_name, parent = self.parent_of[link]
            joint = self.joints[joint_name]
            local = joint["origin"]
            if joint["type"] in ("revolute", "continuous"):
                local = local @ transform(
                    np.zeros(3), axis_angle_to_matrix(joint["axis"], q.get(joint_name, 0.0))
                )
            elif joint["type"] == "prismatic":
                local = local @ transform(joint["axis"] * q.get(joint_name, 0.0), np.eye(3))
            poses[link] = pose_of(parent) @ local
            return poses[link]

        for link in self.parent_of:
            pose_of(link)
        return poses


def obb_overlap(
    c1: np.ndarray, r1: np.ndarray, h1: np.ndarray,
    c2: np.ndarray, r2: np.ndarray, h2: np.ndarray,
    epsilon: float = 1e-9,
) -> bool:
    """Separating-axis test for two oriented boxes (Gottschalk)."""
    rot = r1.T @ r2
    abs_rot = np.abs(rot) + epsilon
    t = r1.T @ (c2 - c1)

    for i in range(3):
        if abs(t[i]) > h1[i] + h2 @ abs_rot[i, :]:
            return False
    for i in range(3):
        if abs(t @ rot[:, i]) > h1 @ abs_rot[:, i] + h2[i]:
            return False
    for i in range(3):
        for j in range(3):
            i1, i2 = (i + 1) % 3, (i + 2) % 3
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            ra = h1[i1] * abs_rot[i2, j] + h1[i2] * abs_rot[i1, j]
            rb = h2[j1] * abs_rot[i, j2] + h2[j2] * abs_rot[i, j1]
            if abs(t[i2] * rot[i1, j] - t[i1] * rot[i2, j]) > ra + rb:
                return False
    return True


def mirror_link(name: str) -> str:
    """Left-side counterpart of a DexHand link name, matching generate_urdf.py."""
    return "L_" + name[2:] if name.startswith("R_") else "L_" + name


def import_srdf_pairs(path: Path, mirror: bool,
                      present: set[str] | None = None) -> tuple[list[str], set[str]]:
    """Reuse an existing SRDF's disable_collisions entries.

    With `mirror`, each pair is emitted twice: once verbatim for the right hand
    and once with left-side names. The mirrored hand is geometrically identical,
    so a pair that can never collide on one side can never collide on the other.

    `present` is the model's own link set. Pairs naming anything outside it are
    dropped: the imported matrix describes the whole DexHand V2, palm included,
    and this build's palm is V1's, so those pairs are about a part that is not
    here. Keeping them produces an SRDF MoveIt refuses to load, naming a link
    nothing has heard of. Dropping them leaves those pairs to be computed
    against the palm that is actually fitted.
    """
    root = ET.parse(path).getroot()
    lines, links, dropped = [], set(), 0
    for entry in root.findall("disable_collisions"):
        a, b = entry.get("link1"), entry.get("link2")
        reason = entry.get("reason", "Imported")
        for left, right in ([(a, b)] + ([(mirror_link(a), mirror_link(b))] if mirror else [])):
            if present is not None and not {left, right} <= present:
                dropped += 1
                continue
            links.update((left, right))
            lines.append(
                f'    <disable_collisions link1="{left}" link2="{right}" '
                f'reason="{reason}"/>'
            )
    if dropped:
        print(f"<!-- dropped {dropped} imported pairs naming links this model "
              f"does not have -->")
    return lines, links


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--import-srdf", type=Path, default=None,
                        help="SRDF whose disable_collisions are reused verbatim")
    parser.add_argument("--mirror-import", action="store_true",
                        help="also emit an L_-renamed copy of the imported pairs, "
                             "for a mirrored second hand")
    parser.add_argument("--samples", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    model = Model(args.urdf.expanduser())

    imported_lines: list[str] = []
    imported_links: set[str] = set()
    if args.import_srdf:
        imported_lines, imported_links = import_srdf_pairs(
            args.import_srdf.expanduser(), args.mirror_import,
            present={link.get("name") for link in
                     ET.parse(args.urdf).getroot().iter("link")},
        )
        print(f"<!-- imported {len(imported_lines)} pairs over "
              f"{len(imported_links)} links from {args.import_srdf.name} -->")

    sampled = [n for n in model.movable
               if n.startswith("arm_") or n.startswith("R_") or n.startswith("L_")]
    print(f"<!-- collision bodies: {len(model.bodies)}, "
          f"sampled joints: {len(sampled)}, samples: {args.samples} -->")

    # Skip pairs already covered by the imported matrix (both links inside it).
    pairs = [
        (a, b) for a, b in itertools.combinations(range(len(model.bodies)), 2)
        if model.bodies[a][0] != model.bodies[b][0]
        and not (model.bodies[a][0] in imported_links
                 and model.bodies[b][0] in imported_links)
    ]
    print(f"<!-- pairs to sample: {len(pairs)} -->")

    # Broad phase: a body can never reach further than this from its centre.
    radii = np.array([np.linalg.norm(half) for _, _, half in model.bodies])

    rng = np.random.default_rng(args.seed)
    hits = np.zeros(len(pairs), dtype=np.int64)

    for _ in range(args.samples):
        q = {name: rng.uniform(model.joints[name]["lower"],
                               model.joints[name]["upper"])
             for name in sampled}
        poses = model.link_poses(q)
        world = []
        for link, local, half in model.bodies:
            pose = poses.get(link, np.eye(4)) @ local
            world.append((pose[:3, 3], pose[:3, :3], half))
        centres = np.array([w[0] for w in world])

        for index, (a, b) in enumerate(pairs):
            if np.linalg.norm(centres[a] - centres[b]) > radii[a] + radii[b]:
                continue  # cheap reject before the 15-axis test
            if obb_overlap(*world[a], *world[b]):
                hits[index] += 1

    lines = list(imported_lines)
    computed = 0
    for index, (a, b) in enumerate(pairs):
        link_a, link_b = model.bodies[a][0], model.bodies[b][0]
        rate = hits[index] / args.samples
        if rate == 0.0:
            reason = "Never in collision"
        elif rate == 1.0:
            reason = "Always in collision"
        else:
            print(f"<!-- kept enabled: {link_a} / {link_b} "
                  f"collides in {rate:.2%} of samples -->")
            continue
        computed += 1
        lines.append(
            f'    <disable_collisions link1="{link_a}" link2="{link_b}" '
            f'reason="{reason}"/>'
        )

    print(f"<!-- disabled: {computed} computed + {len(imported_lines)} imported "
          f"= {len(lines)} -->")
    print("\n".join(sorted(lines)))


if __name__ == "__main__":
    main()
