#!/usr/bin/env python3
"""Build the ROS-usable URDFs from the upstream asset URDF and the DexHand.

Four models are written to urdf/:

  berkeley_humanoid_lite.urdf          arms only, stock end effectors
  berkeley_humanoid_lite_dexhand.urdf  DexHand v2 welded to the right forearm
  berkeley_humanoid_lite_v1arm.urdf    both arms: V1 forearm + 3-DOF V1 wrist
                                       + DexHand v2 fingers (left hand mirrored)
  berkeley_humanoid_lite_tuning.urdf   v1arm with the right mount as 6 sliders

The upstream humanoid file is emitted by onshape-to-robot and is not directly
usable: mesh paths are written `package://../meshes/foo.stl`, which the ROS
resource retriever cannot resolve, and the root link `base` is free-floating
where MoveIt needs a fixed root.  Both are fixed here.

The V1 forearm and wrist come from the vendored DexHand V1 description with
their real joint origins, axes and travel limits; the one transform still
unmeasured is where the forearm bolts onto elbow_roll.  See
config/arm_attachment.yaml.

DexHand names are left untouched on the right arm so the existing
dexhand_bringup driver keeps matching them.  The left arm is a mirrored copy
with `R_` swapped for `L_` (and `L_` prefixed where there was no side marker),
since two hands cannot share link names.

    python3 generate_urdf.py
"""

from __future__ import annotations

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

PACKAGE = "berkeley_humanoid_lite_description"

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]

UPSTREAM = (
    WORKSPACE
    / "source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid"
    / "berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf"
)
# Vendored, not committed: see fetch_vendor.sh and the licence note there.
VENDOR = HERE.parent.parent / "vendor"
DEXHAND = VENDOR / "dexhandv2_description/urdf/dexhandv2_right.urdf"
# The 8-servo build used to come from a URDF on one developer's laptop, so a
# regeneration anywhere else silently produced the full 16-joint hand instead.
# The variant is described in arm_attachment.yaml now; see apply_hand_variant.
ATTACHMENT_CONFIG = HERE / "config" / "arm_attachment.yaml"
# Vendored DexHand V1 description: the real source of the wrist kinematics.
DEXHAND_V1 = {
    "right": HERE.parent.parent / "vendor/dexhand_v1_description/urdf/dexhand-right.urdf",
    "left": HERE.parent.parent / "vendor/dexhand_v1_description/urdf/dexhand-left.urdf",
}
# Upstream V1 inertials are Fusion defaults at steel density.
V1_SOURCE_DENSITY = 7850.0  # [kg/m3]
# V1 link names per side: forearm, then the two wrist halves.
V1_LINKS = {
    "right": ["forearm_1", "wrist_lower_1", "wrist_upper_1"],
    "left": ["forearm_left_1", "wrist_lower_1", "wrist_upper_1"],
}
# The V1 joint whose child is the palm. Upstream, the third wrist joint drives
# the palm directly rather than through an intermediate frame.
V1_PALM_DRIVER = "wrist_pitch_upper"
# The V1 joint that carries each digit, and so where a V2 digit bolts on. Named
# identically on both sides even though the links they connect are not.
V1_DIGIT_MOUNTS = {
    "Index": "index_yaw", "Middle": "middle_yaw",
    "Ring": "ring_yaw", "Pinky": "pinky_yaw", "Thumb": "thumb_yaw",
}
OUTPUT_DIR = HERE / "urdf"

# Every leg link, and by extension every joint that drives one.
LEG_PREFIX = "leg_"

STOCK_HAND = {
    "right": ("arm_hand_r", "arm_right_hand_link"),
    "left": ("arm_left_hand_l", "arm_left_hand_link"),
}
ELBOW = {"right": "arm_right_elbow_roll", "left": "arm_left_elbow_roll"}
DEXHAND_ROOT = "base_link"

TUNING_TRANSLATION = 0.25  # [m]
TUNING_ROTATION = 3.14159265  # [rad]


def indent(element: ET.Element, level: int = 0) -> None:
    """Pretty-print in place; ElementTree.indent is 3.9+, so keep it explicit."""
    pad = "\n" + "  " * level
    if len(element):
        if not (element.text or "").strip():
            element.text = pad + "  "
        for child in element:
            indent(child, level + 1)
        if not (child.tail or "").strip():
            child.tail = pad
    if level and not (element.tail or "").strip():
        element.tail = pad


def as_text(values) -> str:
    # 12 significant digits so serialisation contributes nothing measurable.
    # Note the model is still not perfectly left/right symmetric: the upstream
    # humanoid's own arms differ by ~4e-7 m at elbow_roll before anything is
    # attached, and that orientation error grows to ~1.6e-6 m by the fingertips.
    # It is inherited from the source CAD, not introduced by the mirror.
    return " ".join(f"{float(v):.12g}" for v in values)


def side_name(name: str, side: str) -> str:
    """Rename a DexHand link/joint for one side.

    Right keeps upstream names verbatim so the existing hardware driver still
    matches.  Left swaps a leading `R_` for `L_`, or prefixes `L_` when the name
    carries no side marker (`base_link`, `Index_Knuckle_1`, ...).
    """
    if side == "right":
        return name
    return "L_" + name[2:] if name.startswith("R_") else "L_" + name


def rotation_from_rpy(rpy) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw to a rotation matrix."""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch),
                              np.sin(pitch), np.cos(yaw), np.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rpy_from_rotation(R: np.ndarray) -> list[float]:
    """Inverse of `rotation_from_rpy`, picking the branch with pitch in +/-90."""
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if np.isclose(abs(R[2, 0]), 1.0):        # gimbal lock: fold roll into yaw
        return [0.0, float(pitch), float(np.arctan2(-R[0, 1], R[1, 1]))]
    return [float(np.arctan2(R[2, 1], R[2, 2])), float(pitch),
            float(np.arctan2(R[1, 0], R[0, 0]))]


def rotation_about(axis, angle: float) -> np.ndarray:
    """Rodrigues' formula, for baking a joint angle into a fixed transform."""
    a = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = a / norm
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def mirror_vector(xyz) -> list[float]:
    """Mirror a position across the xz plane."""
    x, y, z = (float(v) for v in xyz)
    return [x, -y, z]


def mirror_axis(xyz) -> list[float]:
    """Mirror a rotation axis across the xz plane.

    A rotation axis is a pseudovector: under a reflection M it maps to
    det(M) * M * a.  With M = diag(1, -1, 1) that is (-ax, ay, -az).  Getting
    this right is what makes a positive joint angle curl a left finger the same
    way it curls the right one, so the imported grip presets carry over.
    """
    x, y, z = (float(v) for v in xyz)
    return [-x, y, -z]


def mirror_rpy(rpy) -> list[float]:
    """Mirror an orientation across the xz plane: roll and yaw flip sign."""
    roll, pitch, yaw = (float(v) for v in rpy)
    return [-roll, pitch, -yaw]


def load_config() -> dict:
    config = yaml.safe_load(ATTACHMENT_CONFIG.read_text())
    mount = config["mount"]
    if mount.get("left") == "mirror":
        right = mount["right"]
        mount["left"] = {
            "xyz": mirror_vector(right["xyz"]),
            "rpy": mirror_rpy(right["rpy"]),
        }
    return config


def load_humanoid(config: dict | None = None) -> ET.Element:
    """Upstream humanoid with mesh paths fixed and a fixed `world` root."""
    text = UPSTREAM.read_text()
    fixed = text.replace(
        'filename="package://../meshes/',
        f'filename="package://{PACKAGE}/meshes/',
    )
    if fixed == text:
        raise SystemExit("no mesh paths rewritten; upstream format changed?")

    root = ET.fromstring(fixed)
    if root.find("link[@name='world']") is not None:
        raise SystemExit("upstream already defines a world link; review by hand")

    world = ET.Element("link", {"name": "world"})
    weld = ET.Element("joint", {"name": "world_to_base", "type": "fixed"})
    ET.SubElement(weld, "parent", {"link": "world"})
    ET.SubElement(weld, "child", {"link": "base"})
    ET.SubElement(weld, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    root.insert(0, weld)
    root.insert(0, world)

    if config is not None and not config.get("include_legs", True):
        strip_legs(root)
    return root


def load_dexhand(side: str, config: dict | None = None) -> ET.Element:
    """DexHand for one side: meshes repointed, mirrored/renamed if left, and
    reduced to the servo variant this robot is built with."""
    if not DEXHAND.is_file():
        raise SystemExit(
            f"DexHand URDF not found: {DEXHAND}\n"
            "The dexhand workspace has moved; update DEXHAND in this script."
        )
    mesh_dir = "meshes_dexhand/right" if side == "right" else "meshes_dexhand_left"
    text = DEXHAND.read_text().replace(
        'filename="package://dexhandv2_description/meshes/right/',
        f'filename="package://{PACKAGE}/{mesh_dir}/',
    )
    if "dexhandv2_description" in text:
        raise SystemExit("some DexHand mesh paths were not rewritten")
    root = ET.fromstring(text)

    if side == "left":
        left_meshes = HERE / "meshes_dexhand_left"
        if not left_meshes.is_dir():
            raise SystemExit(
                f"{left_meshes} missing; run mirror_meshes.py first"
            )

    for element in root:
        if element.get("name"):
            element.set("name", side_name(element.get("name"), side))
        if element.tag == "joint":
            for tag in ("parent", "child"):
                node = element.find(tag)
                node.set("link", side_name(node.get("link"), side))
            if side == "left":
                origin = element.find("origin")
                if origin is not None:
                    origin.set("xyz", as_text(mirror_vector(
                        np.fromstring(origin.get("xyz", "0 0 0"), sep=" "))))
                    origin.set("rpy", as_text(mirror_rpy(
                        np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))))
                axis = element.find("axis")
                if axis is not None:
                    axis.set("xyz", as_text(mirror_axis(
                        np.fromstring(axis.get("xyz"), sep=" "))))
        if element.tag == "link" and side == "left":
            # Visual/collision/inertial origins live in the link frame, which is
            # itself mirrored, so only their own offsets need flipping.
            for child in element:
                origin = child.find("origin") if len(child) else None
                if origin is not None:
                    origin.set("xyz", as_text(mirror_vector(
                        np.fromstring(origin.get("xyz", "0 0 0"), sep=" "))))
                    origin.set("rpy", as_text(mirror_rpy(
                        np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))))

    # After renaming, so the config's `R_` names map onto whichever side this is.
    if config is not None:
        apply_hand_variant(root, config, side)
    return root


def apply_hand_variant(root: ET.Element, config: dict, side: str) -> None:
    """Reduce the full V2 hand description to the variant this robot has.

    The vendored description is the complete hand: 16 independently actuated
    joints plus 5 that mimic. This build has 8 servos, and which joints those
    drive -- and what the rest do -- is a property of the hardware, so it is read
    from `arm_attachment.yaml` rather than from whichever URDF happens to be on
    the machine doing the generating.

    Three outcomes per joint:

    * **actuated** -- stays revolute, limits narrowed to the servo's travel.
    * **coupled** -- stays revolute and gains a `<mimic>` naming its driver. This
      is what makes the fingers curl: without it the flexor and DIP are welded
      and a finger is one rigid 75 mm link that cannot close on anything.
    * **frozen** -- becomes `fixed`, with its angle baked into the joint origin
      so a non-zero angle actually moves the link. Freezing at zero is not
      neutral; for the thumb, zero is the fully extended splayed pose.

    Upstream's own DIP mimics are re-emitted rather than kept: they name the
    driver `Index_Flexor` where the joint is `R_Index_Flexor`, so as shipped
    every one of them points at a joint that does not exist.
    """
    hand = config.get("hand")
    if not hand:
        return

    joints = {j.get("name"): j for j in root.iter("joint")}
    plan: dict[str, tuple] = {}
    for finger in hand["fingers"]:
        for row, limits in hand["actuated"].items():
            plan[side_name(f"R_{finger}_{row}", side)] = ("actuated", limits)
        for row, rule in hand["coupled"].items():
            driver = side_name(f"R_{finger}_{rule['driver']}", side)
            plan[side_name(f"R_{finger}_{row}", side)] = (
                "coupled", driver, float(rule["multiplier"]))
    for name, angle in (hand.get("thumb") or {}).items():
        plan[side_name(name, side)] = ("frozen", float(angle))

    unknown = sorted(set(plan) - set(joints))
    if unknown:
        raise SystemExit(f"config names DexHand joints that do not exist: {unknown}")

    # Couplings resolve in dependency order so a chained driver (DIP <- Flexor
    # <- Pitch) inherits limits that already reflect the stage above it.
    for name in _coupling_order(plan):
        joint = joints[name]
        action = plan[name]
        if action[0] == "actuated":
            _set_limits(joint, action[1]["lower"], action[1]["upper"])
        elif action[0] == "coupled":
            _, driver, multiplier = action
            lower, upper = _limits_of(joints[driver])
            span = sorted((lower * multiplier, upper * multiplier))
            _set_limits(joint, *span)
            for existing in joint.findall("mimic"):
                joint.remove(existing)
            ET.SubElement(joint, "mimic", {
                "joint": driver, "multiplier": f"{multiplier:.12g}", "offset": "0",
            })
        else:
            _freeze(joint, action[1])

    # `<transmission>` blocks name joints without the `R_` prefix and there is no
    # ros2_control in this workspace, so they describe nothing that exists.
    for transmission in root.findall("transmission"):
        root.remove(transmission)

    tips = hand.get("tip_frames")
    if tips:
        offset = as_text(tips["offset"])
        for finger, parent in tips["parents"].items():
            frame = side_name(f"R_{finger}_tip_frame", side)
            add_frame(root, frame)
            joint = ET.SubElement(root, "joint", {
                "name": side_name(f"R_{finger}_tip_fixed", side), "type": "fixed"})
            ET.SubElement(joint, "origin", {"xyz": offset, "rpy": "0 0 0"})
            ET.SubElement(joint, "parent", {"link": side_name(parent, side)})
            ET.SubElement(joint, "child", {"link": frame})


def _coupling_order(plan: dict) -> list[str]:
    """Joint names with every driver ahead of the joint it drives."""
    done, order = set(), []

    def visit(name: str) -> None:
        if name in done or name not in plan:
            return
        done.add(name)
        if plan[name][0] == "coupled":
            visit(plan[name][1])
        order.append(name)

    for name in plan:
        visit(name)
    return order


def _limits_of(joint: ET.Element) -> tuple[float, float]:
    limit = joint.find("limit")
    if limit is None:
        raise SystemExit(f"joint {joint.get('name')!r} drives a coupling but has no limits")
    return float(limit.get("lower")), float(limit.get("upper"))


def _set_limits(joint: ET.Element, lower: float, upper: float) -> None:
    limit = joint.find("limit")
    limit.set("lower", f"{lower:.12g}")
    limit.set("upper", f"{upper:.12g}")


def _freeze(joint: ET.Element, angle: float) -> None:
    """Weld a joint at `angle`, composing that rotation into its origin."""
    axis = joint.find("axis")
    if axis is not None and abs(angle) > 1e-15:
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        rotation = rotation_from_rpy(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
        turn = rotation_about(np.fromstring(axis.get("xyz"), sep=" "), angle)
        origin.set("rpy", as_text(rpy_from_rotation(rotation @ turn)))
    joint.set("type", "fixed")
    for tag in ("axis", "limit", "mimic"):
        for element in joint.findall(tag):
            joint.remove(element)


def strip_legs(robot: ET.Element) -> None:
    """Drop the leg subtrees.

    Removing a link means removing the joint that has it as a child, or the URDF
    stops parsing. Nothing else in this workspace references the legs: they are
    in no planning group and no hardware map.
    """
    legs = {l.get("name") for l in robot.findall("link")
            if l.get("name").startswith(LEG_PREFIX)}
    if not legs:
        return
    removed_joints = 0
    for joint in list(robot.findall("joint")):
        if (joint.find("child").get("link") in legs
                or joint.find("parent").get("link") in legs):
            robot.remove(joint)
            removed_joints += 1
    for link in list(robot.findall("link")):
        if link.get("name") in legs:
            robot.remove(link)
    print(f"  stripped {len(legs)} leg links and {removed_joints} joints")


def strip_stock_hand(robot: ET.Element, side: str) -> None:
    joint_name, link_name = STOCK_HAND[side]
    joint = robot.find(f"joint[@name='{joint_name}']")
    link = robot.find(f"link[@name='{link_name}']")
    if joint is None or link is None:
        raise SystemExit(f"expected {joint_name}/{link_name} in the upstream URDF")
    robot.remove(joint)
    robot.remove(link)


def graft(robot: ET.Element, source: ET.Element) -> None:
    """Copy links, joints and materials from one robot element into another."""
    existing = {
        tag: {e.get("name") for e in robot.findall(tag)}
        for tag in ("link", "joint", "material")
    }
    for element in source:
        name = element.get("name")
        if element.tag in ("link", "joint"):
            if name in existing[element.tag]:
                raise SystemExit(f"{element.tag} name collision: {name}")
            existing[element.tag].add(name)
        elif element.tag == "material":
            if name in existing["material"]:
                continue
            existing["material"].add(name)
        robot.append(copy.deepcopy(element))


def add_joint(robot, name, kind, parent, child, origin, axis=None, limits=None,
              effort="5", velocity="3"):
    joint = ET.SubElement(robot, "joint", {"name": name, "type": kind})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {
        "xyz": as_text(origin["xyz"]), "rpy": as_text(origin["rpy"])
    })
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": as_text(axis)})
    if limits is not None:
        ET.SubElement(joint, "limit", {
            "lower": f"{limits[0]:.9g}", "upper": f"{limits[1]:.9g}",
            "effort": str(effort), "velocity": str(velocity),
        })


def add_frame(robot: ET.Element, name: str) -> None:
    """A massless link. MoveIt warns about visual-without-collision, not this."""
    ET.SubElement(robot, "link", {"name": name})


def scale_inertial(link: ET.Element, factor: float) -> None:
    """Rescale a link's mass and inertia tensor in place."""
    inertial = link.find("inertial")
    if inertial is None:
        return
    mass = inertial.find("mass")
    if mass is not None:
        mass.set("value", f"{float(mass.get('value')) * factor:.9g}")
    inertia = inertial.find("inertia")
    if inertia is not None:
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            if inertia.get(key) is not None:
                inertia.set(key, f"{float(inertia.get(key)) * factor:.9g}")


def load_v1_forearm(side: str, config: dict) -> tuple[list[ET.Element], list[dict]]:
    """Pull the forearm and wrist out of the vendored DexHand V1 description.

    Returns the three renamed links and, for each requested wrist joint, its
    real origin, axis and limits. Nothing here is invented: the numbers are
    whatever upstream ships, read per side because the left variant is
    separately authored rather than a mirror of the right.
    """
    path = DEXHAND_V1[side]
    if not path.is_file():
        raise SystemExit(
            f"V1 description not found: {path}\n"
            "Expected the vendored copy under ros2_ws/vendor/."
        )
    root = ET.parse(path).getroot()

    density = float(config["wrist"].get("structure_density_kgm3", V1_SOURCE_DENSITY))
    factor = density / V1_SOURCE_DENSITY

    links = []
    for index, source_name in enumerate(V1_LINKS[side]):
        source = root.find(f"link[@name='{source_name}']")
        if source is None:
            raise SystemExit(f"{path}: no link named {source_name}")
        link = copy.deepcopy(source)
        link.set("name", v1_link_name(side, index))
        for mesh in link.iter("mesh"):
            mesh.set("filename", mesh.get("filename").replace(
                "package://dexhand_description/meshes/",
                f"package://{PACKAGE}/meshes_v1/",
            ))
            if "dexhand_description" in mesh.get("filename"):
                raise SystemExit(f"unrewritten V1 mesh path: {mesh.get('filename')}")
        scale_inertial(link, factor)
        links.append(link)

    joints = []
    for name in config["wrist"]["joints"]:
        source = root.find(f"joint[@name='{name}']")
        if source is None:
            raise SystemExit(f"{path}: no joint named {name}")
        origin = source.find("origin")
        axis = source.find("axis")
        limit = source.find("limit")
        joints.append({
            "name": name,
            "xyz": np.fromstring(origin.get("xyz", "0 0 0"), sep=" "),
            "rpy": np.fromstring(origin.get("rpy", "0 0 0"), sep=" "),
            "axis": np.fromstring(axis.get("xyz"), sep=" "),
            "limits": [float(limit.get("lower")), float(limit.get("upper"))],
            "effort": limit.get("effort", "5"),
            "velocity": limit.get("velocity", "3"),
        })
    return links, joints


def v1_link_name(side: str, index: int) -> str:
    return f"arm_{side}_" + ["forearm", "wrist_lower", "wrist_upper"][index]


def v1_palm_link_name(side: str, upstream: str) -> str:
    """Rename a V1 palm link per side; the two arms cannot share link names."""
    return f"arm_{side}_" + upstream.replace("-", "_").replace("_left_1", "").replace("_1", "")


def rewrite_v1_meshes(link: ET.Element) -> None:
    for mesh in link.iter("mesh"):
        mesh.set("filename", mesh.get("filename").replace(
            "package://dexhand_description/meshes/",
            f"package://{PACKAGE}/meshes_v1/",
        ))
        if "dexhand_description" in mesh.get("filename"):
            raise SystemExit(f"unrewritten V1 mesh path: {mesh.get('filename')}")


def load_v1_palm(side: str, config: dict) -> tuple[list[ET.Element], list[dict], dict]:
    """The V1 palm: the bulk chain and its cover, plus where the digits mount.

    Walked out of the description rather than listed here. The palm is whatever
    hangs off `wrist_pitch_upper` through *fixed* joints -- four bolted bulk
    pieces and a cover plate -- and the digits hang off it through revolute
    ones, so following only the fixed joints separates the two without naming
    either. A palm redesign upstream would be picked up rather than missed.

    Returns the renamed links, the fixed joints holding them together, and
    {digit: (parent link, origin element)} for the digit mounts.
    """
    root = ET.parse(DEXHAND_V1[side]).getroot()
    joints = [j for j in root.iter("joint") if j.find("parent") is not None]
    by_parent = {}
    for joint in joints:
        by_parent.setdefault(joint.find("parent").get("link"), []).append(joint)
    by_name = {j.get("name"): j for j in joints}

    driver = by_name.get(V1_PALM_DRIVER)
    if driver is None:
        raise SystemExit(f"{DEXHAND_V1[side]}: no joint named {V1_PALM_DRIVER}")
    palm_root = driver.find("child").get("link")

    factor = float(config["wrist"].get("structure_density_kgm3",
                                       V1_SOURCE_DENSITY)) / V1_SOURCE_DENSITY

    order, welds = [palm_root], []
    queue = [palm_root]
    while queue:
        for joint in by_parent.get(queue.pop(0), []):
            if joint.get("type") != "fixed":
                continue
            child = joint.find("child").get("link")
            order.append(child)
            welds.append({
                "name": joint.get("name"),
                "parent": joint.find("parent").get("link"),
                "child": child,
                "origin": joint.find("origin"),
            })
            queue.append(child)

    links = []
    for name in order:
        source = root.find(f"link[@name='{name}']")
        if source is None:
            raise SystemExit(f"{DEXHAND_V1[side]}: no link named {name}")
        link = copy.deepcopy(source)
        link.set("name", v1_palm_link_name(side, name))
        rewrite_v1_meshes(link)
        scale_inertial(link, factor)
        links.append(link)

    mounts = {}
    for digit, joint_name in V1_DIGIT_MOUNTS.items():
        joint = by_name.get(joint_name)
        if joint is None:
            raise SystemExit(f"{DEXHAND_V1[side]}: no joint named {joint_name}")
        mounts[digit] = (v1_palm_link_name(side, joint.find("parent").get("link")),
                         joint.find("origin"))
    return links, welds, mounts


def subtree(root: ET.Element, start: str) -> tuple[list[ET.Element], list[ET.Element]]:
    """Every link and joint below `start`, itself included, in document order."""
    joints = [j for j in root.iter("joint") if j.find("parent") is not None]
    by_parent = {}
    for joint in joints:
        by_parent.setdefault(joint.find("parent").get("link"), []).append(joint)

    wanted, queue = {start}, [start]
    kept_joints = []
    while queue:
        for joint in by_parent.get(queue.pop(0), []):
            child = joint.find("child").get("link")
            kept_joints.append(joint)
            wanted.add(child)
            queue.append(child)
    kept_links = [link for link in root.iter("link") if link.get("name") in wanted]
    return kept_links, kept_joints


def build_wrist_chain(robot, side, config, parent):
    """Graft the V1 forearm, wrist and -- if configured -- the V1 palm.

    Returns `(attachment, mounts)`. With a V2 palm, `attachment` is the frame
    the V2 hand welds to and `mounts` is None. With a V1 palm the palm is part
    of this chain, so `attachment` is its root and `mounts` says which bulk each
    digit bolts to.
    """
    if not config["wrist"].get("enabled", False):
        return parent, None

    links, joints = load_v1_forearm(side, config)

    # The forearm link itself carries the mount transform, which the caller has
    # already applied, so it is welded to `parent` at identity.
    robot.append(links[0])
    add_joint(robot, f"arm_{side}_forearm_weld", "fixed", parent,
              links[0].get("name"), {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})
    parent = links[0].get("name")

    v1_palm = (config.get("hand") or {}).get("palm", "v2") == "v1"
    palm_links, palm_welds, mounts = ([], [], None)
    if v1_palm:
        palm_links, palm_welds, mounts = load_v1_palm(side, config)

    # Each wrist joint drives the next V1 link. What the last one drives depends
    # on the palm: upstream it is the palm's first bulk, and with a V2 hand in
    # its place it is a bare frame for the hand interface to hang off.
    for index, spec in enumerate(joints):
        if index + 1 < len(links):
            child_link = links[index + 1]
            robot.append(child_link)
            child = child_link.get("name")
        elif v1_palm:
            robot.append(palm_links[0])
            child = palm_links[0].get("name")
        else:
            child = f"arm_{side}_wrist_tip"
            add_frame(robot, child)
        add_joint(robot, f"arm_{side}_{spec['name']}_joint", "revolute",
                  parent, child,
                  {"xyz": spec["xyz"], "rpy": spec["rpy"]},
                  spec["axis"], spec["limits"],
                  effort=spec["effort"], velocity=spec["velocity"])
        parent = child

    # Any V1 links past the last requested joint are welded on so their geometry
    # is still present rather than silently dropped.
    for link in links[len(joints) + 1:]:
        robot.append(link)
        add_joint(robot, f"weld_{link.get('name')}", "fixed", parent,
                  link.get("name"), {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})
        parent = link.get("name")

    # The rest of the palm: bolted pieces, welded exactly as upstream has them.
    for link in palm_links[1:]:
        robot.append(link)
    for weld in palm_welds:
        origin = weld["origin"]
        add_joint(robot, f"arm_{side}_{weld['name']}", "fixed",
                  v1_palm_link_name(side, weld["parent"]),
                  v1_palm_link_name(side, weld["child"]),
                  {"xyz": np.fromstring(origin.get("xyz", "0 0 0"), sep=" "),
                   "rpy": np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")}
                  if origin is not None else {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})

    return parent, mounts


def graft_v2_digits(robot, side, config, dexhand, mounts) -> None:
    """Bolt the V2 digit assemblies onto the V1 palm where V1's own digits were.

    The V2 hand's own palm link is dropped; only the digits are taken. Each one
    keeps its V2 joint exactly as the variant left it -- axis, travel, and the
    mimic tags that make the finger curl -- and is given the V1 mount's origin,
    because the bracket it bolts to is V1's.

    That the two descriptions agree about where the fingers sit is not assumed:
    referred to a common frame, the four finger mounts differ by 2 to 3 mm
    between V1 and V2. The thumb is the exception at 15 mm, which is real -- V2
    moved it -- and is why the adapter below is per-digit rather than shared.
    """
    adapters = (config.get("hand") or {}).get("digit_adapter") or {}
    for digit, (parent, origin) in mounts.items():
        joint_name = side_name(f"R_{digit}_Yaw", side)
        joint = dexhand.find(f"joint[@name='{joint_name}']")
        if joint is None:
            raise SystemExit(f"the V2 description has no joint {joint_name}")
        root_link = joint.find("child").get("link")

        adapter = adapters.get(digit) or {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        if side == "left":
            adapter = {"xyz": mirror_vector(adapter["xyz"]),
                       "rpy": mirror_rpy(adapter["rpy"])}
        joint.find("parent").set("link", parent)
        placed = joint.find("origin")
        if placed is None:
            placed = ET.SubElement(joint, "origin")
        placed.set("xyz", as_text(xyz + np.asarray(adapter["xyz"], dtype=float)))
        placed.set("rpy", as_text(rpy + np.asarray(adapter["rpy"], dtype=float)))

        links, joints = subtree(dexhand, root_link)
        for element in links + [joint] + joints:
            robot.append(element)

    # The digits' visuals name materials ("silver") that the V2 description
    # defines once at the top level. Without those definitions RViz has no
    # colour to give the hand and falls back to its error red.
    existing = {m.get("name") for m in robot.findall("material")}
    for material in dexhand.findall("material"):
        if material.get("name") not in existing:
            existing.add(material.get("name"))
            robot.append(copy.deepcopy(material))


def attach_adapter(robot, side, config, mount, link_name=None,
                   visual_only=False) -> None:
    """Give the mount link the printed adapter's geometry and mass.

    The adapter is bolted to the elbow_roll flange, so its mesh is authored in
    the elbow_roll frame (mm).  The mount link sits at `mount` relative to that
    frame, so the geometry origin is the INVERSE of the mount transform:
    re-tuning the mount moves the forearm, never the printed part.

    With `link_name`/`visual_only` (the tuning model) the mesh is hung on the
    elbow link itself at identity, as a pure visual to align the cuff against.
    """
    adapter = config["mount"].get("adapter")
    if not adapter:
        return
    name = link_name or f"arm_{side}_mount"
    link = robot.find(f"link[@name='{name}']")
    if link is None:
        raise SystemExit(f"adapter target link missing: {name}")

    if mount is None:
        origin_xyz, origin_rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        rot_inv = np.eye(3)
        t_inv = np.zeros(3)
    else:
        rot = rotation_from_rpy(mount["rpy"])
        t = np.asarray([float(v) for v in mount["xyz"]])
        rot_inv = rot.T
        t_inv = -rot_inv @ t
        origin_xyz = [float(v) for v in t_inv]
        origin_rpy = rpy_from_rotation(rot_inv)

    scale = float(adapter.get("scale", 1.0))
    tags = ("visual",) if visual_only else ("visual", "collision")
    for tag in tags:
        element = ET.SubElement(link, tag)
        ET.SubElement(element, "origin", {
            "xyz": as_text(origin_xyz), "rpy": as_text(origin_rpy)})
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "mesh", {
            "filename": adapter["mesh"], "scale": as_text([scale] * 3)})
        if tag == "visual":
            material = ET.SubElement(
                element, "material", {"name": f"arm_{side}_adapter_material"})
            ET.SubElement(material, "color", {"rgba": "0.35 0.35 0.38 1"})

    if visual_only:
        return
    inertial = ET.SubElement(link, "inertial")
    com = rot_inv @ np.asarray([float(v) for v in adapter["com"]]) + t_inv
    ET.SubElement(inertial, "origin", {
        "xyz": as_text([float(v) for v in com]), "rpy": as_text(origin_rpy)})
    ET.SubElement(inertial, "mass", {"value": f"{float(adapter['mass']):.9g}"})
    ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in adapter["inertia"])
    ET.SubElement(inertial, "inertia", {
        "ixx": f"{ixx:.9g}", "iyy": f"{iyy:.9g}", "izz": f"{izz:.9g}",
        "ixy": f"{ixy:.9g}", "ixz": f"{ixz:.9g}", "iyz": f"{iyz:.9g}"})


def attach_arm(robot, side, config, dexhand, tunable=False):
    """Strip the stock hand and build elbow -> mount -> wrist -> hand."""
    strip_stock_hand(robot, side)
    mount = config["mount"][side]
    parent = ELBOW[side]

    if tunable:
        parent = add_tunable_mount(robot, side, mount, parent)
        attach_adapter(robot, side, config, None, link_name=ELBOW[side],
                       visual_only=True)
    else:
        child = f"arm_{side}_mount"
        add_frame(robot, child)
        add_joint(robot, f"arm_{side}_forearm_mount", "fixed", parent, child, mount)
        attach_adapter(robot, side, config, mount)
        parent = child

    parent, mounts = build_wrist_chain(robot, side, config, parent)

    if mounts is not None:
        # V1 palm: the palm is already part of the chain, and only the V2
        # digits are grafted onto it. There is no hand interface to place,
        # because there is no separate hand to interface with.
        graft_v2_digits(robot, side, config, dexhand, mounts)
        return

    interface = config["hand_interface"]
    if side == "left":
        interface = {"xyz": mirror_vector(interface["xyz"]),
                     "rpy": mirror_rpy(interface["rpy"])}
    graft(robot, dexhand)
    add_joint(robot, f"arm_{side}_hand_interface", "fixed",
              parent, side_name(DEXHAND_ROOT, side), interface)


def add_tunable_mount(robot, side, mount, parent) -> str:
    """Replace the mount weld with 6 joints whose values *are* the URDF origin.

    Order matters. The three slides come first and share an orientation, so they
    add up in the parent frame exactly like an origin `xyz`. The rotations then
    go yaw, pitch, roll, composing to Rz*Ry*Rx -- which is what URDF `rpy`
    means. Read the six slider values off and they drop straight into
    arm_attachment.yaml with no conversion.
    """
    axes = [
        ("dexhand_mount_x", "prismatic", [1, 0, 0], TUNING_TRANSLATION),
        ("dexhand_mount_y", "prismatic", [0, 1, 0], TUNING_TRANSLATION),
        ("dexhand_mount_z", "prismatic", [0, 0, 1], TUNING_TRANSLATION),
        ("dexhand_mount_yaw", "revolute", [0, 0, 1], TUNING_ROTATION),
        ("dexhand_mount_pitch", "revolute", [0, 1, 0], TUNING_ROTATION),
        ("dexhand_mount_roll", "revolute", [1, 0, 0], TUNING_ROTATION),
    ]
    zero = {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}
    for name, kind, axis, span in axes:
        child = f"arm_{side}_{name}_link"
        add_frame(robot, child)
        add_joint(robot, f"arm_{side}_{name}", kind, parent, child, zero,
                  axis, [-span, span])
        parent = child
    return parent


def write(robot: ET.Element, path: Path, note: str) -> None:
    indent(robot)
    body = ET.tostring(robot, encoding="unicode")
    # NB: XML comments may not contain a double hyphen.
    header = (
        '<?xml version="1.0" ?>\n'
        "<!-- GENERATED by generate_urdf.py. Do not edit by hand. -->\n"
        f"<!-- {note} -->\n"
    )
    path.write_text(header + body + "\n")

    joints = robot.findall("joint")
    movable = [j for j in joints if j.get("type") != "fixed"]
    print(f"wrote {path.name}: {len(robot.findall('link'))} links, "
          f"{len(joints)} joints ({len(movable)} movable)")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-dexhand", action="store_true",
                        help="only regenerate the arms-only model")
    args = parser.parse_args()

    if not UPSTREAM.is_file():
        raise SystemExit(f"upstream URDF not found: {UPSTREAM}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    config = load_config()
    write(load_humanoid(config), OUTPUT_DIR / "berkeley_humanoid_lite.urdf",
          "Arms only, stock end effectors.")
    if args.skip_dexhand:
        return

    for side in ("right", "left"):
        mount = config["mount"][side]
        print(f"{side:5s} mount: xyz={as_text(mount['xyz'])} "
              f"rpy={as_text(mount['rpy'])}")

    # Right arm only, hand welded straight on: the DexHand V2 arrangement.
    v2_config = copy.deepcopy(config)
    v2_config["wrist"]["enabled"] = False
    v2_config.setdefault("hand", {})["palm"] = "v2"
    v2_config["hand_interface"] = {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}
    robot = load_humanoid(config)
    attach_arm(robot, "right", v2_config, load_dexhand("right", config))
    write(robot, OUTPUT_DIR / "berkeley_humanoid_lite_dexhand.urdf",
          "DexHand v2 (8 servo) welded to the right forearm.")

    # Both arms with the V1 forearm and wrist.
    robot = load_humanoid(config)
    for side in ("right", "left"):
        attach_arm(robot, side, config, load_dexhand(side, config))
    write(robot, OUTPUT_DIR / "berkeley_humanoid_lite_v1arm.urdf",
          "V1 forearm and 3 DOF wrist on both arms, DexHand v2 fingers.")

    # Same, with the right mount exposed as sliders.
    robot = load_humanoid(config)
    attach_arm(robot, "right", config, load_dexhand("right", config), tunable=True)
    attach_arm(robot, "left", config, load_dexhand("left", config))
    write(robot, OUTPUT_DIR / "berkeley_humanoid_lite_tuning.urdf",
          "Right mount exposed as 6 sliders; not for planning.")


if __name__ == "__main__":
    main()
