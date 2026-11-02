#!/usr/bin/env python3
"""Import a Berkeley-Humanoid-Lite URDF into Blender as an articulated empty tree.

Run inside Blender (Scripting tab, or via the MCP bridge):

    exec(open(r"...\\scripts\\blender_import_urdf.py", encoding="utf-8").read())

Every URDF link becomes an Empty whose local matrix is the joint origin, so
the kinematic tree is live: rotate an empty about its stored joint axis and
the whole subtree follows.  Visual meshes are parented to their link empty
with the visual origin (and mm->m scale) baked into the object matrix.

Two module-level helpers are registered for later interactive use:

    bhl_set_joint("R_Index_Pitch", 0.6)   # radians, respects stored axis
    bhl_zero_pose()                        # every joint back to zero

package:// paths are resolved through PACKAGE_MAP below because the repo's
symlinks (meshes, meshes_v1, meshes_dexhand) do not survive a Windows
checkout.  Missing STL files are collected, not fatal.
"""

import math
import os
import xml.etree.ElementTree as ET

# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------
REPO_ROOT = os.environ.get(
    "BHL_REPO",
    r"C:\Users\jeong\OneDrive\바탕 화면\석사\휴머노이드\Berkeley-Humanoid-Lite-origin-",
)
URDF_PATH = os.path.join(
    REPO_ROOT, "ros2_ws", "src", "berkeley_humanoid_lite_description",
    "urdf", "berkeley_humanoid_lite_v1arm.urdf",
)

_PKG = "package://berkeley_humanoid_lite_description/"
PACKAGE_MAP = {
    # order matters: longest prefix first
    "meshes_dexhand_left": os.path.join(
        REPO_ROOT, "ros2_ws", "src", "berkeley_humanoid_lite_description",
        "meshes_dexhand_left"),
    "meshes_dexhand": os.path.join(
        REPO_ROOT, "ros2_ws", "vendor", "dexhandv2_description", "meshes"),
    "meshes_v1": os.path.join(
        REPO_ROOT, "ros2_ws", "vendor", "dexhand_v1_description", "meshes"),
    "meshes_adapter": os.path.join(
        REPO_ROOT, "ros2_ws", "src", "berkeley_humanoid_lite_description",
        "meshes_adapter"),
    "meshes": os.path.join(
        REPO_ROOT, "source", "berkeley_humanoid_lite_assets", "data",
        "robots", "berkeley_humanoid", "berkeley_humanoid_lite", "meshes"),
}

COLLECTION_ROOTS = [
    # (subtree root link, collection name) — nearest matching ancestor wins
    ("Index_Knuckle_Cross_1", "Hand_R"), ("Middle_Knuckle_Cross_1", "Hand_R"),
    ("Ring_Knuckle_Cross_1", "Hand_R"), ("Pinky_Knuckle_Cross_1", "Hand_R"),
    ("Thumb_Prox_KC_1", "Hand_R"),
    ("L_Index_Knuckle_Cross_1", "Hand_L"), ("L_Middle_Knuckle_Cross_1", "Hand_L"),
    ("L_Ring_Knuckle_Cross_1", "Hand_L"), ("L_Pinky_Knuckle_Cross_1", "Hand_L"),
    ("L_Thumb_Prox_KC_1", "Hand_L"),
    ("arm_right_mount", "Forearm_R"),
    ("arm_left_mount", "Forearm_L"),
    ("arm_right_shoulder_pitch", "Arm_R"),
    ("arm_left_shoulder_pitch", "Arm_L"),
]
DEFAULT_COLLECTION = "Torso"


# --------------------------------------------------------------------------
# URDF parsing (no bpy needed)
# --------------------------------------------------------------------------
def _floats(s, default):
    if not s:
        return list(default)
    return [float(v) for v in s.replace(",", " ").split()]


def parse_urdf(path):
    root = ET.parse(path).getroot()
    materials = {}
    for m in root.findall("material"):
        c = m.find("color")
        if c is not None:
            materials[m.get("name")] = _floats(c.get("rgba"), (0.7, 0.7, 0.7, 1))

    links = {}
    for l in root.findall("link"):
        visuals = []
        for v in l.findall("visual"):
            o = v.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None, (0, 0, 0))
            rpy = _floats(o.get("rpy") if o is not None else None, (0, 0, 0))
            g = v.find("geometry/mesh")
            if g is None:
                continue
            scale = _floats(g.get("scale"), (1, 1, 1))
            mat = v.find("material")
            rgba = None
            if mat is not None:
                c = mat.find("color")
                if c is not None:
                    rgba = _floats(c.get("rgba"), (0.7, 0.7, 0.7, 1))
                elif mat.get("name") in materials:
                    rgba = materials[mat.get("name")]
            visuals.append(dict(filename=g.get("filename"), xyz=xyz, rpy=rpy,
                                scale=scale, rgba=rgba))
        links[l.get("name")] = visuals

    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        joints.append(dict(
            name=j.get("name"), type=j.get("type"),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=_floats(o.get("xyz") if o is not None else None, (0, 0, 0)),
            rpy=_floats(o.get("rpy") if o is not None else None, (0, 0, 0)),
            axis=_floats(a.get("xyz") if a is not None else None, (1, 0, 0)),
            lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
            upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
        ))

    children = {}
    joint_of = {}
    for j in joints:
        children.setdefault(j["parent"], []).append(j["child"])
        joint_of[j["child"]] = j
    roots = [n for n in links if n not in joint_of]
    return links, joints, joint_of, children, roots


def resolve_mesh(filename):
    """package://berkeley_humanoid_lite_description/<dir>/<rest> -> abs path"""
    if not filename.startswith(_PKG):
        return filename
    rel = filename[len(_PKG):]
    for prefix in ("meshes_dexhand_left", "meshes_dexhand", "meshes_v1",
                   "meshes_adapter", "meshes"):
        if rel == prefix or rel.startswith(prefix + "/"):
            rest = rel[len(prefix):].lstrip("/")
            return os.path.normpath(os.path.join(PACKAGE_MAP[prefix], *rest.split("/")))
    return os.path.normpath(os.path.join(REPO_ROOT, *rel.split("/")))


def collection_for(link, joint_of):
    """Walk ancestors until a COLLECTION_ROOTS entry matches."""
    seen = set()
    n = link
    while n is not None and n not in seen:
        seen.add(n)
        for root_link, coll in COLLECTION_ROOTS:
            if n == root_link:
                return coll
        j = joint_of.get(n)
        n = j["parent"] if j else None
    return DEFAULT_COLLECTION


# --------------------------------------------------------------------------
# Blender build
# --------------------------------------------------------------------------
def build_in_blender():
    import bpy
    from mathutils import Euler, Matrix, Vector

    links, joints, joint_of, children, roots = parse_urdf(URDF_PATH)

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0

    # fresh collections
    colls = {}
    for name in [DEFAULT_COLLECTION] + [c for _, c in COLLECTION_ROOTS]:
        if name in bpy.data.collections:
            coll = bpy.data.collections[name]
        else:
            coll = bpy.data.collections.new(name)
            scene.collection.children.link(coll)
        colls[name] = coll

    def link_to_coll(obj, name):
        for c in obj.users_collection:
            c.objects.unlink(obj)
        colls[name].objects.link(obj)

    def loc_rot(xyz, rpy):
        return Matrix.Translation(Vector(xyz)) @ Euler(rpy, "XYZ").to_matrix().to_4x4()

    # materials
    mat_cache = {}

    def material_for(rgba):
        key = tuple(round(v, 5) for v in rgba)
        if key in mat_cache:
            return mat_cache[key]
        m = bpy.data.materials.new("bhl_" + "_".join(f"{v:.2f}" for v in key))
        m.use_nodes = True
        bsdf = m.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = key
            if key[3] < 1.0:
                bsdf.inputs["Alpha"].default_value = key[3]
                m.blend_method = "BLEND"
        m.diffuse_color = key
        mat_cache[key] = m
        return m

    # mesh import with datablock cache
    mesh_cache = {}
    missing = []

    def import_mesh(path):
        if path in mesh_cache:
            return mesh_cache[path]
        if not os.path.isfile(path):
            missing.append(path)
            return None
        before = set(bpy.data.objects)
        try:
            bpy.ops.wm.stl_import(filepath=path)
        except AttributeError:
            bpy.ops.import_mesh.stl(filepath=path)
        new = [o for o in bpy.data.objects if o not in before]
        if not new:
            missing.append(path)
            return None
        data = new[0].data
        for o in new:
            bpy.data.objects.remove(o)
        mesh_cache[path] = data
        return data

    # empties for every link, wired into the tree
    empties = {}
    for name in links:
        e = bpy.data.objects.new(name, None)
        e.empty_display_type = "ARROWS"
        e.empty_display_size = 0.02
        link_to_coll(e, collection_for(name, joint_of))
        empties[name] = e

    for j in joints:
        c, p = empties[j["child"]], empties[j["parent"]]
        c.parent = p
        c.matrix_parent_inverse = Matrix.Identity(4)
        c.matrix_basis = loc_rot(j["xyz"], j["rpy"])
        c["urdf_joint"] = j["name"]
        c["urdf_type"] = j["type"]
        c["urdf_axis"] = list(j["axis"])
        c["urdf_xyz"] = list(j["xyz"])
        c["urdf_rpy"] = list(j["rpy"])
        if j["lower"] is not None:
            c["urdf_lower"] = j["lower"]
            c["urdf_upper"] = j["upper"]

    # visual meshes
    n_meshes = 0
    for name, visuals in links.items():
        for i, v in enumerate(visuals):
            data = import_mesh(resolve_mesh(v["filename"]))
            if data is None:
                continue
            o = bpy.data.objects.new(f"{name}_v{i}", data)
            o.parent = empties[name]
            o.matrix_parent_inverse = Matrix.Identity(4)
            o.matrix_basis = loc_rot(v["xyz"], v["rpy"]) @ Matrix.Diagonal(
                (*v["scale"], 1.0))
            if v["rgba"]:
                o.data.materials.clear()
                o.data.materials.append(material_for(v["rgba"]))
            link_to_coll(o, collection_for(name, joint_of))
            n_meshes += 1

    # joint helpers, kept importable from later exec() calls
    def bhl_set_joint(joint_name, value):
        from mathutils import Matrix as M, Vector as V
        for e in empties.values():
            if e.get("urdf_joint") == joint_name:
                base = loc_rot(list(e["urdf_xyz"]), list(e["urdf_rpy"]))
                axis = V(list(e["urdf_axis"]))
                if e["urdf_type"] == "prismatic":
                    e.matrix_basis = base @ M.Translation(axis.normalized() * value)
                else:
                    e.matrix_basis = base @ M.Rotation(value, 4, axis.normalized())
                e["urdf_value"] = value
                return True
        raise KeyError(joint_name)

    def bhl_zero_pose():
        for e in empties.values():
            if "urdf_xyz" in e.keys():
                e.matrix_basis = loc_rot(list(e["urdf_xyz"]), list(e["urdf_rpy"]))
                e["urdf_value"] = 0.0

    import builtins
    builtins.bhl_set_joint = bhl_set_joint
    builtins.bhl_zero_pose = bhl_zero_pose
    builtins.bhl_empties = empties

    bpy.context.view_layer.update()
    return dict(
        links=len(links), joints=len(joints), meshes=n_meshes,
        missing=missing, roots=roots,
    )


if __name__ == "__main__":
    try:
        import bpy  # noqa: F401
        result = build_in_blender()
        print("BHL import:", result)
    except ImportError:
        # standalone smoke test: parse + path mapping only
        links, joints, joint_of, children, roots = parse_urdf(URDF_PATH)
        print(f"links={len(links)} joints={len(joints)} roots={roots}")
        shown = set()
        for vs in links.values():
            for v in vs:
                p = resolve_mesh(v["filename"])
                head = v["filename"].split("/")[2]
                if head not in shown:
                    shown.add(head)
                    print(f"  {v['filename']}\n    -> {p}")
