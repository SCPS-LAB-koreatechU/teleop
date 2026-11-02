#!/usr/bin/env python3
"""Export the assembled BHL v1arm scene to USD, then author UsdPhysics on it.

Run inside Blender (the MCP bridge calls the two stages separately):

    exec(open(SCRIPT, encoding="utf-8").read())
    r1 = usd_stage1_export()      # flatten link tree -> visual .usdc, restore scene
    r2 = usd_stage2_physics()     # copy -> physics .usdc with UsdPhysics authoring

Stage 1 writes  usd/bhl_v1arm_visual.usdc   (flat: /<root>/<link>/<mesh>)
Stage 2 writes  usd/bhl_v1arm_physics.usdc  (adds ArticulationRoot, RigidBody,
Mass, convex-hull collisions, Revolute/Fixed joints with limits and drives)

Conventions: Z-up, metres, joint limits authored in degrees (UsdPhysics).
URDF <mimic> couplings (Flexor<-Pitch, DIP<-Flexor) have no UsdPhysics
equivalent and are exported as independent driven joints -- enforce the
coupling in the controller, or leave those drives stiff at 0 until measured.
"""

import json
import math
import os
import shutil
import xml.etree.ElementTree as ET

REPO = r"C:\Users\jeong\OneDrive\바탕 화면\석사\휴머노이드\Berkeley-Humanoid-Lite-origin-"
URDF = os.path.join(REPO, "ros2_ws", "src", "berkeley_humanoid_lite_description",
                    "urdf", "berkeley_humanoid_lite_v1arm.urdf")
BLEND = os.path.join(REPO, "blender", "bhl_v1arm_assembly.blend")
USD_DIR = os.path.join(REPO, "usd")
USD_VISUAL = os.path.join(USD_DIR, "bhl_v1arm_visual.usdc")
USD_PHYSICS = os.path.join(USD_DIR, "bhl_v1arm_physics.usdc")
JOINTS_JSON = os.path.join(USD_DIR, "bhl_v1arm_joints.json")


# ------------------------------------------------------------------ stage 1
def usd_stage1_export():
    import bpy

    empties = {o.name: o for o in bpy.data.objects
               if o.type == "EMPTY" and "urdf_joint" in o.keys() or
               (o.type == "EMPTY" and o.name == "world")}
    # joint table from the live tree, before any reparenting
    joints = {}
    for name, e in empties.items():
        if "urdf_joint" not in e.keys():
            continue
        joints[name] = dict(
            joint=e["urdf_joint"], type=e["urdf_type"],
            parent=e.parent.name if e.parent else None,
            axis=list(e["urdf_axis"]), xyz=list(e["urdf_xyz"]),
            rpy=list(e["urdf_rpy"]),
            lower=float(e["urdf_lower"]) if "urdf_lower" in e.keys() else None,
            upper=float(e["urdf_upper"]) if "urdf_upper" in e.keys() else None,
        )
    os.makedirs(USD_DIR, exist_ok=True)
    with open(JOINTS_JSON, "w", encoding="utf-8") as fh:
        json.dump(joints, fh, indent=1)

    # flatten: every link empty becomes a root-level object, world tf kept
    for e in empties.values():
        if e.parent is not None:
            m = e.matrix_world.copy()
            e.parent = None
            e.matrix_world = m

    # select link empties + their meshes only (skip Camera/Light)
    n_sel = 0
    for o in bpy.data.objects:
        take = (o.type == "EMPTY" and o.name in empties) or \
               (o.type == "MESH" and o.parent is not None and o.parent.name in empties)
        try:
            o.select_set(take)
            n_sel += int(take)
        except RuntimeError:
            pass  # object not in the active view layer

    kwargs = dict(filepath=USD_VISUAL, selected_objects_only=True,
                  export_animation=False, export_materials=True)
    try:
        bpy.ops.wm.usd_export(root_prim_path="/bhl", **kwargs)
    except TypeError:
        bpy.ops.wm.usd_export(**kwargs)

    ok = os.path.isfile(USD_VISUAL)
    size = os.path.getsize(USD_VISUAL) if ok else 0

    # restore the parented tree in place (no file reload: pure data ops)
    from mathutils import Euler, Matrix, Vector
    for name, j in joints.items():
        e = empties[name]
        e.parent = empties[j["parent"]]
        e.matrix_parent_inverse = Matrix.Identity(4)
        e.matrix_basis = (Matrix.Translation(Vector(j["xyz"])) @
                          Euler(j["rpy"], "XYZ").to_matrix().to_4x4())
    bpy.context.view_layer.update()
    return {"exported": ok, "bytes": size, "selected": n_sel,
            "joints": len(joints)}


# ------------------------------------------------------------------ stage 2
def _parse_inertials():
    root = ET.parse(URDF).getroot()
    out = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        o = inertial.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        mass = float(inertial.find("mass").get("value"))
        it = inertial.find("inertia")
        I = [[float(it.get("ixx")), float(it.get("ixy", 0)), float(it.get("ixz", 0))],
             [float(it.get("ixy", 0)), float(it.get("iyy")), float(it.get("iyz", 0))],
             [float(it.get("ixz", 0)), float(it.get("iyz", 0)), float(it.get("izz"))]]
        out[link.get("name")] = (mass, xyz, rpy, I)
    return out


def _quat_from_rpy(r, p, y):
    """URDF rpy (fixed XYZ) -> (w, x, y, z)."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def _quat_x_to_axis(axis):
    """Quaternion (w,x,y,z) rotating +X onto unit `axis` -- half-vector form."""
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    hx, hy, hz = 1.0 + ax, ay, az
    hn = math.sqrt(hx * hx + hy * hy + hz * hz)
    if hn < 1e-8:
        return (0.0, 0.0, 0.0, 1.0)          # 180deg about Z
    hx, hy, hz = hx / hn, hy / hn, hz / hn
    w = hx                                    # X . h
    cx = 0.0 * hz - 0.0 * hy                  # X x h = (0, -hz, hy)
    return (w, 0.0, -hz, hy)


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def usd_stage2_physics():
    from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

    shutil.copyfile(USD_VISUAL, USD_PHYSICS)
    with open(JOINTS_JSON, encoding="utf-8") as fh:
        joints = json.load(fh)
    inertials = _parse_inertials()

    stage = Usd.Stage.Open(USD_PHYSICS)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # map link name -> prim path (search once)
    link_prim = {}
    wanted = set(joints) | {"world"}
    for prim in stage.Traverse():
        if prim.GetName() in wanted and prim.GetName() not in link_prim:
            link_prim[prim.GetName()] = prim.GetPath()
    missing = sorted(wanted - set(link_prim))

    root_path = stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() \
        else Sdf.Path("/root")
    UsdPhysics.Scene.Define(stage, root_path.AppendChild("physicsScene"))
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetDefaultPrim()
                                         if stage.GetDefaultPrim()
                                         else stage.GetPrimAtPath(root_path))

    import numpy as np
    n_bodies = n_colls = 0
    for name, path in link_prim.items():
        if name == "world":
            continue
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        if name in inertials:
            mass, com, rpy, I = inertials[name]
            R = np.array(_rpy_mat(*rpy))
            I_l = R @ np.array(I) @ R.T          # tensor into link axes
            w, v = np.linalg.eigh(I_l)
            if np.linalg.det(v) < 0:
                v[:, 0] *= -1
            q = _mat_to_quat(v)
            mass_api.CreateMassAttr(float(mass))
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*[float(c) for c in com]))
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*[max(float(x), 1e-9) for x in w]))
            mass_api.CreatePrincipalAxesAttr(Gf.Quatf(q[0], q[1], q[2], q[3]))
        else:
            mass_api.CreateMassAttr(0.001)
        n_bodies += 1
        for child in Usd.PrimRange(prim):
            if child.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(child)
                mc = UsdPhysics.MeshCollisionAPI.Apply(child)
                mc.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
                n_colls += 1

    jscope = stage.DefinePrim(root_path.AppendChild("joints"), "Scope")
    n_rev = n_fix = 0
    for child_link, j in joints.items():
        parent_link = j["parent"]
        if child_link not in link_prim or (parent_link not in link_prim
                                           and parent_link != "world"):
            continue
        jpath = jscope.GetPath().AppendChild(j["joint"].replace(".", "_"))
        q_origin = _quat_from_rpy(*j["rpy"])
        if j["type"] in ("revolute", "continuous"):
            joint = UsdPhysics.RevoluteJoint.Define(stage, jpath)
            q_align = _quat_x_to_axis(j["axis"])
            joint.CreateAxisAttr(UsdPhysics.Tokens.x)
            q0 = _qmul(q_origin, q_align)
            joint.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in q0]))
            joint.CreateLocalRot1Attr(Gf.Quatf(*[float(v) for v in q_align]))
            if j["lower"] is not None:
                joint.CreateLowerLimitAttr(math.degrees(j["lower"]))
                joint.CreateUpperLimitAttr(math.degrees(j["upper"]))
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr(UsdPhysics.Tokens.force)
            drive.CreateTargetPositionAttr(0.0)
            drive.CreateStiffnessAttr(0.0)     # tune in Isaac env cfg
            drive.CreateDampingAttr(0.05)
            n_rev += 1
        else:
            joint = UsdPhysics.FixedJoint.Define(stage, jpath)
            joint.CreateLocalRot0Attr(Gf.Quatf(*[float(v) for v in q_origin]))
            n_fix += 1
        if parent_link != "world":
            joint.CreateBody0Rel().SetTargets([link_prim[parent_link]])
        joint.CreateBody1Rel().SetTargets([link_prim[child_link]])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in j["xyz"]]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))

    stage.GetRootLayer().Save()
    return {"physics": USD_PHYSICS, "bodies": n_bodies, "collisions": n_colls,
            "revolute": n_rev, "fixed": n_fix,
            "bytes": os.path.getsize(USD_PHYSICS), "missing_prims": missing}


def _rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mat_to_quat(m):
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    i = max(range(3), key=lambda k: m[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(1e-12, 1.0 + m[i][i] - m[j][j] - m[k][k])) * 2
    q = [0.0] * 4
    q[0] = (m[k][j] - m[j][k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (m[j][i] + m[i][j]) / s
    q[1 + k] = (m[k][i] + m[i][k]) / s
    return tuple(q)
