#!/usr/bin/env python3
"""Drag an arm's hand around in RViz and have the real arm follow.

An interactive marker sits on the hand. Dragging it solves IK for the joints
that are actually drivable and streams the result to
hardware_bridge.py's `~/joint_command`, which slew-limits, clamps and sends it
over CAN. The RobotModel in RViz is drawn from the bridge's *measured* joint
states, so what you see move is the real arm, not the command.

  * Only `--joints` are solved for. Every other joint in the chain is held at
    its measured angle, so a dead motor (CAN 1, 3) does not get a command the
    real arm cannot follow -- the reachable set just gets smaller.
  * The marker also has roll / pitch / yaw rings about the hand's own axes.
    With six or more free joints (the 3-DOF wrist in the solve: 8 joints) IK
    matches position and attitude (orientation weighted by
    ORIENTATION_WEIGHT, low so the hand stays put and the attitude gives
    when the wrist runs out of travel). With
    the five arm joints alone the rings do nothing and IK is position-only as
    before (a weighted full-pose solve cost up to 70 mm of position there).
    The marker turns red while the hand is more than 1 cm off the requested
    point, or -- with 6+ free joints -- more than ORIENT_TOLERANCE off the
    requested attitude.
  * When you let go, the marker snaps back onto the real hand.
  * Right-click the marker for Enable / Disable / E-STOP / Reset E-STOP.
    Nothing moves until output is enabled.

    ros2 launch berkeley_humanoid_lite_moveit_config drag_teleop.launch.py live:=true
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

REACH_TOLERANCE = 0.01  # [m] beyond this the marker turns red
ORIENT_TOLERANCE = math.radians(10.0)   # [rad] same, for the attitude (6+ free joints only)
# [m per rad] how much 1 rad of attitude error counts. The V1 wrist only has
# +-20..30 deg of travel, so many requested attitudes are out of reach; the
# weight decides what gives. On 60 random poses + 10/20/30 deg turns: 0.1 cost
# up to 46 mm of position, 0.02 at most 11 mm (median 0) for the same attitude
# error, and a strict position-first null-space solve was worse (75 mm) once
# joints hit their limits.
ORIENTATION_WEIGHT = 0.02


def _rot(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def rotation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Axis * angle (base frame) that turns `current` into `target`."""
    r = target @ current.T
    cos = max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))
    angle = math.acos(cos)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle < 1e-6:
        return 0.5 * axis
    if math.pi - angle < 1e-4:
        # 180 deg: the skew part vanishes, take the axis from the diagonal
        k = int(np.argmax(np.diag(r)))
        v = r[:, k] + np.eye(3)[k]
        return angle * v / np.linalg.norm(v)
    return angle * axis / (2.0 * math.sin(angle))


def quaternion_from_matrix(m: np.ndarray) -> tuple[float, float, float, float]:
    """(w, x, y, z) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1])
    return w, x, y, z


def matrix_from_quaternion(w: float, x: float, y: float, z: float) -> np.ndarray:
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


class Chain:
    """Serial chain from `base` to `tip`, read straight from the URDF."""

    def __init__(self, urdf: str, base: str, tip: str) -> None:
        root = ET.fromstring(urdf)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        joints = []
        link = tip
        while link != base:
            joint = by_child.get(link)
            if joint is None:
                raise SystemExit(f"no joint chain from {base} to {tip}")
            joints.append(joint)
            link = joint.find("parent").get("link")
        joints.reverse()

        self.segments = []   # (name or None for fixed, origin 4x4, axis)
        self.limits: dict[str, tuple[float, float]] = {}
        for joint in joints:
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            transform = np.eye(4)
            transform[:3, :3] = _rot(rpy)
            transform[:3, 3] = xyz
            kind = joint.get("type")
            if kind in ("revolute", "continuous"):
                axis_el = joint.find("axis")
                axis = np.array([float(v) for v in (axis_el.get("xyz") if axis_el is not None else "1 0 0").split()])
                axis /= np.linalg.norm(axis)
                name = joint.get("name")
                limit = joint.find("limit")
                if kind == "revolute" and limit is not None:
                    self.limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
                else:
                    self.limits[name] = (-math.pi, math.pi)
                self.segments.append((name, transform, axis))
            elif kind == "fixed":
                self.segments.append((None, transform, None))
            else:
                raise SystemExit(f"{joint.get('name')}: joint type {kind} not supported")
        self.names = [s[0] for s in self.segments if s[0]]
        # Point on the tip link that IK places (tip frame). Zero = the link
        # origin, which on this URDF sits exactly on the elbow-roll axis.
        self.tool = np.zeros(3)

    def fk(self, q: dict[str, float]) -> np.ndarray:
        transform = np.eye(4)
        for name, origin, axis in self.segments:
            transform = transform @ origin
            if name:
                rotation = np.eye(4)
                rotation[:3, :3] = _axis_angle(axis, q.get(name, 0.0))
                transform = transform @ rotation
        tool = np.eye(4)
        tool[:3, 3] = self.tool
        return transform @ tool

    def points(self, q: dict[str, float]) -> list[tuple[str | None, np.ndarray]]:
        """Origin of every joint frame down the chain, then the tip -- the
        polyline an arm diagram is drawn from. Names are None for fixed joints."""
        transform = np.eye(4)
        out = [(None, transform[:3, 3].copy())]
        for name, origin, axis in self.segments:
            transform = transform @ origin
            out.append((name, transform[:3, 3].copy()))
            if name:
                rotation = np.eye(4)
                rotation[:3, :3] = _axis_angle(axis, q.get(name, 0.0))
                transform = transform @ rotation
        return out

    def ik(self, target: np.ndarray, seed: dict[str, float], free: list[str],
           iterations: int = 60) -> tuple[dict[str, float], float]:
        """Damped least squares on position only, inside the joint limits.
        Returns the solution and its remaining position error [m]."""
        q = dict(seed)
        eps = 1e-5
        for _ in range(iterations):
            position = self.fk(q)[:3, 3]
            error = target - position
            if np.linalg.norm(error) < 1e-4:
                break
            jacobian = np.zeros((3, len(free)))
            for column, name in enumerate(free):
                probe = dict(q)
                probe[name] += eps
                jacobian[:, column] = (self.fk(probe)[:3, 3] - position) / eps
            damping = 1e-3
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(3), error)
            step = np.clip(step, -0.2, 0.2)
            for column, name in enumerate(free):
                lower, upper = self.limits[name]
                q[name] = min(max(q[name] + step[column], lower), upper)
        return q, float(np.linalg.norm(target - self.fk(q)[:3, 3]))

    def ik_pose(self, target: np.ndarray, attitude: np.ndarray, seed: dict[str, float],
                free: list[str], weight: float = ORIENTATION_WEIGHT,
                iterations: int = 100) -> tuple[dict[str, float], float, float]:
        """Damped least squares on position + weighted orientation (axis-angle),
        inside the joint limits. `attitude` is the 3x3 target rotation of the
        tool frame in the base frame. Returns the solution, its position error
        [m] and its orientation error [rad]."""
        q = dict(seed)
        eps = 1e-5

        def residual(pose: np.ndarray) -> np.ndarray:
            return np.concatenate([target - pose[:3, 3],
                                   weight * rotation_error(attitude, pose[:3, :3])])

        for _ in range(iterations):
            pose = self.fk(q)
            error = residual(pose)
            if np.linalg.norm(error[:3]) < 1e-4 and np.linalg.norm(error[3:]) < weight * 1e-3:
                break
            jacobian = np.zeros((6, len(free)))
            for column, name in enumerate(free):
                probe = dict(q)
                probe[name] += eps
                jacobian[:, column] = (error - residual(self.fk(probe))) / eps
            damping = 1e-3
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(6), error)
            step = np.clip(step, -0.2, 0.2)
            for column, name in enumerate(free):
                lower, upper = self.limits[name]
                q[name] = min(max(q[name] + step[column], lower), upper)
        pose = self.fk(q)
        return (q, float(np.linalg.norm(target - pose[:3, 3])),
                float(np.linalg.norm(rotation_error(attitude, pose[:3, :3]))))


def hand_tool_point(urdf: str, tip: str, mesh_root: Path) -> np.ndarray:
    """A grab point inside the gripper, in the tip link frame: twice the
    centroid of the tip link's visual mesh (the link origin itself sits on the
    elbow-roll axis). Works for both hands, which are mirror images."""
    root = ET.fromstring(urdf)
    link = next((l for l in root.findall("link") if l.get("name") == tip), None)
    visual = link.find("visual") if link is not None else None
    mesh = visual.find("geometry/mesh") if visual is not None else None
    if mesh is None:
        return np.zeros(3)
    filename = mesh.get("filename")
    if filename.startswith("package://"):
        # package://<pkg>/<rel>: <rel> is relative to the package share dir, which
        # holds meshes/ and (for the DexHand model) meshes_dexhand/
        path = mesh_root.parent / filename[len("package://"):].split("/", 1)[-1]
    else:
        path = mesh_root / filename.split("/meshes/", 1)[-1]
    if not path.is_file():
        return np.zeros(3)
    data = path.read_bytes()
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
    if len(data) != 84 + 50 * count:
        return np.zeros(3)
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    points = np.frombuffer(data, dtype=record, count=count, offset=84)["v"].reshape(-1, 3)
    # the DexHand meshes are in mm with scale="0.001 0.001 0.001"
    points = points * np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
    origin = visual.find("origin")
    xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
    rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
    centroid = _rot(rpy) @ points.mean(axis=0) + np.array(xyz)
    return 2 * centroid


class DragTeleop(Node):
    def __init__(self, args) -> None:
        super().__init__(args.name)
        urdf_text = Path(args.urdf).read_text()
        self.chain = Chain(urdf_text, args.base, args.tip)
        self.chain.tool = hand_tool_point(urdf_text, args.tip, Path(args.urdf).parent.parent / "meshes")
        # From the chain, not the tip name: with the DexHand model the right tip is `base_link`.
        self.side = "오른손" if any("_right_" in n for n in self.chain.names) else "왼손"
        self.free = [n for n in args.joints if n in self.chain.names]
        unknown = [n for n in args.joints if n not in self.chain.names]
        if unknown:
            raise SystemExit(f"--joints not in the {args.base}->{args.tip} chain: {unknown}")
        if not self.free:
            raise SystemExit("no drivable joint in the chain; nothing to solve for")
        self.base = args.base
        self.bridge = args.bridge
        # The bridge clamps every command to the limits less `safety.limit_margin`;
        # solve inside the same band, or a joint IK leans on near its limit gets
        # cut back and the hand lands off target (the +-20..30 deg wrist most of all).
        self.margin = args.limit_margin
        for name in self.free:
            lower, upper = self.chain.limits[name]
            self.chain.limits[name] = self._narrow(lower, upper)

        self.lock = threading.Lock()
        self.measured: dict[str, float] = {}
        self.dragging = False
        self.last_update = 0.0
        self.seed: dict[str, float] | None = None

        self.command = self.create_publisher(JointState, f"{self.bridge}/joint_command", 10)
        self.create_subscription(JointState, "/joint_states", self._on_states, 10)
        # IK stays inside the limits the bridge actually enforces (URDF narrowed
        # by the operator limits set in arm_panel.py), so the solution and the
        # arm agree instead of the bridge clamping a joint IK was counting on.
        self.create_subscription(String, f"{self.bridge}/status", self._on_status, 10)

        self.bridge_services = {
            "enable": self.create_client(SetBool, f"{self.bridge}/enable"),
            "estop": self.create_client(Trigger, f"{self.bridge}/estop"),
            "reset_estop": self.create_client(Trigger, f"{self.bridge}/reset_estop"),
        }

        # Depth 1 (the library default) drops MOUSE_DOWN under a burst of pose
        # updates; the drag logic below copes with that, a deeper queue helps.
        self.server = InteractiveMarkerServer(self, args.name,
                                              feedback_sub_qos=QoSProfile(depth=20))
        self.menu = MenuHandler()
        self.menu.insert("토크 켜기", callback=lambda _f: self._call("enable", True))
        self.menu.insert("토크 끄기", callback=lambda _f: self._call("enable", False))
        self.menu.insert("비상정지", callback=lambda _f: self._call("estop"))
        self.menu.insert("비상정지 해제", callback=lambda _f: self._call("reset_estop"))
        self._make_marker()

        # Keep the marker on the real hand while nobody is dragging it.
        self.create_timer(0.1, self._follow_hand)
        locked = [n for n in self.chain.names if n not in self.free]
        self.get_logger().info(
            f"dragging {args.tip}; solving for {self.free}, holding {locked} at their "
            f"measured angles. Right-click the marker to enable output.")

    # ---------------------------------------------------------------- marker

    def _make_marker(self, reachable: bool = True) -> None:
        marker = InteractiveMarker()
        marker.header.frame_id = self.base
        marker.name = "hand"
        marker.description = f"{self.side}: 끌어서 이동 (우클릭: 토크 켜기/끄기, 비상정지)"
        marker.scale = 0.12
        with self.lock:
            q = self._current()
        marker.pose = self._pose_of(self.chain.fk(q))

        sphere = Marker(type=Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
        sphere.color.r, sphere.color.g, sphere.color.b = (0.2, 0.8, 0.3) if reachable else (0.9, 0.2, 0.2)
        sphere.color.a = 0.8
        free_move = InteractiveMarkerControl()
        free_move.name = "move_3d"
        free_move.interaction_mode = InteractiveMarkerControl.MOVE_3D
        free_move.always_visible = True
        free_move.markers.append(sphere)
        marker.controls.append(free_move)

        for name, (w, x, y, z) in (("move_x", (1, 1, 0, 0)), ("move_y", (1, 0, 0, 1)),
                                   ("move_z", (1, 0, 1, 0))):
            control = InteractiveMarkerControl()
            control.name = name
            control.orientation.w, control.orientation.x = float(w), float(x)
            control.orientation.y, control.orientation.z = float(y), float(z)
            control.orientation_mode = InteractiveMarkerControl.FIXED
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)
        # Roll / pitch / yaw rings about the hand's own x / y / z axes (the
        # marker pose carries the hand's attitude, INHERIT keeps them on it).
        for name, (w, x, y, z) in (("roll", (1, 1, 0, 0)), ("pitch", (1, 0, 0, 1)),
                                   ("yaw", (1, 0, 1, 0))):
            control = InteractiveMarkerControl()
            control.name = name
            control.orientation.w, control.orientation.x = float(w), float(x)
            control.orientation.y, control.orientation.z = float(y), float(z)
            control.orientation_mode = InteractiveMarkerControl.INHERIT
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        marker.controls.append(menu_control)

        self.server.insert(marker, feedback_callback=self._on_feedback)
        self.menu.apply(self.server, marker.name)
        self.server.applyChanges()
        self.reachable = reachable

    @staticmethod
    def _pose_of(transform: np.ndarray) -> Pose:
        """Marker pose from a 4x4 hand transform: position and attitude."""
        pose = Pose()
        position = transform[:3, 3]
        pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        w, x, y, z = quaternion_from_matrix(transform[:3, :3])
        pose.orientation.w, pose.orientation.x = float(w), float(x)
        pose.orientation.y, pose.orientation.z = float(y), float(z)
        return pose

    def _current(self) -> dict[str, float]:
        return {n: self.measured.get(n, 0.0) for n in self.chain.names}

    def _on_states(self, message: JointState) -> None:
        with self.lock:
            for name, position in zip(message.name, message.position):
                self.measured[name] = position

    def _on_status(self, message: String) -> None:
        try:
            joints = json.loads(message.data).get("joints", {})
        except json.JSONDecodeError:
            return
        with self.lock:
            for name in self.free:
                limits = (joints.get(name) or {}).get("limits")
                if limits and None not in limits:
                    self.chain.limits[name] = self._narrow(float(limits[0]), float(limits[1]))

    def _narrow(self, lower: float, upper: float) -> tuple[float, float]:
        """Limits less the bridge's margin (never inverted)."""
        if upper - lower > 2 * self.margin:
            return lower + self.margin, upper - self.margin
        middle = (lower + upper) / 2
        return middle, middle

    def _follow_hand(self) -> None:
        # A lost MOUSE_UP must not leave the marker detached from the arm.
        if self.dragging and time.monotonic() - self.last_update < 1.0:
            return
        self.dragging = False
        with self.lock:
            q = self._current()
        self.server.setPose("hand", self._pose_of(self.chain.fk(q)))
        self.server.applyChanges()

    def _on_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging = True
            self.last_update = time.monotonic()
            with self.lock:
                self.seed = self._current()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # A pose update is a drag even if its MOUSE_DOWN was lost.
            if not self.dragging:
                self.dragging = True
                with self.lock:
                    self.seed = self._current()
            self.last_update = time.monotonic()
            p = feedback.pose.position
            target = np.array([p.x, p.y, p.z])
            o = feedback.pose.orientation
            attitude = matrix_from_quaternion(o.w, o.x, o.y, o.z)
            with self.lock:
                seed = dict(self.seed or self._current())
                # Locked joints track the real arm, not the drag-start snapshot.
                for name in self.chain.names:
                    if name not in self.free:
                        seed[name] = self.measured.get(name, 0.0)
            with self.lock:
                for name in self.free:
                    lower, upper = self.chain.limits[name]
                    seed[name] = min(max(seed[name], lower), upper)
            if len(self.free) >= 6:
                solution, error, attitude_error = self.chain.ik_pose(target, attitude, seed, self.free)
            else:
                # Five joints cannot hold a full pose; weighting the attitude in
                # cost up to 70 mm of position in tests, so keep the old
                # position-only solve and leave the rings without effect.
                solution, error = self.chain.ik(target, seed, self.free)
                attitude_error = 0.0
            self.seed = solution
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self.free)
            message.position = [solution[n] for n in self.free]
            self.command.publish(message)
            # With fewer than six free joints the attitude is best effort, so
            # only position decides the colour.
            reachable = error <= REACH_TOLERANCE and (
                len(self.free) < 6 or attitude_error <= ORIENT_TOLERANCE)
            if reachable != self.reachable:
                self._make_marker(reachable)
                self.server.setPose("hand", feedback.pose)
                self.server.applyChanges()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.dragging = False
            if not self.reachable:
                self._make_marker(True)

    # --------------------------------------------------------------- services

    def _call(self, which: str, value: bool | None = None) -> None:
        client = self.bridge_services[which]
        if not client.service_is_ready():
            self.get_logger().error(f"{client.srv_name} is not available")
            return
        request = SetBool.Request(data=value) if value is not None else Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().warn(
            f"{client.srv_name}: {f.result().message if f.result() else 'no reply'}"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--base", default="base")
    parser.add_argument("--tip", default="arm_left_hand_link")
    parser.add_argument("--joints", nargs="+", required=True,
                        help="joints IK may move; the rest of the chain is held")
    parser.add_argument("--bridge", default="/hardware_bridge")
    parser.add_argument("--limit-margin", type=float, default=0.05,
                        help="[rad] the bridge's safety.limit_margin; IK stays this far inside the limits")
    parser.add_argument("--name", default="drag_teleop",
                        help="node name and marker namespace -- one instance per arm")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = DragTeleop(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
