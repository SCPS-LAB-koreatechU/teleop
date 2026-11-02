#!/usr/bin/env python3
"""The Berkeley Humanoid Lite, drawn from its own URDF meshes, in a Qt widget.

Used by arm_panel.py for its 3D window: every link's STL is loaded once into a
GPU buffer and placed each frame from the measured joint angles. While output
is on, the commanded pose of the driven arm is overlaid as a translucent blue
ghost, and each actuated joint gets a status-coloured marker and a label.

Mouse: drag the yellow hand ring to move the hand (IK over the drivable
joints), drag a coloured joint dot up/down to turn that one joint, drag empty
space to orbit, right-drag to pan, wheel to zoom, R to reset the view. Moves
go out through `on_command` -- arm_panel.py streams them to the bridge's
~/joint_command, the same path as the RViz marker, so its slew limit, travel
limits and E-STOP apply.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from OpenGL import GL
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QOpenGLWidget


JOINT_KO = {
    "shoulder_pitch": "어깨 앞뒤(피치)",
    "shoulder_roll": "어깨 옆벌림(롤)",
    "shoulder_yaw": "어깨 비틀기(요)",
    "elbow_pitch": "팔꿈치 굽힘(피치)",
    "elbow_roll": "팔꿈치 비틀기(롤)",
}


def joint_label(name: str) -> str:
    """arm_left_elbow_pitch_joint -> '왼팔 팔꿈치 굽힘(피치)'."""
    side = "왼팔" if "_left_" in name else "오른팔" if "_right_" in name else ""
    for key, label in JOINT_KO.items():
        if key in name:
            return f"{side} {label}".strip()
    return name


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _origin(element) -> np.ndarray:
    transform = np.eye(4)
    if element is not None:
        xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
        transform[:3, :3] = _rpy(*rpy)
        transform[:3, 3] = xyz
    return transform


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    out = np.eye(4)
    out[:3, :3] = [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
    return out


def load_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Binary (or ASCII) STL -> per-vertex positions and flat normals, float32."""
    data = path.read_bytes()
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0]) if len(data) >= 84 else 0
    if len(data) == 84 + 50 * count:
        record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        tris = np.frombuffer(data, dtype=record, count=count, offset=84)["v"]
    else:   # ASCII
        numbers = [line.split()[1:] for line in data.decode(errors="ignore").splitlines()
                   if line.strip().startswith("vertex")]
        tris = np.array(numbers, dtype=np.float32).reshape(-1, 3, 3)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.where(length == 0, 1, length)
    vertices = np.ascontiguousarray(tris.reshape(-1, 3), dtype=np.float32)
    normals = np.ascontiguousarray(np.repeat(normals, 3, axis=0), dtype=np.float32)
    return vertices, normals


class RobotModel:
    """Link tree, visuals and forward kinematics straight from the URDF."""

    def __init__(self, urdf_path: Path, mesh_root: Path) -> None:
        root = ET.fromstring(urdf_path.read_text())
        self.visuals: dict[str, list[dict]] = {}
        for link in root.findall("link"):
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    continue
                filename = mesh.get("filename")
                if filename.startswith("package://"):
                    # package://<pkg>/<rel>: <rel> is relative to the package share dir, which
                    # holds meshes/ and, for the v1arm / DexHand models, meshes_v1/,
                    # meshes_adapter/, meshes_dexhand/ and meshes_dexhand_left/
                    path = mesh_root.parent / filename[len("package://"):].split("/", 1)[-1]
                elif "/meshes/" in filename:
                    path = mesh_root / filename.split("/meshes/", 1)[-1]
                else:
                    path = Path(filename)
                color = visual.find("material/color")
                rgba = [float(v) for v in color.get("rgba").split()] if color is not None else [0.7, 0.7, 0.7, 1.0]
                rgba[3] = 1.0   # the exported alphas are arbitrary; draw solid
                scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
                self.visuals.setdefault(link.get("name"), []).append(
                    {"path": path, "origin": _origin(visual.find("origin")), "rgba": rgba, "scale": scale})

        self.joints = []     # (name, type, parent, child, origin, axis)
        children = set()
        for joint in root.findall("joint"):
            axis = joint.find("axis")
            axis = np.array([float(v) for v in (axis.get("xyz") if axis is not None else "1 0 0").split()])
            self.joints.append((joint.get("name"), joint.get("type"), joint.find("parent").get("link"),
                                joint.find("child").get("link"), _origin(joint.find("origin")),
                                axis / (np.linalg.norm(axis) or 1.0)))
            children.add(joint.find("child").get("link"))
        links = [link.get("name") for link in root.findall("link")]
        self.root = next(name for name in links if name not in children)
        self.child_joints: dict[str, list] = {}
        for joint in self.joints:
            self.child_joints.setdefault(joint[2], []).append(joint)

    def link_poses(self, q: dict[str, float]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
        """World transforms of every link, and of every joint frame."""
        links = {self.root: np.eye(4)}
        joint_frames = {}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            for name, kind, _p, child, origin, axis in self.child_joints.get(parent, []):
                frame = links[parent] @ origin
                joint_frames[name] = frame
                if kind in ("revolute", "continuous"):
                    frame = frame @ _axis_angle(axis, q.get(name, 0.0))
                links[child] = frame
                stack.append(child)
        return links, joint_frames

    def links_moved_by(self, joint_names: set[str]) -> set[str]:
        """Links downstream of any of `joint_names` -- the ones a ghost pose redraws."""
        out = set()
        stack = [(self.root, False)]
        while stack:
            link, moved = stack.pop()
            if moved:
                out.add(link)
            for name, _k, _p, child, _o, _a in self.child_joints.get(link, []):
                stack.append((child, moved or name in joint_names))
        return out


class RobotView(QOpenGLWidget):
    COLORS = {"ok": (0.12, 0.62, 0.33), "idle": (0.85, 0.47, 0.02), "bad": (0.86, 0.15, 0.15),
              "off": (0.42, 0.45, 0.50)}
    FLOOR_Z = 0.3   # [m] grid height

    def __init__(self, model: RobotModel, arms: list, on_command=None, parent=None) -> None:
        """`arms`: [(drag_teleop.Chain, tip_link), ...] -- one per draggable arm."""
        super().__init__(parent)
        self.model = model
        self.arms = arms
        self.driven = [n for chain, _tip in arms for n in chain.names]   # all draggable joints
        self.chain = None                     # the arm being dragged right now
        # Joints the bridge does not drive (DexHand fingers), from /joint_states;
        # drawn only, under the bridge's own values.
        self.extra_q: dict[str, float] = {}
        self.tip = ""
        self.on_command = on_command          # callable({joint: rad}) or None
        self.mode = None                      # None | "orbit" | "pan" | "hand" | ("joint", name)
        self.hand_target: np.ndarray | None = None
        self.plane: tuple[np.ndarray, np.ndarray] | None = None   # (point, normal)
        self.seed: dict[str, float] | None = None
        self.joint_angle = 0.0
        self.mvp = np.eye(4)
        self.hand_screens: list[QPointF | None] = [None] * len(arms)
        self.dots: dict[str, QPointF] = {}
        self.reach_error = 0.0
        self.ghost_links = model.links_moved_by(set(self.driven))
        self.meshes: dict[str, list[tuple]] = {}   # link -> [(vbo, nbo, count, origin, rgba)]
        self.status: dict | None = None
        self.overlay_text = ""
        self.notice = ""
        self.yaw, self.pitch, self.distance = math.radians(35), math.radians(15), 1.6
        self.target = np.array([0.0, 0.0, 0.62])
        self._last = None
        self.setMinimumSize(400, 300)
        self.setFocusPolicy(Qt.StrongFocus)

    # ------------------------------------------------------------------ input

    def _drivable(self, names: list[str] | None = None) -> list[str]:
        """Joints a drag may command: those of `names` (default: every arm)
        that the bridge has live."""
        if self.status is None:
            return []
        joints = self.status["joints"]
        return [n for n in (names if names is not None else self.driven)
                if joints.get(n, {}).get("live") and not joints[n].get("fault")]

    def _measured(self) -> dict[str, float]:
        return {n: j["measured"] for n, j in (self.status or {}).get("joints", {}).items()}

    def _limits(self, name: str) -> tuple[float, float]:
        info = (self.status or {}).get("joints", {}).get(name, {})
        lower, upper = info.get("limits") or info.get("urdf") or (-math.pi, math.pi)
        return float(lower), float(upper)

    def _ray(self, x: float, y: float) -> tuple[np.ndarray, np.ndarray]:
        """World-space ray under the widget point (x, y)."""
        inverse = np.linalg.inv(self.mvp)
        ndc_x, ndc_y = 2 * x / self.width() - 1, 1 - 2 * y / self.height()
        near = inverse @ np.array([ndc_x, ndc_y, -1.0, 1.0])
        far = inverse @ np.array([ndc_x, ndc_y, 1.0, 1.0])
        near, far = near[:3] / near[3], far[:3] / far[3]
        direction = far - near
        return near, direction / np.linalg.norm(direction)

    def mousePressEvent(self, event) -> None:
        self._last = (event.pos(), event.button())
        self.mode = "orbit" if event.button() == Qt.LeftButton else "pan"
        if event.button() != Qt.LeftButton or not self.arms or self.on_command is None:
            return
        grab = 22 * self._ui_scale()
        point = QPointF(event.pos())
        drivable = self._drivable()
        near = lambda q: q is not None and (q - point).manhattanLength() < grab  # noqa: E731
        if self.status is not None and not drivable and (
                any(near(q) for q in self.hand_screens) or any(near(q) for q in self.dots.values())):
            self.notice = "토크를 켜야 3D 화면에서 팔을 움직일 수 있습니다"
            self.update()
            return
        arm = next((k for k, q in enumerate(self.hand_screens) if near(q)), None)
        if arm is not None and self._drivable(self.arms[arm][0].names):
            self.chain, self.tip = self.arms[arm]
            hand = self.chain.fk(self._measured())[:3, 3]
            eye = self._camera()
            normal = (eye - hand) / np.linalg.norm(eye - hand)
            self.plane = (hand, normal)
            self.hand_target = hand.copy()
            self.seed = self._measured()
            self.mode = "hand"
            return
        for name, at in self.dots.items():
            if name in drivable and (at - point).manhattanLength() < grab:
                self.mode = ("joint", name)
                self.joint_angle = self._measured().get(name, 0.0)
                return

    def mouseMoveEvent(self, event) -> None:
        if self._last is None:
            return
        previous, button = self._last
        delta = event.pos() - previous
        self._last = (event.pos(), button)
        if self.mode == "hand":
            origin, direction = self._ray(event.x(), event.y())
            point, normal = self.plane
            denominator = direction @ normal
            if abs(denominator) > 1e-6:
                self.hand_target = origin + direction * ((point - origin) @ normal) / denominator
                self._solve_hand()
        elif isinstance(self.mode, tuple):
            name = self.mode[1]
            lower, upper = self._limits(name)
            # Up = positive, 0.4 deg per pixel.
            self.joint_angle = min(max(self.joint_angle - math.radians(0.4) * delta.y(), lower), upper)
            self.on_command({name: self.joint_angle})
        elif self.mode == "orbit":
            self.yaw -= delta.x() * 0.01
            self.pitch = min(max(self.pitch + delta.y() * 0.01, -1.4), 1.4)
        else:
            right = np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.0])
            scale = self.distance * 0.0015
            self.target += -right * delta.x() * scale + np.array([0, 0, 1.0]) * delta.y() * scale
        self.update()

    def _solve_hand(self) -> None:
        free = self._drivable(self.chain.names)
        if not free:
            return
        seed = dict(self.seed or self._measured())
        measured = self._measured()
        for name in self.chain.names:
            if name not in free:
                seed[name] = measured.get(name, 0.0)     # held joints track the real arm
        for name in free:
            self.chain.limits[name] = self._limits(name)
            lower, upper = self.chain.limits[name]
            seed[name] = min(max(seed.get(name, 0.0), lower), upper)
        solution, self.reach_error = self.chain.ik(self.hand_target, seed, free)
        self.seed = solution
        self.on_command({name: solution[name] for name in free})

    def mouseReleaseEvent(self, _event) -> None:
        self._last = None
        self.mode = None
        self.hand_target = None
        self.reach_error = 0.0

    def wheelEvent(self, event) -> None:
        self.distance *= 0.9 if event.angleDelta().y() > 0 else 1 / 0.9
        self.distance = min(max(self.distance, 0.3), 6.0)
        self.update()

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_R:
            self.yaw, self.pitch, self.distance = 0.0, 0.0, 1.6
            self.target = np.array([0.0, 0.0, 0.62])
            self.update()
        else:
            super().keyPressEvent(event)

    def set_status(self, status: dict | None, text: str = "") -> None:
        self.status = status
        if self.notice and self._drivable():
            self.notice = ""
        self.overlay_text = text
        self.update()

    # --------------------------------------------------------------------- GL

    def initializeGL(self) -> None:
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_LIGHT0)
        GL.glEnable(GL.GL_LIGHT1)
        GL.glEnable(GL.GL_NORMALIZE)
        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE)
        GL.glLightModeli(GL.GL_LIGHT_MODEL_TWO_SIDE, GL.GL_TRUE)
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, (0.85, 0.85, 0.85, 1.0))
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_SPECULAR, (0.4, 0.4, 0.4, 1.0))
        GL.glLightfv(GL.GL_LIGHT1, GL.GL_DIFFUSE, (0.35, 0.35, 0.4, 1.0))
        GL.glLightModelfv(GL.GL_LIGHT_MODEL_AMBIENT, (0.25, 0.25, 0.28, 1.0))
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_SPECULAR, (0.3, 0.3, 0.3, 1.0))
        GL.glMaterialf(GL.GL_FRONT_AND_BACK, GL.GL_SHININESS, 40.0)
        GL.glShadeModel(GL.GL_SMOOTH)
        GL.glEnable(GL.GL_MULTISAMPLE)
        cache: dict[Path, tuple] = {}
        for link, visuals in self.model.visuals.items():
            for visual in visuals:
                path = visual["path"]
                if not path.is_file():
                    continue
                if path not in cache:
                    vertices, normals = load_stl(path)
                    vertices = vertices * np.array(visual["scale"], dtype=np.float32)
                    vbo, nbo = GL.glGenBuffers(2)
                    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
                    GL.glBufferData(GL.GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL.GL_STATIC_DRAW)
                    GL.glBindBuffer(GL.GL_ARRAY_BUFFER, nbo)
                    GL.glBufferData(GL.GL_ARRAY_BUFFER, normals.nbytes, normals, GL.GL_STATIC_DRAW)
                    cache[path] = (vbo, nbo, len(vertices))
                vbo, nbo, count = cache[path]
                self.meshes.setdefault(link, []).append((vbo, nbo, count, visual["origin"], visual["rgba"]))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

    def resizeGL(self, width: int, height: int) -> None:
        GL.glViewport(0, 0, width, height)

    def _camera(self) -> np.ndarray:
        eye_dir = np.array([math.cos(self.pitch) * math.cos(self.yaw),
                            math.cos(self.pitch) * math.sin(self.yaw), math.sin(self.pitch)])
        return self.target + eye_dir * self.distance

    def _view_matrix(self) -> np.ndarray:
        eye = self._camera()
        forward = self.target - eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        view = np.eye(4)
        view[0, :3], view[1, :3], view[2, :3] = right, up, -forward
        view[:3, 3] = -view[:3, :3] @ eye
        return view

    def _projection(self) -> np.ndarray:
        aspect = max(self.width(), 1) / max(self.height(), 1)
        near, far, fov = 0.02, 20.0, math.radians(40)
        f = 1 / math.tan(fov / 2)
        return np.array([[f / aspect, 0, 0, 0], [0, f, 0, 0],
                         [0, 0, (far + near) / (near - far), 2 * far * near / (near - far)],
                         [0, 0, -1, 0]])

    def _to_screen(self, point, mvp) -> QPointF | None:
        clip = mvp @ np.append(point, 1.0)
        if clip[3] <= 0:
            return None
        ndc = clip[:3] / clip[3]
        return QPointF((ndc[0] + 1) / 2 * self.width(), (1 - ndc[1]) / 2 * self.height())

    def _draw_links(self, poses: dict[str, np.ndarray], links, color=None, alpha: float = 1.0) -> None:
        GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
        GL.glEnableClientState(GL.GL_NORMAL_ARRAY)
        for link in links:
            if link not in poses:
                continue
            for vbo, nbo, count, origin, rgba in self.meshes.get(link, []):
                GL.glPushMatrix()
                GL.glMultMatrixd((poses[link] @ origin).T.flatten())
                r, g, b = color if color is not None else rgba[:3]
                GL.glColor4f(r, g, b, alpha)
                GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
                GL.glVertexPointer(3, GL.GL_FLOAT, 0, None)
                GL.glBindBuffer(GL.GL_ARRAY_BUFFER, nbo)
                GL.glNormalPointer(GL.GL_FLOAT, 0, None)
                GL.glDrawArrays(GL.GL_TRIANGLES, 0, count)
                GL.glPopMatrix()
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
        GL.glDisableClientState(GL.GL_NORMAL_ARRAY)

    def paintGL(self) -> None:
        painter = QPainter(self)
        painter.beginNativePainting()
        GL.glClearColor(0.12, 0.14, 0.18, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_COLOR_MATERIAL)
        GL.glUseProgram(0)
        projection, view = self._projection(), self._view_matrix()
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadMatrixd(projection.T.flatten())
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        # Lights fixed to the camera: key from upper left, fill from lower right.
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, (-1.0, 1.0, 1.5, 0.0))
        GL.glLightfv(GL.GL_LIGHT1, GL.GL_POSITION, (1.0, -0.5, 0.5, 0.0))
        GL.glLoadMatrixd(view.T.flatten())

        self._draw_floor()
        status = self.status
        joint_screen = []
        if status is not None:
            joints = status["joints"]
            measured = {**self.extra_q, **{n: j["measured"] for n, j in joints.items()}}
            poses, frames = self.model.link_poses(measured)
            self._draw_links(poses, self.meshes.keys())
            if any(joints.get(n, {}).get("live") for n in self.driven):
                commanded = {**self.extra_q, **{n: j["command"] for n, j in joints.items()}}
                ghost, _ = self.model.link_poses(commanded)
                GL.glEnable(GL.GL_BLEND)
                GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
                GL.glDepthMask(GL.GL_FALSE)
                self._draw_links(ghost, self.ghost_links, color=(0.35, 0.6, 1.0), alpha=0.35)
                GL.glDepthMask(GL.GL_TRUE)
                GL.glDisable(GL.GL_BLEND)
            mvp = projection @ view
            self.mvp = mvp
            self.hand_screens = [self._to_screen(chain.fk(measured)[:3, 3], mvp)
                                 for chain, _tip in self.arms]
            target_screen = (self._to_screen(self.hand_target, mvp)
                             if self.hand_target is not None else None)
            for name, info in joints.items():
                if name in frames:
                    at = self._to_screen(frames[name][:3, 3], mvp)
                    if at is not None:
                        joint_screen.append((name, info, at))
        # Hand the context back the way Qt's paint engine expects it; left on,
        # depth testing and lighting swallow every label drawn below.
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glDisable(GL.GL_LIGHTING)
        GL.glDisable(GL.GL_COLOR_MATERIAL)
        GL.glDisable(GL.GL_BLEND)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        painter.endNativePainting()

        # 2D overlay: joint markers, labels, help text -- all sized with the
        # window, like the robot itself.
        k = self._ui_scale()
        painter.setRenderHint(QPainter.Antialiasing)
        placed: list[QPointF] = []
        self.dots = {name: at for name, info, at in joint_screen}
        for (chain, tip), hand_screen in zip(self.arms, self.hand_screens if status is not None else []):
            if hand_screen is None:
                continue
            live = bool(self._drivable(chain.names))
            side = "오른손" if "_right_" in tip else "왼손"
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor(251, 191, 36) if live else QColor(156, 163, 175), 4 * k))
            painter.drawEllipse(hand_screen, 16 * k, 16 * k)
            painter.setFont(self._font(14 * k, True))
            painter.drawText(hand_screen + QPointF(-60 * k, 40 * k),
                             f"{side}: 끌어서 이동" if live else
                             f"{side} (선택 해제)" if not any(
                                 (status["joints"].get(n) or {}).get("arm_active", True)
                                 for n in chain.names if n in status["joints"]) else
                             f"{side} (토크 꺼짐)")
            if target_screen is not None and chain is self.chain:
                far = self.reach_error > 0.01
                painter.setPen(QPen(QColor(239, 68, 68) if far else QColor(251, 191, 36), 2 * k, Qt.DashLine))
                painter.drawLine(hand_screen, target_screen)
                painter.setBrush(QColor(239, 68, 68) if far else QColor(251, 191, 36))
                painter.drawEllipse(target_screen, 7 * k, 7 * k)
                if far:
                    painter.drawText(target_screen + QPointF(-60 * k, 28 * k),
                                     f"닿지 않음 ({self.reach_error * 100:.1f} cm)")
        for name, info, at in joint_screen:
            key = self._state(status, info)
            r, g, b = self.COLORS[key]
            painter.setPen(QColor(17, 24, 39))
            painter.setBrush(QColor(int(r * 255), int(g * 255), int(b * 255)))
            painter.drawEllipse(at, 9 * k, 9 * k)
            if name not in self.driven:
                continue   # label only the arm being driven; the other arm just gets dots
            label = at + QPointF(14 * k, 5 * k)
            while any(abs(label.y() - q.y()) < 20 * k and abs(label.x() - q.x()) < 170 * k for q in placed):
                label += QPointF(0, 20 * k)
            placed.append(label)
            text = (f"{info.get('device_id', '')}번 {joint_label(name).split(' ', 1)[-1]}  "
                    f"{math.degrees(info.get('measured', 0.0)):+.1f}°")
            painter.setFont(self._font(15 * k, True))
            painter.setPen(QColor(0, 0, 0, 160))
            painter.drawText(label + QPointF(1, 1), text)
            painter.setPen(QColor(243, 244, 246))
            painter.drawText(label, text)
        painter.setPen(QColor(229, 231, 235))
        painter.setFont(self._font(16 * k, True))
        painter.drawText(QPointF(12 * k, 26 * k), "버클리 휴머노이드 라이트")
        painter.setFont(self._font(14 * k))
        painter.drawText(QPointF(12 * k, 50 * k),
                         "손(노란 원) 끌기: 팔 이동 · 관절 점 위아래로 끌기: 그 관절만 회전 · "
                         "빈 곳 끌기: 화면 회전 · 오른쪽 드래그: 화면 이동 · 휠: 확대 · R: 정면")
        painter.setFont(self._font(14 * k))
        painter.drawText(QPointF(12 * k, self.height() - 12 * k),
                         "파란 반투명: 목표 자세   관절 점: 초록 구동 중 / 주황 토크 꺼짐 / 빨강 끊김·오류 / 회색 사용 안 함")
        if self.notice:
            painter.setPen(QColor(251, 191, 36))
            painter.setFont(self._font(18 * k, True))
            painter.drawText(QPointF(12 * k, 80 * k), self.notice)
        if self.overlay_text:
            painter.setPen(QColor(248, 113, 113))
            painter.setFont(self._font(24 * k, True))
            painter.drawText(self.rect(), Qt.AlignCenter, self.overlay_text)
        painter.end()

    REFERENCE_SIZE = (1100, 850)   # overlay text is designed for this window size

    def _ui_scale(self) -> float:
        return min(max(min(self.width() / self.REFERENCE_SIZE[0],
                           self.height() / self.REFERENCE_SIZE[1]), 0.5), 4.0)

    @staticmethod
    def _font(pixels: float, bold: bool = False) -> QFont:
        font = QFont("Sans")
        font.setPixelSize(max(6, round(pixels)))
        font.setBold(bold)
        return font

    @staticmethod
    def _state(status: dict, info: dict) -> str:
        if info.get("fault") or (status["live"] and info.get("enabled") and not info.get("seen")):
            return "bad"
        if info.get("live"):
            return "ok"
        if info.get("enabled"):
            return "idle"
        return "off"

    def _draw_floor(self) -> None:
        GL.glDisable(GL.GL_LIGHTING)
        GL.glLineWidth(1.0)
        GL.glColor3f(0.27, 0.30, 0.36)
        GL.glBegin(GL.GL_LINES)
        # The model is the upper body only, so the grid sits just under the
        # hands (like a table) instead of on the floor far below.
        z = self.FLOOR_Z
        for k in range(-8, 9):
            v = k * 0.1
            GL.glVertex3f(v, -0.8, z)
            GL.glVertex3f(v, 0.8, z)
            GL.glVertex3f(-0.8, v, z)
            GL.glVertex3f(0.8, v, z)
        GL.glEnd()
        GL.glLineWidth(3.0)
        GL.glBegin(GL.GL_LINES)
        for axis, color in ((0, (0.94, 0.27, 0.27)), (1, (0.13, 0.77, 0.37)), (2, (0.23, 0.51, 0.96))):
            GL.glColor3f(*color)
            GL.glVertex3f(0.0, 0.0, self.FLOOR_Z)
            end = [0.0, 0.0, self.FLOOR_Z]
            end[axis] += 0.15
            GL.glVertex3f(*end)
        GL.glEnd()
        GL.glEnable(GL.GL_LIGHTING)
