#!/usr/bin/env python3
"""Operator panel for hardware_bridge.py: link status, E-STOP, travel limits,
and a separate 3D window showing the arm.

Runs next to RViz while drag_teleop.py moves the arm. Everything shown comes
from the bridge's `~/status` topic, so what the panel says is what the bridge
believes -- if the bridge stops publishing, the panel says so in red.

  * E-STOP: the big red button, or Esc (in either window). It latches
    in the bridge (all motors IDLE, torque off -- the arm will sag under
    gravity); Reset E-STOP clears the latch and output stays off until Enable.
  * Per-motor travel limits in degrees, inside the URDF range. Applied limits
    take effect at once (a joint outside its new range is walked back in at
    the bridge's slew rate) and drag IK respects them. Save writes them to the
    limits file the bridge loads at start-up.
  * The 3D window draws the whole robot from its URDF meshes at the measured
    joint angles, with the commanded pose as a translucent blue ghost while
    output is on (robot_view.py).
  * The top row shows the system as a chain -- RViz marker, drag IK, bridge,
    CAN, motors -- each block coloured by whether that stage is alive.
  * The reference (0 deg) is each motor's pose when it first answers after
    power-on (bridge --zero-at-start), fixed for the whole run; there is
    deliberately no re-zero button.
  * Closing the panel disables the output.

Qt, not Tk: Tk in the same process as rclpy segfaulted libtk intermittently
on this machine whichever thread spun ROS. rclpy is spun from a QTimer.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QKeySequence, QPainter, QPen
from PyQt5.QtWidgets import (QApplication, QGridLayout, QHBoxLayout, QLabel, QLayout, QLineEdit,
                             QPushButton, QShortcut, QVBoxLayout, QWidget, QFrame, QCheckBox)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from drag_teleop import Chain, hand_tool_point
from robot_view import RobotModel, RobotView, joint_label

STALE_AFTER = 1.0     # [s] status older than this = bridge not answering
SLOW_REPLY = 0.2      # [s] a motor quieter than this is shown as lagging

UI_SCALE = 1.6   # panel text/padding multiplier; follows the window size
BASE_SIZE = (1250, 760)   # panel size at UI_SCALE 1.0 (both arms) -- the layout reference


def px(pixels: float) -> int:
    return max(1, round(pixels * UI_SCALE))


def font(pixels: int, bold: bool = False, family: str = "Sans") -> QFont:
    """Fonts in pixels: this display's DPI blows point sizes up ~3.7x while
    style-sheet px stay true, so everything is sized in px to stay consistent."""
    f = QFont(family)
    f.setPixelSize(px(pixels))
    f.setBold(bold)
    return f


C = {"ok": "#1f9d55", "idle": "#d97706", "bad": "#dc2626", "off": "#6b7280", "text": "#111827"}


class PanelNode(Node):
    def __init__(self, bridge: str) -> None:
        super().__init__("arm_panel")
        self.bridge = bridge
        self.status: dict | None = None
        self.status_time = 0.0
        self.last_message = ""
        self.create_subscription(String, f"{bridge}/status", self._on_status, 10)
        # DexHand finger angles (joint_state_publisher merges the hand drivers in),
        # only so the 3D window can draw the fingers.
        self.hand_q: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.limits_pub = self.create_publisher(String, f"{bridge}/set_limits", 10)
        self.command_pub = self.create_publisher(JointState, f"{bridge}/joint_command", 10)
        self.arms_pub = self.create_publisher(String, f"{bridge}/set_arms", 10)
        self.bridge_services = {
            "enable": self.create_client(SetBool, f"{bridge}/enable"),
            "estop": self.create_client(Trigger, f"{bridge}/estop"),
            "reset_estop": self.create_client(Trigger, f"{bridge}/reset_estop"),
        }

    def _on_status(self, message: String) -> None:
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.status_time = time.monotonic()

    def _on_joint_states(self, message: JointState) -> None:
        for name, position in zip(message.name, message.position):
            if not name.startswith("arm_"):
                self.hand_q[name] = position

    def snapshot(self) -> tuple[dict | None, float]:
        return self.status, time.monotonic() - self.status_time

    def call(self, which: str, value: bool | None = None) -> bool:
        client = self.bridge_services[which]
        if not client.service_is_ready():
            self.last_message = f"{client.srv_name} 응답 없음 (브리지가 떠 있나요?)"
            return False
        request = SetBool.Request(data=value) if value is not None else Trigger.Request()
        future = client.call_async(request)

        def done(f):
            result = f.result()
            self.last_message = translate(result.message) if result else f"{client.srv_name}: 응답 없음"
        future.add_done_callback(done)
        return True

    def send_command(self, positions: dict[str, float]) -> None:
        """Stream joint goals from the 3D window, same topic as drag_teleop.py."""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(positions)
        message.position = [float(v) for v in positions.values()]
        self.command_pub.publish(message)

    def send_arms(self, arms: dict[str, bool]) -> None:
        self.arms_pub.publish(String(data=json.dumps(arms)))

    def send_limits(self, limits: dict) -> None:
        self.limits_pub.publish(String(data=json.dumps(limits)))


BRIDGE_MESSAGES = (
    ("output enabled (live)", "토크 켜짐 (실제 모터)"),
    ("output enabled (dry run)", "토크 켜짐 (시험 모드)"),
    ("output disabled", "토크 꺼짐"),
    ("E-STOP latched; clear it before enabling", "비상정지 상태입니다. '비상정지 해제' 후 토크를 켜세요"),
    ("E-STOP: all motors idle, output latched off", "비상정지: 모든 모터 토크 해제, 해제 전까지 잠김"),
    ("E-STOP cleared; output is still disabled", "비상정지 해제됨 (토크는 아직 꺼져 있음)"),
    ("no E-STOP was latched", "걸려 있는 비상정지가 없습니다"),
    ("disable the output first; zeroing a live joint would jump it",
     "먼저 토크를 끄세요. 토크가 켜진 채 0점을 바꾸면 관절이 튑니다"),
    ("dry run: there is nothing to zero", "시험 모드에서는 0점을 잡을 모터가 없습니다"),
)


def translate(message: str) -> str:
    """The bridge answers in English; show the operator Korean."""
    for english, korean in BRIDGE_MESSAGES:
        if message.startswith(english):
            return korean + message[len(english):]
    if message.startswith("applied in memory only"):
        return "현재 자세를 0°로 잡았습니다 (이번 실행에만 적용)\n" + message.split("\n", 1)[-1]
    return message


def can_state(channel: str) -> str:
    try:
        out = subprocess.run(["ip", "-details", "link", "show", channel],
                             capture_output=True, text=True, timeout=1.0).stdout
    except (OSError, subprocess.TimeoutExpired):
        return "?"
    if not out:
        return "없음"
    if "state DOWN" in out:
        return "DOWN"
    for token in ("ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE", "BUS-OFF", "STOPPED"):
        if token in out:
            return token
    return "UP"


def joint_color(status: dict, info: dict) -> str:
    if info.get("fault") or (status["live"] and info.get("enabled") and not info.get("seen")):
        return C["bad"]
    if info.get("live"):
        return C["ok"]
    if info.get("enabled"):
        return C["idle"]
    return C["off"]


class ArmWindow(QWidget):
    """Separate window holding the mesh view of the whole robot."""

    def __init__(self, view: RobotView) -> None:
        super().__init__()
        self.view = view
        self.setWindowTitle("BHL 로봇 자세 (3D)")
        self.resize(1100, 850)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(view)

    def update_status(self, status: dict | None) -> None:
        if self.isVisible():
            self.view.set_status(status, "" if status is not None else "브리지 상태 없음")


class Panel(QWidget):
    def __init__(self, node: PanelNode, limits_file: Path, view: ArmWindow,
                 arm_joints: list[str]) -> None:
        super().__init__()
        self.arm_joints = arm_joints     # only this arm's rows are shown
        self.node_bridge = node.bridge
        self.node = node
        self.limits_file = limits_file
        self.view = view
        self.rows: dict[str, dict] = {}
        self.bus_states: dict[str, str] = {}
        self._bus_poll = 0.0
        self._closing = False
        self._msg_color = C["text"]
        self.base_scale = UI_SCALE        # --font-scale: the scale at the reference size
        self.arrows: list[QLabel] = []
        self.header_labels: list[QLabel] = []

        self.setWindowTitle("BHL 팔 조작 패널")
        layout = QVBoxLayout(self)
        # Growing fonts must never push the window bigger (which would grow
        # the fonts again); the window size is the user's, the content follows.
        layout.setSizeConstraint(QLayout.SetNoConstraint)
        self._rescale_timer = QTimer(self)
        self._rescale_timer.setSingleShot(True)
        self._rescale_timer.timeout.connect(self._rescale)

        top = QHBoxLayout()
        self.estop_button = QPushButton("비상정지\n(Esc)")
        self.estop_button.clicked.connect(self.on_estop)
        top.addWidget(self.estop_button)
        info = QVBoxLayout()
        # System layout, left to right, each block coloured by its health.
        chain_row = QHBoxLayout()
        self.blocks = {}
        steps = (("rviz", "① 조작 화면\n(RViz·3D 창)"), ("ik", "② 드래그 계산\n(역기구학)"),
                 ("bridge", "③ 하드웨어 브리지\n(안전·속도 제한)"), ("can", "④ CAN 통신"),
                 ("motors", "⑤ 모터"))
        for index, (key, text) in enumerate(steps):
            if index:
                arrow = QLabel("→")
                self.arrows.append(arrow)
                chain_row.addWidget(arrow)
            block = QLabel(text)
            block.setAlignment(Qt.AlignCenter)
            block.setFrameShape(QFrame.StyledPanel)
            chain_row.addWidget(block)
            self.blocks[key] = block
        chain_row.addStretch()
        info.addLayout(chain_row)
        self.mode_label = QLabel("")
        info.addWidget(self.mode_label)
        buttons = QHBoxLayout()
        for text, slot in (("토크 켜기", lambda: self.node.call("enable", True)),
                           ("토크 끄기", lambda: self.node.call("enable", False)),
                           ("비상정지 해제", lambda: self.node.call("reset_estop")),
                           ("3D 창 열기", self.show_view)):
            button = QPushButton(text)
            button.clicked.connect(slot)
            button.setFocusPolicy(Qt.NoFocus)   # a stray key must not press a focused button
            buttons.addWidget(button)
        # Which arms move: an unticked arm is idled by the bridge and ignores
        # every drag, from RViz or the 3D window.
        self.arm_boxes: dict[str, QCheckBox] = {}
        for side, label in (("left", "왼팔 사용"), ("right", "오른팔 사용")):
            if not any(f"_{side}_" in n for n in self.arm_joints):
                continue
            box = QCheckBox(label)
            box.setChecked(True)
            box.setFocusPolicy(Qt.NoFocus)
            box.toggled.connect(lambda on, sd=side: self.node.send_arms({sd: on}))
            buttons.addWidget(box)
            self.arm_boxes[side] = box
        buttons.addStretch()
        info.addLayout(buttons)
        top.addLayout(info, 1)
        layout.addLayout(top)

        self.grid = QGridLayout()
        headers = ["모터 ID", "관절", "상태", "응답 시간", "현재 각도(°)", "목표 각도(°)",
                   "기구 한계(°)", "최소 제한(°)", "최대 제한(°)", ""]
        for column, text in enumerate(headers):
            label = QLabel(text)
            self.header_labels.append(label)
            self.grid.addWidget(label, 0, column)
        layout.addLayout(self.grid)

        bottom = QHBoxLayout()
        for text, slot in (("모든 제한 적용", self.apply_all), ("제한 저장", self.save_limits),
                           ("제한 해제 (URDF 범위로)", self.clear_limits)):
            button = QPushButton(text)
            button.clicked.connect(slot)
            button.setFocusPolicy(Qt.NoFocus)
            bottom.addWidget(button)
        bottom.addStretch()
        layout.addLayout(bottom)
        self.message = QLabel("")
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        layout.addStretch()

        # E-STOP from either window: Esc only. Space was dropped after stray
        # typing into a focused panel latched the E-STOP three times -- on the
        # real arm that is a sudden sag, not a harmless no-op.
        for widget in (self, self.view):
            shortcut = QShortcut(QKeySequence("Escape"), widget)
            shortcut.setContext(Qt.WindowShortcut)
            shortcut.activated.connect(self.on_estop)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.restyle()

    # ---------------------------------------------------------------- scaling

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._rescale_timer.start(120)    # restyle once the drag settles

    def _rescale(self) -> None:
        global UI_SCALE
        fit = min(self.width() / (BASE_SIZE[0] * self.base_scale),
                  self.height() / (BASE_SIZE[1] * self.base_scale))
        scale = min(max(self.base_scale * fit, 0.7), 5.0)
        if abs(scale - UI_SCALE) / UI_SCALE < 0.03:
            return
        UI_SCALE = scale
        self.restyle()

    def restyle(self) -> None:
        """Re-apply every size that depends on UI_SCALE."""
        QApplication.instance().setFont(font(15))
        self.estop_button.setStyleSheet(
            f"QPushButton {{ background: #dc2626; color: white; font-size: {px(26)}px; font-weight: bold;"
            f" padding: {px(22)}px {px(34)}px; border-radius: {px(10)}px; border: {px(4)}px solid #7f1d1d; }}"
            "QPushButton:pressed { background: #991b1b; }")
        for arrow in self.arrows:
            arrow.setStyleSheet(f"font-size: {px(22)}px; color: #6b7280;")
        for label in self.header_labels:
            label.setFont(font(15, bold=True))
        self.mode_label.setFont(font(17, bold=True))
        self.message.setStyleSheet(f"color: {self._msg_color}; font-size: {px(16)}px;")
        for row in self.rows.values():
            self._style_row(row)
        self.refresh()

    def show_view(self) -> None:
        self.view.show()
        self.view.raise_()

    def say(self, text: str, color: str = C["text"]) -> None:
        self._msg_color = color
        self.message.setText(text)
        self.message.setStyleSheet(f"color: {color}; font-size: {px(16)}px;")

    # ------------------------------------------------------------------ table

    def _make_row(self, name: str, info: dict) -> dict:
        r = len(self.rows) + 1
        row = {}
        row["id"] = QLabel(str(info["device_id"]))
        row["name"] = QLabel(joint_label(name))
        row["link"] = QLabel("")
        row["link"].setAlignment(Qt.AlignCenter)
        row["age"] = QLabel("")
        row["meas"] = QLabel("")
        row["cmd"] = QLabel("")
        row["urdf"] = QLabel("")
        row["lo"] = QLineEdit()
        row["hi"] = QLineEdit()
        for edit in (row["lo"], row["hi"]):
            edit.returnPressed.connect(lambda n=name: self.apply([n]))
        row["apply"] = QPushButton("적용")
        row["apply"].setFocusPolicy(Qt.NoFocus)
        row["apply"].clicked.connect(lambda _c=False, n=name: self.apply([n]))
        for column, key in enumerate(("id", "name", "link", "age", "meas", "cmd", "urdf", "lo", "hi", "apply")):
            self.grid.addWidget(row[key], r, column)
        self._style_row(row)
        lower, upper = info["limits"]
        row["lo"].setText(f"{math.degrees(lower):.1f}")
        row["hi"].setText(f"{math.degrees(upper):.1f}")
        return row

    def _style_row(self, row: dict) -> None:
        metrics = self.fontMetrics()
        row["link"].setMinimumWidth(metrics.horizontalAdvance("연결됨·토크 꺼짐") + px(24))
        for key in ("lo", "hi"):
            row[key].setFixedWidth(metrics.horizontalAdvance("-180.00") + px(16))
        for key in ("age", "meas", "cmd"):
            row[key].setFont(font(15, family="Monospace"))
            row[key].setMinimumWidth(metrics.horizontalAdvance("-180.0 ms") + px(8))

    def _parse(self, name: str) -> tuple[float, float] | None:
        row = self.rows[name]
        try:
            return float(row["lo"].text()), float(row["hi"].text())
        except ValueError:
            self.say(f"{joint_label(name)}: 최소/최대는 숫자(도)여야 합니다", C["bad"])
            return None

    def apply(self, names: list[str]) -> None:
        status, _age = self.node.snapshot()
        if status is None:
            self.say("브리지 상태를 받기 전에는 적용할 수 없습니다", C["bad"])
            return
        limits, warnings = {}, []
        for name in names:
            parsed = self._parse(name)
            if parsed is None:
                return
            lo, hi = parsed
            urdf = [math.degrees(v) for v in status["joints"][name]["urdf"]]
            if not (urdf[0] - 1e-6 <= lo < hi <= urdf[1] + 1e-6):
                self.say(f"{joint_label(name)}: {lo:.1f}~{hi:.1f}° 는 기구 한계 {urdf[0]:.1f}~{urdf[1]:.1f}° "
                         "안이어야 하고 최소가 최대보다 작아야 합니다", C["bad"])
                return
            limits[name] = [lo, hi]
            meas = math.degrees(status["joints"][name]["measured"])
            if status["joints"][name]["live"] and not lo <= meas <= hi:
                warnings.append(f"{joint_label(name)}: 현재 {meas:.1f}° 가 새 범위 밖이라 범위 안으로 천천히 움직입니다")
        self.node.send_limits(limits)
        if warnings:
            self.say("\n".join(warnings), C["idle"])
        else:
            self.say(f"제한 적용: {', '.join(joint_label(n) for n in names)}", C["ok"])

    def apply_all(self) -> None:
        self.apply([n for n, r in self.rows.items() if r["lo"].isEnabled()])

    def clear_limits(self) -> None:
        self.node.send_limits({name: [None, None] for name in self.rows})
        for row in self.rows.values():
            row["reset"] = True
        self.say("모든 관절 제한을 URDF 범위로 되돌렸습니다 (저장하려면 '제한 저장')", C["ok"])

    def save_limits(self) -> None:
        status, _age = self.node.snapshot()
        if status is None:
            self.say("브리지 상태를 받기 전에는 저장할 수 없습니다", C["bad"])
            return
        data = {}
        for name, info in status["joints"].items():
            user = info["user"]
            if user[0] is None and user[1] is None:
                continue
            data[name] = [None if v is None else round(math.degrees(v), 2) for v in user]
        self.limits_file.parent.mkdir(parents=True, exist_ok=True)
        header = ("# Operator travel limits [deg] per joint, written by arm_panel.py and\n"
                  "# loaded by hardware_bridge.py --limits-file. null = URDF limit.\n")
        self.limits_file.write_text(header + yaml.safe_dump(data, sort_keys=True))
        self.say(f"저장됨: {self.limits_file} ({len(data)}개 관절)", C["ok"])

    # ---------------------------------------------------------------- refresh

    def refresh(self) -> None:
        status, age = self.node.snapshot()
        now = time.monotonic()
        if status is not None and now - self._bus_poll > 1.0:
            self._bus_poll = now
            # Controller state (ERROR-PASSIVE, BUS-OFF...) of the channels the
            # bridge has open; whether they exist at all comes from the bridge.
            self.bus_states = {ch: can_state(ch) for ch, st in (status.get("channels") or {}).items()
                               if st == "open"}

        fresh = status is not None and age <= STALE_AFTER
        self.view.view.extra_q = self.node.hand_q
        self.view.update_status(status if fresh else None)
        self._update_blocks(status if fresh else None)
        if not fresh:
            self.mode_label.setText("브리지 응답 없음 — 하드웨어 브리지가 꺼졌거나 아직 켜지는 중입니다")
            self.mode_label.setStyleSheet(f"color: {C['bad']};")
        else:
            parts = ["실제 모터 구동 모드" if status["live"] else "시험 모드 (모터에 명령을 보내지 않음)"]
            if status["estop"]:
                parts.append("비상정지 상태 — '비상정지 해제' 후 '토크 켜기'")
                color = C["bad"]
            elif status["output_enabled"]:
                parts.append("토크 켜짐 — RViz 에서 손 마커를 끌면 팔이 따라 움직입니다")
                color = C["ok"]
            else:
                parts.append("토크 꺼짐 — '토크 켜기'를 눌러야 움직입니다")
                color = C["text"]
            self.mode_label.setText("   |   ".join(parts))
            self.mode_label.setStyleSheet(f"color: {color};")
            for side, box in self.arm_boxes.items():
                active = (status.get("arms") or {}).get(side, True)
                if box.isChecked() != active:
                    box.blockSignals(True)
                    box.setChecked(active)
                    box.blockSignals(False)
            for name, info in status["joints"].items():
                if name not in self.arm_joints:
                    continue
                if name not in self.rows:
                    self.rows[name] = self._make_row(name, info)
                self._update_row(status, name, info)

        if self.node.last_message:
            self.say(self.node.last_message)
            self.node.last_message = ""

    def _set_block(self, key: str, text: str, color: str) -> None:
        block = self.blocks[key]
        title = block.text().split("\n")[0]
        block.setText(f"{title}\n{text}")
        block.setStyleSheet(f"background: {color}; color: white; font-weight: bold;"
                            f" padding: {px(6)}px {px(10)}px; border-radius: 6px;")

    def _update_blocks(self, status: dict | None) -> None:
        # The 3D window is this process, so ① is up whenever RViz or it is.
        rviz = any(name.startswith("/drag_teleop") and name.endswith("/update")
                   and self.node.count_subscribers(name) > 0
                   for name, _types in self.node.get_topic_names_and_types())
        self._set_block("rviz", "RViz + 3D 창" if rviz else "3D 창만 (RViz 꺼짐)",
                        C["ok"] if rviz else C["idle"])
        # IK runs in drag_teleop.py for RViz and in this panel for the 3D window.
        self._set_block("ik", "켜짐", C["ok"])
        if status is None:
            self._set_block("bridge", "응답 없음", C["bad"])
            self._set_block("can", "알 수 없음", C["off"])
            self._set_block("motors", "알 수 없음", C["off"])
            return
        if status["estop"]:
            self._set_block("bridge", "비상정지", C["bad"])
        else:
            self._set_block("bridge", "토크 켜짐" if status["output_enabled"] else "토크 꺼짐",
                            C["ok"] if status["output_enabled"] else C["idle"])
        channels = status.get("channels") or {}
        if not status["live"]:
            seen = "  ".join(f"{ch} {'감지됨' if st != 'missing' else '없음'}"
                             for ch, st in channels.items())
            self._set_block("can", "시험 모드(안 씀)" + (f"\n{seen}" if seen else ""), C["off"])
        else:
            words = {"missing": "어댑터 없음", "down": "링크 꺼짐", "up": "여는 중"}
            parts, bad = [], not channels
            for ch, st in channels.items():
                if st == "open":
                    v = self.bus_states.get(ch, "UP")
                    ok = v in ("ERROR-ACTIVE", "UP")
                    parts.append(f"{ch}: {'정상' if ok else v}")
                else:
                    ok = False
                    parts.append(f"{ch}: {words.get(st, st)}")
                bad = bad or not ok
            self._set_block("can", "  ".join(parts) or "CAN 없음", C["bad"] if bad else C["ok"])
        mine = [status["joints"][n] for n in self.arm_joints if n in status["joints"]]
        usable = [j for j in mine if j["enabled"]]
        answering = [j for j in usable if j["seen"] and not j["fault"]]
        if not status["live"]:
            self._set_block("motors", f"사용 {len(usable)}/{len(mine)}개 (시험)", C["off"])
        else:
            ok = len(answering) == len(usable) and usable
            self._set_block("motors", f"응답 {len(answering)}/{len(usable)}개",
                            C["ok"] if ok else C["bad"])

    def _update_row(self, status: dict, name: str, info: dict) -> None:
        row = self.rows[name]
        faults = {"LOST": "통신 끊김", "OUT_OF_RANGE": "범위 밖(영점 확인)", "CAN_LOST": "CAN 끊김"}
        no_bus = status["live"] and info.get("configured_enabled") and not info.get("bus_open", True)
        if no_bus:
            text = "CAN 없음"
        elif not info["enabled"]:
            text = "사용 안 함"
        elif not info.get("arm_active", True):
            text = "팔 선택 해제"
        elif not status["live"]:
            text = "시험 구동" if info["live"] else "시험 대기"
        elif info["fault"]:
            text = faults.get(info["fault"], info["fault"])
        elif not info["seen"]:
            text = "응답 없음"
        elif info["live"]:
            text = "구동 중"
        else:
            text = "연결됨·토크 꺼짐"
        color = joint_color(status, info) if status["live"] or not info["enabled"] else C["off"]
        if no_bus:
            color = C["bad"]
        age = info["age"]
        if status["live"] and info["seen"] and age is not None and age > 1.0:
            text, color = "끊김", C["bad"]
        row["link"].setText(text)
        row["link"].setStyleSheet(f"background: {color}; color: white; font-weight: bold;"
                                  f" padding: {px(2)}px {px(6)}px;")
        if age is None:
            row["age"].setText("-")
            row["age"].setStyleSheet("")
        else:
            row["age"].setText(f"{age * 1000:.0f} ms")
            row["age"].setStyleSheet(f"color: {C['bad'] if age > SLOW_REPLY else C['text']};")
        row["meas"].setText(f"{math.degrees(info['measured']):+.1f}")
        row["cmd"].setText(f"{math.degrees(info['command']):+.1f}" if info["live"] else "-")
        lo, hi = (math.degrees(v) for v in info["urdf"])
        row["urdf"].setText(f"{lo:.0f} ~ {hi:.0f}")
        editable = info["enabled"]
        row["lo"].setEnabled(editable)
        row["hi"].setEnabled(editable)
        row["apply"].setEnabled(editable)
        if row.pop("reset", False):
            row["lo"].setText(f"{lo:.1f}")
            row["hi"].setText(f"{hi:.1f}")

    # ----------------------------------------------------------------- safety

    def on_estop(self) -> None:
        if not self.node.call("estop"):
            self.say("비상정지를 보낼 수 없습니다: 브리지 응답 없음. 모터 전원을 직접 끊으세요!", C["bad"])
        else:
            self.say("비상정지 보냄 — 모든 모터 토크 해제", C["bad"])

    def closeEvent(self, event) -> None:
        if not self._closing:
            self._closing = True
            self.node.call("enable", False)
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            self.view.close()
        event.accept()


def main() -> None:
    global UI_SCALE
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", default="/hardware_bridge")
    parser.add_argument("--limits-file", type=Path, required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--base", default="base")
    parser.add_argument("--tips", nargs="+", default=["arm_left_hand_link"],
                        help="hand link of every arm to show and drag")
    parser.add_argument("--font-scale", type=float, default=1.6,
                        help="text scale at the default window size (1.0 = the original small size); "
                             "resizing the window scales from there")
    args, _ = parser.parse_known_args()
    UI_SCALE = args.font_scale

    rclpy.init()
    node = PanelNode(args.bridge)
    app = QApplication(sys.argv[:1])
    app.setFont(font(15))
    urdf = Path(args.urdf)
    arms = []
    for tip in args.tips:
        chain = Chain(urdf.read_text(), args.base, tip)
        # Grab each hand inside its gripper, not at the link origin (which is
        # the elbow-roll axis); computed from the hand mesh, so both sides work.
        chain.tool = hand_tool_point(urdf.read_text(), tip, urdf.parent.parent / "meshes")
        arms.append((chain, tip))
    model = RobotModel(urdf, urdf.parent.parent / "meshes")
    view = ArmWindow(RobotView(model, arms, on_command=node.send_command))
    panel = Panel(node, args.limits_file, view, [n for chain, _tip in arms for n in chain.names])
    panel.resize(px(BASE_SIZE[0]), px(BASE_SIZE[1]))
    panel.show()
    view.show()

    spinner = QTimer()
    spinner.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spinner.start(10)
    app.lastWindowClosed.connect(app.quit)
    # Ctrl+C / launch shutdown: close like the window's X (torque off first).
    # The spinner timer returns to Python every 10 ms, so the handler runs.
    signal.signal(signal.SIGINT, lambda *_a: panel.close())
    signal.signal(signal.SIGTERM, lambda *_a: panel.close())
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
