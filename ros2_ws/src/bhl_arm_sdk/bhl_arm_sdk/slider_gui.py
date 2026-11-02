"""Slider panel for arm_controller: one slider per motor, in degrees.

Dragging a slider publishes that joint's goal to `/arm_controller/target`; the
controller slews to it at its max speed. Each row also shows the measured
angle and the joint's state (live / fault). Buttons: enable, disable, E-STOP,
reset E-STOP, home (all 0 deg). A speed slider sets `max_speed_deg`.

Sliders only send while the user moves them, and they snap to the measured /
goal angle whenever the controller changes the goal itself (enable, home,
`ros2 param set`), so the panel never fights the parameter interface.

    ros2 run bhl_arm_sdk slider_gui
"""

from __future__ import annotations

import json
import math
import signal
import sys

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                             QPushButton, QSlider, QVBoxLayout, QWidget)
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

STEPS_PER_DEG = 10        # slider resolution 0.1 deg
LABELS = {
    "shoulder_pitch": "어깨 앞뒤(피치)", "shoulder_roll": "어깨 옆벌림(롤)",
    "shoulder_yaw": "어깨 비틀기(요)", "elbow_pitch": "팔꿈치 굽힘(피치)",
    "elbow_roll": "팔꿈치 비틀기(롤)",
}


def label_of(name: str) -> str:
    side = "왼팔" if "_left_" in name else "오른팔" if "_right_" in name else ""
    for key, text in LABELS.items():
        if key in name:
            return f"{side} {text}"
    return name


class GuiNode(Node):
    def __init__(self, controller: str) -> None:
        super().__init__("arm_slider_gui")
        self.controller = controller
        self.status: dict | None = None
        self.create_subscription(String, f"{controller}/status", self._on_status, 10)
        self.target_pub = self.create_publisher(JointState, f"{controller}/target", 10)
        self.enable_client = self.create_client(SetBool, f"{controller}/enable")
        self.triggers = {name: self.create_client(Trigger, f"{controller}/{name}")
                         for name in ("estop", "reset_estop", "home")}
        self.param_client = self.create_client(SetParameters, f"{controller}/set_parameters")
        self.last_reply = ""

    def _on_status(self, message: String) -> None:
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def send_goal(self, name: str, degrees: float) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [name]
        message.position = [math.radians(degrees)]
        self.target_pub.publish(message)

    def _done(self, future) -> None:
        result = future.result()
        self.last_reply = getattr(result, "message", "") if result is not None else "응답 없음"

    def enable(self, on: bool) -> None:
        if not self.enable_client.service_is_ready():
            self.last_reply = "arm_controller 응답 없음 (노드가 떠 있나요?)"
            return
        self.enable_client.call_async(SetBool.Request(data=on)).add_done_callback(self._done)

    def trigger(self, name: str) -> None:
        client = self.triggers[name]
        if not client.service_is_ready():
            self.last_reply = "arm_controller 응답 없음"
            return
        client.call_async(Trigger.Request()).add_done_callback(self._done)

    def set_speed(self, degrees_per_second: float) -> None:
        if not self.param_client.service_is_ready():
            return
        value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(degrees_per_second))
        request = SetParameters.Request(parameters=[ParameterMsg(name="max_speed_deg", value=value)])
        self.param_client.call_async(request)


class Row:
    def __init__(self, grid: QGridLayout, row: int, name: str, lower: float, upper: float, node: GuiNode) -> None:
        self.name, self.node = name, node
        self.dragging = False
        grid.addWidget(QLabel(label_of(name)), row, 0)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(round(lower * STEPS_PER_DEG), round(upper * STEPS_PER_DEG))
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.slider.setMinimumWidth(320)
        self.slider.sliderPressed.connect(lambda: setattr(self, "dragging", True))
        self.slider.sliderReleased.connect(self._released)
        self.slider.valueChanged.connect(self._changed)
        grid.addWidget(QLabel(f"{lower:+.0f}°"), row, 1)
        grid.addWidget(self.slider, row, 2)
        grid.addWidget(QLabel(f"{upper:+.0f}°"), row, 3)
        self.goal = QLabel("")
        self.goal.setMinimumWidth(80)
        grid.addWidget(self.goal, row, 4)
        self.measured = QLabel("")
        self.measured.setMinimumWidth(80)
        grid.addWidget(self.measured, row, 5)
        self.state = QLabel("")
        self.state.setMinimumWidth(110)
        grid.addWidget(self.state, row, 6)
        self.last_goal = None

    def _changed(self, value: int) -> None:
        degrees = value / STEPS_PER_DEG
        self.goal.setText(f"목표 {degrees:+6.1f}°")
        if self.dragging:
            self.node.send_goal(self.name, degrees)

    def _released(self) -> None:
        self.node.send_goal(self.name, self.slider.value() / STEPS_PER_DEG)
        self.dragging = False

    def update(self, info: dict) -> None:
        self.measured.setText(f"측정 {info['measured_deg']:+6.1f}°")
        if info["fault"]:
            text, color = f"오류 {info['fault']}", "#c0392b"
        elif info["live"]:
            text, color = "구동 중", "#1e8449"
        elif not info["enabled"]:
            text, color = "사용 안 함", "#7f8c8d"
        else:
            text, color = "토크 꺼짐", "#ca6f1e"
        self.state.setText(text)
        self.state.setStyleSheet(f"color: {color};")
        goal = info["goal_deg"]
        if not self.dragging and goal != self.last_goal:
            self.slider.blockSignals(True)
            self.slider.setValue(round(goal * STEPS_PER_DEG))
            self.slider.blockSignals(False)
            self.goal.setText(f"목표 {goal:+6.1f}°")
        self.last_goal = goal
        self.slider.setEnabled(info["enabled"])


class Panel(QWidget):
    def __init__(self, node: GuiNode) -> None:
        super().__init__()
        self.node = node
        self.rows: dict[str, Row] = {}
        self.setWindowTitle("BHL 모터 각도 제어")
        layout = QVBoxLayout(self)
        self.header = QLabel("arm_controller 를 기다리는 중...")
        layout.addWidget(self.header)

        buttons = QHBoxLayout()
        for text, slot in (("토크 켜기", lambda: node.enable(True)), ("토크 끄기", lambda: node.enable(False)),
                           ("비상정지", lambda: node.trigger("estop")),
                           ("비상정지 해제", lambda: node.trigger("reset_estop")),
                           ("모두 0°", lambda: node.trigger("home"))):
            button = QPushButton(text)
            button.setFocusPolicy(Qt.NoFocus)
            button.clicked.connect(slot)
            if text == "비상정지":
                button.setStyleSheet("background: #c0392b; color: white; font-weight: bold;")
            buttons.addWidget(button)
        layout.addLayout(buttons)

        speed = QHBoxLayout()
        speed.addWidget(QLabel("최대 속도"))
        self.speed = QSlider(Qt.Horizontal)
        self.speed.setRange(5, 90)
        self.speed.setValue(30)
        self.speed.setFocusPolicy(Qt.NoFocus)
        self.speed_label = QLabel("30°/s")
        self.speed.valueChanged.connect(lambda v: self.speed_label.setText(f"{v}°/s"))
        self.speed.sliderReleased.connect(lambda: node.set_speed(self.speed.value()))
        speed.addWidget(self.speed, 1)
        speed.addWidget(self.speed_label)
        layout.addLayout(speed)

        self.groups = QHBoxLayout()
        layout.addLayout(self.groups)
        self.reply = QLabel("")
        layout.addWidget(self.reply)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def _build(self, status: dict) -> None:
        sides = {"왼팔": [], "오른팔": [], "기타": []}
        for name in status["joints"]:
            key = "왼팔" if "_left_" in name else "오른팔" if "_right_" in name else "기타"
            sides[key].append(name)
        for side, names in sides.items():
            if not names:
                continue
            box = QGroupBox(side)
            grid = QGridLayout(box)
            for i, name in enumerate(names):
                lower, upper = status["joints"][name]["limits_deg"]
                self.rows[name] = Row(grid, i, name, lower, upper, self.node)
            self.groups.addWidget(box)
        if not status.get("max_speed_deg") is None:
            self.speed.setValue(round(status["max_speed_deg"]))

    def refresh(self) -> None:
        status = self.node.status
        if status is None:
            return
        if not self.rows:
            self._build(status)
        live = sum(1 for j in status["joints"].values() if j["live"])
        mode = "연습 모드 (CAN 안 열림)" if status["dry_run"] else "실물 모드"
        estop = "  |  비상정지 걸림" if status["estop"] else ""
        self.header.setText(f"{mode}  |  구동 중 {live}/{len(status['joints'])}{estop}  |  {status['message']}")
        self.header.setStyleSheet("color: #c0392b; font-weight: bold;" if status["estop"] else "")
        for name, info in status["joints"].items():
            if name in self.rows:
                self.rows[name].update(info)
        self.reply.setText(self.node.last_reply)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = GuiNode("/arm_controller")
    app = QApplication(sys.argv[:1])
    panel = Panel(node)
    panel.show()
    spinner = QTimer()
    spinner.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spinner.start(10)
    signal.signal(signal.SIGINT, lambda *_a: panel.close())
    signal.signal(signal.SIGTERM, lambda *_a: panel.close())
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
