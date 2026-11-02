#!/usr/bin/env python3
"""DexHand finger sliders for drag teleop.

One group per hand: an output switch, a curl (Pitch) and a spread (Yaw) slider
for each of the four fingers, and open / fist / centre buttons. The thumb is a
fixed joint in the URDF, so there is nothing to drive.

Targets go to dexhand_bringup's hand_driver as sensor_msgs/JointState on
`<driver>/joint_command`, re-sent at 20 Hz (the driver slew-limits them and
keeps its own 50 Hz stream to the board). Output is switched with the driver's
`<driver>/enable` service; it starts off.

The left hand runs the same driver in the /left_hand namespace and still uses
R_ joint names internally (left_hand_adapter.py renames its states to L_), so
both hands are commanded with R_ names.

    finger_panel.py --urdf <urdf> --hands right left
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

FINGERS = [("Index", "검지"), ("Middle", "중지"), ("Ring", "약지"), ("Pinky", "새끼")]
HANDS = {
    # side: (label, driver node path, URDF joint prefix used for the limits)
    "right": ("오른손", "/dexhand_driver", "R_"),
    "left": ("왼손", "/left_hand/dexhand_driver", "L_"),
}
STEPS = 1000          # slider resolution over the joint range
SEND_HZ = 20.0


def joint_limits(urdf: Path) -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in ET.parse(urdf).getroot().findall("joint"):
        limit = joint.find("limit")
        if joint.get("type") == "revolute" and limit is not None:
            limits[joint.get("name")] = (float(limit.get("lower", 0.0)), float(limit.get("upper", 0.0)))
    return limits


class FingerNode(Node):
    def __init__(self, hands: list[str]) -> None:
        super().__init__("finger_panel")
        self.publishers_ = {side: self.create_publisher(JointState, f"{HANDS[side][1]}/joint_command", 10)
                            for side in hands}
        self.enable_clients = {side: self.create_client(SetBool, f"{HANDS[side][1]}/enable")
                               for side in hands}

    def send(self, side: str, targets: dict[str, float]) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(targets)
        message.position = [float(v) for v in targets.values()]
        self.publishers_[side].publish(message)

    def enable(self, side: str, on: bool, done) -> bool:
        client = self.enable_clients[side]
        if not client.service_is_ready():
            return False
        future = client.call_async(SetBool.Request(data=on))
        future.add_done_callback(lambda f: done(f.result()))
        return True


class HandBox(QGroupBox):
    """Sliders for one hand. Values are kept in R_ names (what the driver takes)."""

    def __init__(self, node: FingerNode, side: str, limits: dict[str, tuple[float, float]]) -> None:
        label, _driver, prefix = HANDS[side]
        super().__init__(label)
        self.node = node
        self.side = side
        self.sliders: dict[str, tuple[QSlider, QLabel, float, float]] = {}

        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        self.output = QCheckBox("출력 켜기")
        self.output.setFocusPolicy(Qt.NoFocus)
        self.output.toggled.connect(self.on_output)
        top.addWidget(self.output)
        self.state = QLabel("출력 꺼짐")
        top.addWidget(self.state, 1)
        layout.addLayout(top)

        grid = QGridLayout()
        grid.addWidget(QLabel("손가락"), 0, 0)
        grid.addWidget(QLabel("굽힘 (Pitch)"), 0, 1, 1, 2)
        grid.addWidget(QLabel("벌림 (Yaw)"), 0, 3, 1, 2)
        for row, (finger, finger_label) in enumerate(FINGERS, start=1):
            grid.addWidget(QLabel(finger_label), row, 0)
            for col, kind in ((1, "Pitch"), (3, "Yaw")):
                urdf_name = f"{prefix}{finger}_{kind}"
                lower, upper = limits.get(urdf_name, (0.0, 0.95) if kind == "Pitch" else (-0.3, 0.3))
                slider = QSlider(Qt.Horizontal)
                slider.setRange(0, STEPS)
                slider.setFocusPolicy(Qt.NoFocus)
                value = QLabel()
                value.setMinimumWidth(60)
                name = f"R_{finger}_{kind}"
                self.sliders[name] = (slider, value, lower, upper)
                slider.valueChanged.connect(lambda _v, n=name: self._show(n))
                self.set(name, 0.0)
                grid.addWidget(slider, row, col)
                grid.addWidget(value, row, col + 1)
        layout.addLayout(grid)

        buttons = QHBoxLayout()
        for text, slot in (("펴기", self.open_hand), ("주먹 쥐기", self.fist), ("벌림 가운데", self.centre_spread)):
            button = QPushButton(text)
            button.setFocusPolicy(Qt.NoFocus)
            button.clicked.connect(slot)
            buttons.addWidget(button)
        layout.addLayout(buttons)

    # -- values ---------------------------------------------------------------
    def get(self, name: str) -> float:
        slider, _value, lower, upper = self.sliders[name]
        return lower + (upper - lower) * slider.value() / STEPS

    def set(self, name: str, position: float) -> None:
        slider, _value, lower, upper = self.sliders[name]
        position = min(max(position, lower), upper)
        slider.setValue(round((position - lower) / (upper - lower) * STEPS) if upper > lower else 0)
        self._show(name)

    def _show(self, name: str) -> None:
        self.sliders[name][1].setText(f"{math.degrees(self.get(name)):+5.1f}°")

    def targets(self) -> dict[str, float]:
        return {name: self.get(name) for name in self.sliders}

    # -- buttons --------------------------------------------------------------
    def open_hand(self) -> None:
        for finger, _ in FINGERS:
            self.set(f"R_{finger}_Pitch", 0.0)

    def fist(self) -> None:
        for finger, _ in FINGERS:
            self.set(f"R_{finger}_Pitch", self.sliders[f"R_{finger}_Pitch"][3])

    def centre_spread(self) -> None:
        for finger, _ in FINGERS:
            self.set(f"R_{finger}_Yaw", 0.0)

    def on_output(self, on: bool) -> None:
        def done(result) -> None:
            ok = result is not None and result.success
            text = ("출력 켜짐" if on else "출력 꺼짐") if ok else f"실패: {getattr(result, 'message', '응답 없음')}"
            self.state.setText(text)
        if not self.node.enable(self.side, on, done):
            self.state.setText("드라이버 응답 없음 (hand_driver 가 떠 있나요?)")
            self.output.blockSignals(True)
            self.output.setChecked(False)
            self.output.blockSignals(False)


class Panel(QWidget):
    def __init__(self, node: FingerNode, hands: list[str], limits: dict) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("DexHand 손가락")
        layout = QHBoxLayout(self)
        self.boxes = [HandBox(node, side, limits) for side in hands]
        for box in self.boxes:
            layout.addWidget(box)
        self.timer = QTimer()
        self.timer.timeout.connect(self.send)
        self.timer.start(int(1000 / SEND_HZ))

    def send(self) -> None:
        for box in self.boxes:
            self.node.send(box.side, box.targets())

    def closeEvent(self, event) -> None:
        # Leave the hands powered down rather than holding the last pose.
        for box in self.boxes:
            if box.output.isChecked():
                self.node.enable(box.side, False, lambda _r: None)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        event.accept()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--urdf", required=True, help="robot URDF, for the finger limits")
    parser.add_argument("--hands", nargs="+", choices=list(HANDS), default=["right"])
    args, _ros = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = FingerNode(args.hands)
    app = QApplication(sys.argv[:1])
    panel = Panel(node, args.hands, joint_limits(Path(args.urdf)))
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
