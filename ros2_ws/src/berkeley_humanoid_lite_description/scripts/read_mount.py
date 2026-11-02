#!/usr/bin/env python3
"""Read the six mount sliders back as a dexhand_mount.yaml block.

Pair with the tuning model:

    ros2 launch berkeley_humanoid_lite_description display.launch.py model:=tuning
    ros2 run berkeley_humanoid_lite_description read_mount.py

Drag the dexhand_mount_* sliders until the hand sits right, then read the
printed block and paste it over the `mount:` section of
config/dexhand_mount.yaml.  Re-run generate_urdf.py and rebuild to bake it in.

The slider values are the URDF origin directly -- the tuning model orders the
joints x, y, z, yaw, pitch, roll precisely so that no conversion is needed here.

By default this prints once and exits; --watch keeps printing as you drag.
"""

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [
    "dexhand_mount_x",
    "dexhand_mount_y",
    "dexhand_mount_z",
    "dexhand_mount_yaw",
    "dexhand_mount_pitch",
    "dexhand_mount_roll",
]


def format_block(values: dict[str, float]) -> str:
    xyz = [values[n] for n in JOINTS[:3]]
    # Back to roll, pitch, yaw order for the YAML.
    rpy = [values["dexhand_mount_roll"],
           values["dexhand_mount_pitch"],
           values["dexhand_mount_yaw"]]
    return (
        "mount:\n"
        f"  xyz: [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]\n"
        f"  rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]"
    )


class MountReader(Node):
    def __init__(self, watch: bool) -> None:
        super().__init__("read_mount")
        self.watch = watch
        self.last = None
        self.done = False
        self.create_subscription(JointState, "/joint_states", self.on_states, 10)

    def on_states(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        missing = [name for name in JOINTS if name not in values]
        if missing:
            self.get_logger().error(
                f"missing joints {missing}; is the tuning model loaded? "
                "(display.launch.py model:=tuning)"
            )
            self.done = True
            return

        block = format_block(values)
        if block != self.last:
            self.last = block
            print(block, flush=True)
            if self.watch:
                print("-" * 40, flush=True)
        if not self.watch:
            self.done = True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--watch", action="store_true",
                        help="keep printing as the sliders move")
    args = parser.parse_args()

    rclpy.init()
    node = MountReader(args.watch)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
