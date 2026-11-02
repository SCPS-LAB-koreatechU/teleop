#!/usr/bin/env python3
"""FollowJointTrajectory servers that replay trajectories onto /joint_states.

These stand in for real controllers so "Plan & Execute" works in RViz without
ros2_control installed.  The node holds a position for every joint in the robot,
publishes them continuously so robot_state_publisher can build the TF tree, and
walks the commanded positions along accepted trajectories.  There is no dynamics
here: whatever MoveIt plans is exactly what comes back out.

Two controllers are exposed, matching the split the hardware will have -- the
arm joints ride a CAN bus, the fingers a serial link to the DexHand -- but both
live in one node so there is a single writer for /joint_states:

    /arms_controller/follow_joint_trajectory
    /right_hand_controller/follow_joint_trajectory

Pass --no-hand for the stock model, which has no fingers.

The action interface is what a hardware bridge would expose, so replacing this
node with one that forwards to `recoil.Bus.transmit_pdo_2` (arms) or the DexHand
serial driver (fingers) is a drop-in change on the MoveIt side.
"""

import argparse
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    "arm_left_shoulder_pitch_joint",
    "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint",
    "arm_left_elbow_pitch_joint",
    "arm_left_elbow_roll_joint",
    "arm_right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint",
    "arm_right_shoulder_yaw_joint",
    "arm_right_elbow_pitch_joint",
    "arm_right_elbow_roll_joint",
]
# DexHand v2, 8 servo. Names match upstream so the real driver stays compatible.
HAND_JOINTS = [
    "R_Index_Pitch", "R_Middle_Pitch", "R_Ring_Pitch", "R_Pinky_Pitch",
    "R_Index_Yaw", "R_Middle_Yaw", "R_Ring_Yaw", "R_Pinky_Yaw",
]
LEFT_HAND_JOINTS = [name.replace("R_", "L_", 1) for name in HAND_JOINTS]
# DexHand V1 wrist: two pitch joints with a yaw between them, per arm.
WRIST_JOINTS = [
    f"arm_{side}_{name}_joint"
    for side in ("left", "right")
    for name in ("wrist_pitch_lower", "wrist_yaw", "wrist_pitch_upper")
]
# The legs are stripped from the URDF (include_legs in arm_attachment.yaml), so
# nothing here publishes them. Publishing a joint the robot description does not
# have makes robot_state_publisher complain on every message. Put the list back
# alongside include_legs: true if the legs ever return.
LEG_JOINTS: list[str] = []

PUBLISH_RATE = 50.0  # [Hz]


def duration_to_seconds(duration: Duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


class SimControllers(Node):
    def __init__(self, controllers: dict[str, list[str]]) -> None:
        super().__init__("sim_controllers")

        joints = list(LEG_JOINTS)
        for names in controllers.values():
            joints.extend(names)

        # Guards `self.positions`, which action threads write and the publisher
        # timer reads.
        self.lock = threading.Lock()
        self.positions = {name: 0.0 for name in joints}

        callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(
            1.0 / PUBLISH_RATE, self.publish_joint_states, callback_group=callback_group
        )

        self.servers = []
        for controller, names in controllers.items():
            self.servers.append(ActionServer(
                self,
                FollowJointTrajectory,
                f"/{controller}/follow_joint_trajectory",
                execute_callback=self.execute_callback,
                goal_callback=lambda _goal: GoalResponse.ACCEPT,
                cancel_callback=lambda _goal: CancelResponse.ACCEPT,
                callback_group=callback_group,
            ))
            self.get_logger().info(
                f"controller '{controller}' ready with {len(names)} joints"
            )

    def publish_joint_states(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            message.name = list(self.positions)
            message.position = [self.positions[name] for name in message.name]
        self.publisher.publish(message)

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        points = list(trajectory.points)
        result = FollowJointTrajectory.Result()

        unknown = [name for name in names if name not in self.positions]
        if unknown:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"unknown joints: {unknown}"
            return result

        if not points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        self.get_logger().info(
            f"executing {len(points)} points over "
            f"{duration_to_seconds(points[-1].time_from_start):.2f}s "
            f"for {len(names)} joints"
        )

        # Replay in wall-clock time, interpolating between planned points so RViz
        # shows smooth motion rather than snapping between waypoints.
        start = self.get_clock().now()
        rate = self.create_rate(PUBLISH_RATE)
        previous_time = 0.0
        with self.lock:
            previous = [self.positions[name] for name in names]

        for point in points:
            target_time = duration_to_seconds(point.time_from_start)
            segment = target_time - previous_time

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "cancelled"
                    return result

                elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
                if elapsed >= target_time:
                    break

                alpha = 1.0 if segment <= 0.0 else (elapsed - previous_time) / segment
                alpha = min(max(alpha, 0.0), 1.0)
                with self.lock:
                    for index, name in enumerate(names):
                        self.positions[name] = (
                            previous[index]
                            + alpha * (point.positions[index] - previous[index])
                        )
                rate.sleep()

            with self.lock:
                for index, name in enumerate(names):
                    self.positions[name] = point.positions[index]
            previous = list(point.positions)
            previous_time = target_time

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=("stock", "dexhand", "v1arm"),
                        default="v1arm")
    args, _ = parser.parse_known_args()

    if args.model == "stock":
        controllers = {"arms_controller": ARM_JOINTS}
    elif args.model == "dexhand":
        controllers = {"arms_controller": ARM_JOINTS,
                       "right_hand_controller": HAND_JOINTS}
    else:
        controllers = {"arms_controller": ARM_JOINTS + WRIST_JOINTS,
                       "right_hand_controller": HAND_JOINTS,
                       "left_hand_controller": LEFT_HAND_JOINTS}

    rclpy.init()
    node = SimControllers(controllers)
    # MultiThreadedExecutor: execute_callback blocks on rate.sleep() while the
    # publisher timer has to keep firing.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
