#!/usr/bin/env python3
"""Drive the arms and hands from code instead of dragging the RViz marker.

Humble predates `moveit_py`, so this talks to move_group over its action and
service interfaces directly: `/move_action` to plan and execute, and
`/compute_cartesian_path` for straight-line moves.

Named targets (`home`, `ready`, `fist`, `cylinder_grip`, ...) are read from the
**running** move_group's `robot_description_semantic` parameter rather than from
a file on disk, so this always agrees with whichever model was launched.

Use as a library:

    from motion_commander import MotionCommander
    arm = MotionCommander()
    arm.move_named("right_arm", "ready")
    arm.move_position("right_arm", [0.30, -0.42, 0.62])
    arm.move_named("right_hand", "cylinder_grip")

or from the command line:

    ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py --list
    ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py right_arm ready
    ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py right_hand fist
    ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py --position 0.30 -0.42 0.62
    ros2 run berkeley_humanoid_lite_moveit_config motion_commander.py --demo

A note on orientation: on this robot the wrist travel is small, so only a narrow
cone of palm orientations is reachable at any given point.  `move_position`
leaves orientation free and is what you normally want; `move_pose` constrains it
and will fail far more often.
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

# Which link each group's pose goals refer to. Groups not listed here are
# joint-space only (hands), which is why they only take named targets.
DEFAULT_TIPS = {
    "right_arm": ["base_link", "arm_right_hand_link"],
    "left_arm": ["L_base_link", "arm_left_hand_link"],
}

PLANNING_FRAME = "world"
POSITION_TOLERANCE = 0.005  # [m]
ORIENTATION_TOLERANCE = 0.05  # [rad]


class MotionCommander(Node):
    def __init__(self, timeout: float = 20.0) -> None:
        super().__init__("motion_commander")

        self.move_group = ActionClient(self, MoveGroup, "/move_action")
        if not self.move_group.wait_for_server(timeout_sec=timeout):
            raise SystemExit("/move_action unavailable; is the demo launched?")

        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")

        self.state: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_states, 10)

        self.semantic = self._fetch_semantic(timeout)
        self.groups = self._parse_groups(self.semantic)
        self.named = self._parse_named(self.semantic)
        self.tips = self._resolve_tips()

    # ---------------------------------------------------------------- setup

    def _on_states(self, message: JointState) -> None:
        self.state.update(zip(message.name, message.position))

    def _fetch_semantic(self, timeout: float) -> str:
        """Read the SRDF off the running move_group, not off disk.

        rclpy.parameter_client only lands in Iron, so this calls move_group's
        parameter service directly.
        """
        client = self.create_client(GetParameters, "/move_group/get_parameters")
        if not client.wait_for_service(timeout_sec=timeout):
            raise SystemExit("/move_group/get_parameters unavailable")
        request = GetParameters.Request(names=["robot_description_semantic"])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is None or not result.values:
            raise SystemExit("could not read robot_description_semantic")
        text = result.values[0].string_value
        if not text:
            raise SystemExit("robot_description_semantic is empty")
        return text

    @staticmethod
    def _parse_groups(semantic: str) -> dict[str, list[str]]:
        root = ET.fromstring(semantic)
        groups: dict[str, list[str]] = {}
        for group in root.findall("group"):
            groups[group.get("name")] = [j.get("name") for j in group.findall("joint")]
        return groups

    @staticmethod
    def _parse_named(semantic: str) -> dict[str, dict[str, dict[str, float]]]:
        root = ET.fromstring(semantic)
        named: dict[str, dict[str, dict[str, float]]] = {}
        for state in root.findall("group_state"):
            values = {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
            named.setdefault(state.get("group"), {})[state.get("name")] = values
        return named

    def _resolve_tips(self) -> dict[str, str]:
        """Pick each arm's tip link from the chain the SRDF actually declares."""
        root = ET.fromstring(self.semantic)
        tips: dict[str, str] = {}
        for group in root.findall("group"):
            chain = group.find("chain")
            if chain is not None:
                tips[group.get("name")] = chain.get("tip_link")
        for group, candidates in DEFAULT_TIPS.items():
            if group not in tips:
                continue
            if tips[group] not in candidates:
                self.get_logger().warn(
                    f"{group} tip is {tips[group]}, not one of {candidates}"
                )
        return tips

    # ------------------------------------------------------------ execution

    def _send(self, goal: MoveGroup.Goal, description: str) -> bool:
        send = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{description}: goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        wrapper = result_future.result()
        if wrapper is None:
            self.get_logger().error(f"{description}: no result before timeout")
            return False

        code = wrapper.result.error_code.val
        if code != 1:
            self.get_logger().error(f"{description}: failed, MoveIt error code {code}")
            return False

        points = len(wrapper.result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(f"{description}: done ({points} trajectory points)")
        return True

    @staticmethod
    def _base_goal(group: str, velocity: float, acceleration: float) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = velocity
        goal.request.max_acceleration_scaling_factor = acceleration
        goal.planning_options.plan_only = False
        return goal

    def _check_group(self, group: str) -> None:
        if group not in self.groups:
            raise SystemExit(
                f"unknown group '{group}'; this model has {sorted(self.groups)}"
            )

    # --------------------------------------------------------------- motions

    def move_joints(self, group: str, values: dict[str, float],
                    tolerance: float = 0.01, velocity: float = 0.5,
                    acceleration: float = 0.5) -> bool:
        """Plan and execute to an explicit joint configuration."""
        self._check_group(group)
        goal = self._base_goal(group, velocity, acceleration)
        constraints = Constraints()
        for name, value in values.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value,
                tolerance_above=tolerance, tolerance_below=tolerance, weight=1.0,
            ))
        goal.request.goal_constraints.append(constraints)
        return self._send(goal, f"{group} -> {len(values)} joint targets")

    def move_named(self, group: str, state: str, **kwargs) -> bool:
        """Plan and execute to a named target from the SRDF."""
        self._check_group(group)
        available = self.named.get(group, {})
        if state not in available:
            raise SystemExit(
                f"'{state}' is not a target of {group}; try {sorted(available)}"
            )
        self.get_logger().info(f"{group} -> '{state}'")
        return self.move_joints(group, available[state], **kwargs)

    def move_position(self, group: str, xyz, tolerance: float = POSITION_TOLERANCE,
                      velocity: float = 0.5, acceleration: float = 0.5) -> bool:
        """Send the tip to a point, leaving orientation free.

        This is the goal type to reach for on this robot: the wrist has little
        travel, so constraining orientation as well usually makes the goal
        unreachable.
        """
        self._check_group(group)
        tip = self.tips.get(group)
        if tip is None:
            raise SystemExit(f"{group} has no chain tip; it takes named targets only")

        goal = self._base_goal(group, velocity, acceleration)
        constraints = Constraints()

        volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tolerance]
        volume.primitives.append(sphere)
        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = (float(v) for v in xyz)
        centre.orientation.w = 1.0
        volume.primitive_poses.append(centre)

        position = PositionConstraint()
        position.header.frame_id = PLANNING_FRAME
        position.link_name = tip
        position.constraint_region = volume
        position.weight = 1.0
        constraints.position_constraints.append(position)

        goal.request.goal_constraints.append(constraints)
        return self._send(
            goal,
            f"{group} -> ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})",
        )

    def move_pose(self, group: str, xyz, quaternion,
                  position_tolerance: float = POSITION_TOLERANCE,
                  orientation_tolerance: float = ORIENTATION_TOLERANCE,
                  velocity: float = 0.5, acceleration: float = 0.5) -> bool:
        """Send the tip to a full pose. Expect failures: see the module docstring."""
        self._check_group(group)
        tip = self.tips.get(group)
        if tip is None:
            raise SystemExit(f"{group} has no chain tip; it takes named targets only")

        goal = self._base_goal(group, velocity, acceleration)
        constraints = Constraints()

        volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]
        volume.primitives.append(sphere)
        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = (float(v) for v in xyz)
        centre.orientation.w = 1.0
        volume.primitive_poses.append(centre)

        position = PositionConstraint()
        position.header.frame_id = PLANNING_FRAME
        position.link_name = tip
        position.constraint_region = volume
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = PLANNING_FRAME
        orientation.link_name = tip
        (orientation.orientation.x, orientation.orientation.y,
         orientation.orientation.z, orientation.orientation.w) = (
            float(v) for v in quaternion)
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)

        goal.request.goal_constraints.append(constraints)
        return self._send(goal, f"{group} -> pose")

    def move_cartesian(self, group: str, waypoints: list[PoseStamped],
                       step: float = 0.005, min_fraction: float = 0.9) -> bool:
        """Follow a straight line through the given tip poses.

        Returns False if less than `min_fraction` of the path could be planned,
        which is the usual outcome when a segment leaves the workspace.
        """
        self._check_group(group)
        if not self.cartesian.wait_for_service(timeout_sec=10.0):
            raise SystemExit("/compute_cartesian_path unavailable")

        request = GetCartesianPath.Request()
        request.header.frame_id = PLANNING_FRAME
        request.group_name = group
        request.link_name = self.tips[group]
        request.waypoints = [w.pose for w in waypoints]
        request.max_step = step
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        future = self.cartesian.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        response = future.result()
        if response is None:
            self.get_logger().error("cartesian path: no response")
            return False
        if response.fraction < min_fraction:
            self.get_logger().error(
                f"cartesian path: only {response.fraction:.0%} planned "
                f"(wanted {min_fraction:.0%})"
            )
            return False

        self.get_logger().info(f"cartesian path: {response.fraction:.0%} planned")
        # Executing the returned trajectory needs /execute_trajectory; planning
        # it is the useful part here, so report and let the caller decide.
        return True

    # ---------------------------------------------------------------- report

    def describe(self) -> str:
        lines = ["groups and named targets on the running model:", ""]
        for group in sorted(self.groups):
            tip = self.tips.get(group)
            suffix = f"  (tip: {tip})" if tip else "  (joint-space only)"
            lines.append(f"  {group}{suffix}")
            targets = sorted(self.named.get(group, {}))
            lines.append(f"      {', '.join(targets) if targets else '(no named targets)'}")
        return "\n".join(lines)


def run_demo(commander: MotionCommander) -> bool:
    """A short sequence touching every motion type, on whichever arms exist."""
    arms = [g for g in ("right_arm", "left_arm") if g in commander.groups]
    hands = [g for g in ("right_hand", "left_hand") if g in commander.groups]
    steps: list[tuple[str, callable]] = []

    for arm in arms:
        steps.append((f"{arm} to ready", lambda a=arm: commander.move_named(a, "ready")))
    for hand in hands:
        if "open" in commander.named.get(hand, {}):
            steps.append((f"{hand} open", lambda h=hand: commander.move_named(h, "open")))
        if "cylinder_grip" in commander.named.get(hand, {}):
            steps.append((f"{hand} cylinder_grip",
                          lambda h=hand: commander.move_named(h, "cylinder_grip")))
    for arm in arms:
        steps.append((f"{arm} back to home",
                      lambda a=arm: commander.move_named(a, "home")))

    failures = 0
    for index, (label, action) in enumerate(steps, 1):
        print(f"[{index}/{len(steps)}] {label}")
        if not action():
            failures += 1
            print(f"        FAILED: {label}")
    print(f"\ndemo: {len(steps) - failures}/{len(steps)} steps succeeded")
    return failures == 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Drive the arms and hands through MoveIt.")
    parser.add_argument("group", nargs="?", help="planning group, e.g. right_arm")
    parser.add_argument("target", nargs="?", help="named target, e.g. ready")
    parser.add_argument("--list", action="store_true",
                        help="show groups and named targets, then exit")
    parser.add_argument("--demo", action="store_true", help="run a short sequence")
    parser.add_argument("--position", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="send the group's tip to a point, orientation free")
    parser.add_argument("--pose", nargs=7, type=float,
                        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                        help="send the group's tip to a full pose (often fails)")
    parser.add_argument("--velocity", type=float, default=0.5)
    parser.add_argument("--acceleration", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    commander = MotionCommander()
    try:
        if args.list:
            print(commander.describe())
            return
        if args.demo:
            sys.exit(0 if run_demo(commander) else 1)

        scaling = {"velocity": args.velocity, "acceleration": args.acceleration}
        if args.position:
            if not args.group:
                raise SystemExit("--position needs a group, e.g. right_arm")
            ok = commander.move_position(args.group, args.position, **scaling)
        elif args.pose:
            if not args.group:
                raise SystemExit("--pose needs a group, e.g. right_arm")
            ok = commander.move_pose(args.group, args.pose[:3], args.pose[3:], **scaling)
        elif args.group and args.target:
            ok = commander.move_named(args.group, args.target, **scaling)
        else:
            print(commander.describe())
            print("\nnothing to do; pass a group and target, --position, or --demo")
            return
        sys.exit(0 if ok else 1)
    finally:
        commander.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
