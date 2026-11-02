#!/usr/bin/env python3
"""Round-trip IK smoke test against a running move_group.

For each arm: draw a random joint configuration inside the URDF limits, ask
move_group where that puts the hand (/compute_fk), ask it to solve back to that
point (/compute_ik), then run FK on the returned solution and compare.  The two
positions matching proves the chain, the solver config and the joint limits all
agree; the returned joint values usually differ from the drawn ones, which is
expected for a redundant, position-only solve.

On the 5-DOF models orientation is not checked: kinematics.yaml runs the solver
in position-only mode, so the hand's orientation is whatever it landed on.  On
v1arm each arm has 9 DOF and solves the full pose, so orientation is verified
too -- that check is the whole point of adding the V1 wrist.

Start the demo launch with use_rviz:=false, then:

    ros2 run berkeley_humanoid_lite_moveit_config check_ik.py
    ros2 run berkeley_humanoid_lite_moveit_config check_ik.py --model stock

The right arm's tip depends on the model: the stock hand link, or the DexHand's
mount link once the hand is grafted on. Pass --model to match what is running.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState

# Tips per model. On v1arm both arms end at a DexHand mount link; on the others
# only the right arm was regrafted.
TIPS = {
    "v1arm": {"right": "base_link", "left": "L_base_link"},
    "dexhand": {"right": "base_link", "left": "arm_left_hand_link"},
    "stock": {"right": "arm_right_hand_link", "left": "arm_left_hand_link"},
}
# v1arm has 9 DOF per arm and solves full pose, so orientation is checked too.
FULL_POSE = {"v1arm"}
# V1 wrist joints appended to each arm on v1arm. Limits are the real ones from
# the DexHand V1 description and are tight: +/-30, +/-25 and +/-20 degrees. The
# wrist adds enough freedom to solve a pose, but not to reach every orientation.
WRIST = [("wrist_pitch_lower", -0.523599, 0.523599),
         ("wrist_yaw", -0.436332, 0.436332),
         ("wrist_pitch_upper", -0.349066, 0.349066)]

ARMS = {
    "left_arm": {
        "tip": "arm_left_hand_link",
        "joints": [
            ("arm_left_shoulder_pitch_joint", -1.5708, 0.785398),
            ("arm_left_shoulder_roll_joint", -0.261799, 1.309),
            ("arm_left_shoulder_yaw_joint", -0.785398, 0.785398),
            ("arm_left_elbow_pitch_joint", 0.0, 1.5708),
            ("arm_left_elbow_roll_joint", -0.785398, 0.785398),
        ],
    },
    "right_arm": {
        "tip": "arm_right_hand_link",
        "joints": [
            ("arm_right_shoulder_pitch_joint", -0.785398, 1.5708),
            ("arm_right_shoulder_roll_joint", -1.309, 0.261799),
            ("arm_right_shoulder_yaw_joint", -0.785398, 0.785398),
            ("arm_right_elbow_pitch_joint", -1.5708, 0.0),
            ("arm_right_elbow_roll_joint", -0.785398, 0.785398),
        ],
    },
}

TOLERANCE = 5e-3  # [m]
ANGLE_TOLERANCE = 0.02  # [rad], about 1.1 degrees


def quaternion_angle(a, b) -> float:
    """Smallest rotation angle between two quaternions."""
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    return 2.0 * math.acos(min(1.0, dot))


class IKChecker(Node):
    def __init__(self) -> None:
        super().__init__("check_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        for client, name in ((self.fk_client, "/compute_fk"), (self.ik_client, "/compute_ik")):
            if not client.wait_for_service(timeout_sec=20.0):
                raise SystemExit(f"{name} unavailable; is move_group running?")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done():
            raise SystemExit("service call timed out")
        return future.result()

    def forward(self, tip: str, names: list[str], values: list[float]):
        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = [tip]
        request.robot_state.joint_state = JointState(name=names, position=values)
        response = self.call(self.fk_client, request)
        if response.error_code.val != 1 or not response.pose_stamped:
            raise SystemExit(f"FK failed, error code {response.error_code.val}")
        return response.pose_stamped[0]

    def inverse(self, group: str, tip: str, pose_stamped, seed_names, seed_values):
        request = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = group
        ik.ik_link_name = tip
        ik.pose_stamped = pose_stamped
        ik.avoid_collisions = True
        ik.timeout.sec = 1
        ik.robot_state = RobotState()
        ik.robot_state.joint_state = JointState(name=seed_names, position=seed_values)
        request.ik_request = ik
        return self.call(self.ik_client, request)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=25)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--model", choices=sorted(TIPS), default="v1arm")
    args = parser.parse_args()

    full_pose = args.model in FULL_POSE
    for side in ("right", "left"):
        ARMS[f"{side}_arm"]["tip"] = TIPS[args.model][side]
        if full_pose:
            ARMS[f"{side}_arm"]["joints"] += [
                (f"arm_{side}_{name}_joint", lo, hi) for name, lo, hi in WRIST
            ]

    import random

    rclpy.init()
    checker = IKChecker()
    rng = random.Random(args.seed)
    failed = 0

    for group, spec in ARMS.items():
        tip = spec["tip"]
        names = [name for name, _, _ in spec["joints"]]
        solved = 0
        worst = 0.0
        worst_angle = 0.0

        for _ in range(args.trials):
            values = [rng.uniform(lo, hi) for _, lo, hi in spec["joints"]]
            target = checker.forward(tip, names, values)

            # Seed from the mid-range pose, not from `values`, so a success
            # cannot come from the solver being handed the answer.
            seed = [(lo + hi) / 2.0 for _, lo, hi in spec["joints"]]
            response = checker.inverse(group, tip, target, names, seed)
            if response.error_code.val != 1:
                continue

            solution = response.solution.joint_state
            achieved = checker.forward(
                tip, list(solution.name), list(solution.position)
            )
            error = max(
                abs(achieved.pose.position.x - target.pose.position.x),
                abs(achieved.pose.position.y - target.pose.position.y),
                abs(achieved.pose.position.z - target.pose.position.z),
            )
            angle = quaternion_angle(achieved.pose.orientation,
                                     target.pose.orientation)
            worst = max(worst, error)
            worst_angle = max(worst_angle, angle)
            if full_pose and angle > ANGLE_TOLERANCE:
                failed += 1
                print(f"  {group}: orientation off by {math.degrees(angle):.1f} deg")
            elif error <= TOLERANCE:
                solved += 1
            else:
                failed += 1
                print(f"  {group}: solution off target by {error * 1000:.1f} mm")

        rate = solved / args.trials
        summary = (f"{group}: {solved}/{args.trials} reachable targets solved "
                   f"({rate:.0%}), worst position error {worst * 1000:.2f} mm")
        if full_pose:
            summary += f", worst orientation error {math.degrees(worst_angle):.3f} deg"
        print(summary)
        if rate < 0.8:
            failed += 1
            print(f"  {group}: success rate below 80%")

    checker.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
