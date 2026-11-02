#!/usr/bin/env python3
"""Present a second DexHand as the robot's left hand.

`dexhand_bringup`'s driver is hardwired to one hand: its joint names come from
`JOINT_NAMES = R_<finger>_<Yaw|Pitch>` and it publishes them on the global
`/joint_states`.  Two instances would therefore both claim to be the right hand.

Rather than fork that driver, this node runs in front of a second instance and
translates:

    MoveIt  --L_* trajectory-->  adapter  --R_* trajectory-->  left driver
    MoveIt  <--L_* joint states--  adapter  <--R_* joint states--  left driver

so the left driver never learns it is a left hand, and MoveIt never learns there
is a translation.  The mirroring that matters physically is handled by the
driver's own `yaw_dir` parameter, not here; see left_hand.launch.py.

The launch file is what actually wires this up:

    ros2 launch berkeley_humanoid_lite_moveit_config left_hand.launch.py \\
        serial_port:=/dev/ttyACM1

Servo output stays off until the relayed enable service is called:

    ros2 service call /left_hand_adapter/enable std_srvs/srv/SetBool '{data: true}'
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

# The driver's own prefix, and the one this robot's URDF uses for the left hand.
DRIVER_PREFIX = "R_"
LEFT_PREFIX = "L_"


def to_driver(name: str) -> str:
    """L_Index_Pitch -> R_Index_Pitch."""
    return DRIVER_PREFIX + name[len(LEFT_PREFIX):] if name.startswith(LEFT_PREFIX) else name


def to_left(name: str) -> str:
    """R_Index_Pitch -> L_Index_Pitch."""
    return LEFT_PREFIX + name[len(DRIVER_PREFIX):] if name.startswith(DRIVER_PREFIX) else name


class LeftHandAdapter(Node):
    def __init__(self) -> None:
        super().__init__("left_hand_adapter")

        self.declare_parameter("driver_action",
                               "/left_hand/hand_controller/follow_joint_trajectory")
        self.declare_parameter("driver_states", "/left_hand/raw_joint_states")
        self.declare_parameter("driver_enable", "/left_hand/dexhand_driver/enable")
        self.declare_parameter("controller_action",
                               "/left_hand_controller/follow_joint_trajectory")

        driver_action = self.get_parameter("driver_action").value
        driver_states = self.get_parameter("driver_states").value
        self.driver_enable_name = self.get_parameter("driver_enable").value
        controller_action = self.get_parameter("controller_action").value

        group = ReentrantCallbackGroup()

        self.client = ActionClient(self, FollowJointTrajectory, driver_action,
                                   callback_group=group)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, driver_states, self._on_states, 10,
                                 callback_group=group)
        self.enable_client = self.create_client(SetBool, self.driver_enable_name,
                                                callback_group=group)
        self.create_service(SetBool, "~/enable", self._on_enable,
                            callback_group=group)

        self.server = ActionServer(
            self, FollowJointTrajectory, controller_action,
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=group,
        )

        self.get_logger().info(
            f"left hand adapter up\n"
            f"  MoveIt side : {controller_action}  (L_ names)\n"
            f"  driver side : {driver_action}  (R_ names)\n"
            f"  states      : {driver_states} -> /joint_states"
        )
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"{driver_action} is not up yet; trajectories will be rejected "
                "until the left dexhand_driver starts"
            )

    @staticmethod
    def _await(future, timeout: float):
        """Block on a future without spinning.

        These callbacks already run inside the executor, so calling
        spin_until_future_complete here re-enters it and the future never
        progresses. Waiting on a done-callback instead lets the executor's other
        threads deliver the reply.
        """
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    # ------------------------------------------------------------------ state

    def _on_states(self, message: JointState) -> None:
        """Republish the driver's R_ names under L_ names."""
        out = JointState()
        out.header = message.header
        out.name = [to_left(n) for n in message.name]
        out.position = list(message.position)
        out.velocity = list(message.velocity)
        out.effort = list(message.effort)
        self.publisher.publish(out)

    # ---------------------------------------------------------------- enable

    def _on_enable(self, request, response):
        if not self.enable_client.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = f"{self.driver_enable_name} unavailable"
            return response
        future = self.enable_client.call_async(SetBool.Request(data=request.data))
        result = self._await(future, 10.0)
        if result is None:
            response.success = False
            response.message = "left driver did not answer the enable call"
            return response
        response.success = result.success
        response.message = f"left driver: {result.message}"
        self.get_logger().warn(response.message)
        return response

    # ------------------------------------------------------------- execution

    def _execute(self, goal_handle):
        request = goal_handle.request
        result = FollowJointTrajectory.Result()

        if not self.client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "left dexhand_driver is not running"
            self.get_logger().error(result.error_string)
            return result

        foreign = [n for n in request.trajectory.joint_names
                   if not n.startswith(LEFT_PREFIX)]
        if foreign:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"not left-hand joints: {foreign}"
            self.get_logger().error(result.error_string)
            return result

        forwarded = FollowJointTrajectory.Goal()
        forwarded.trajectory = request.trajectory
        forwarded.trajectory.joint_names = [
            to_driver(n) for n in request.trajectory.joint_names
        ]
        # Tolerances carry joint names too, and the driver only knows R_ names.
        forwarded.path_tolerance = [
            self._rename_tolerance(t) for t in request.path_tolerance
        ]
        forwarded.goal_tolerance = [
            self._rename_tolerance(t) for t in request.goal_tolerance
        ]
        forwarded.goal_time_tolerance = request.goal_time_tolerance

        self.get_logger().info(
            f"forwarding {len(forwarded.trajectory.points)} points as "
            f"{forwarded.trajectory.joint_names}"
        )
        send = self.client.send_goal_async(forwarded)
        handle = self._await(send, 30.0)
        if handle is None or not handle.accepted:
            goal_handle.abort()
            result.error_code = -1
            result.error_string = "left driver rejected the goal"
            self.get_logger().error(result.error_string)
            return result

        result_future = handle.get_result_async()
        finished = threading.Event()
        result_future.add_done_callback(lambda _f: finished.set())
        while rclpy.ok() and not finished.wait(0.05):
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "cancelled"
                return result

        wrapper = result_future.result()
        if wrapper is None:
            goal_handle.abort()
            result.error_code = -1
            result.error_string = "no result from the left driver"
            return result

        downstream = wrapper.result
        if downstream.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result.error_code = downstream.error_code
        result.error_string = downstream.error_string
        return result

    @staticmethod
    def _rename_tolerance(tolerance):
        tolerance.name = to_driver(tolerance.name)
        return tolerance


def main() -> None:
    rclpy.init()
    node = LeftHandAdapter()
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
