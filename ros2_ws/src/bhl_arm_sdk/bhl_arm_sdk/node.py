"""ROS 2 node: per-motor angle control of the BHL arms.

Set a joint's angle by parameter (degrees, clamped to the joint's limits):

    ros2 param set /arm_controller target.arm_left_elbow_pitch_joint 30.0

or several at once from a launch / YAML file (`target.<joint>: <deg>`), or by
topic (sensor_msgs/JointState, radians, any subset of joints):

    ros2 topic pub --once /arm_controller/target sensor_msgs/msg/JointState \\
        "{name: [arm_left_elbow_pitch_joint], position: [0.5]}"

Nothing moves until the joints are enabled:

    ros2 service call /arm_controller/enable std_srvs/srv/SetBool "{data: true}"

Each joint then slews to its target at most `max_speed_deg` deg/s.

  ~/enable        std_srvs/SetBool   true: position mode holding the current pose
                                     false: damping, then nothing is commanded
  ~/estop         std_srvs/Trigger   damping on every joint, latched
  ~/reset_estop   std_srvs/Trigger
  ~/home          std_srvs/Trigger   every target to 0 deg
  ~/target        sensor_msgs/JointState  (in)  goals [rad]
  /joint_states   sensor_msgs/JointState  (out) measured [rad]
  ~/status        std_msgs/String    (out) JSON: per joint measured / command / goal / fault
  /diagnostics    diagnostic_msgs/DiagnosticArray (out, 1 Hz) the same, one status per
                                     joint, for rqt_robot_monitor / diagnostic_aggregator

Parameters: config (YAML path), dry_run (default true: no CAN is opened, the
angle follows the command), max_speed_deg, target.<joint> [deg].

Under bhl_bringup robot.launch.py, /joint_states is remapped to
/arm_controller/joint_states and joint_state_publisher + robot_state_publisher
turn it into the full-URDF /joint_states and /tf.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, FloatingPointRange
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .config import load_config
from .controller import ArmController
from .diagnostics import diagnostics

DIAGNOSTIC_LEVELS = (DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR, DiagnosticStatus.STALE)


class ArmControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_controller")
        default_config = Path(get_package_share_directory("bhl_arm_sdk")) / "config" / "bhl_arms.yaml"
        self.declare_parameter("config", str(default_config))
        self.declare_parameter("dry_run", True)
        config = load_config(self.get_parameter("config").value)
        self.declare_parameter("max_speed_deg", config.max_speed_deg,
                               ParameterDescriptor(description="per-joint slew limit [deg/s]"))
        dry_run = bool(self.get_parameter("dry_run").value)
        config.max_speed_deg = float(self.get_parameter("max_speed_deg").value)

        self.arm = ArmController(config, dry_run=dry_run)
        self.dt = 1.0 / config.control_rate_hz

        # One parameter per joint: target.<joint> in degrees, with the joint's limits.
        for spec in config.joints:
            self.declare_parameter(
                f"target.{spec.name}", 0.0,
                ParameterDescriptor(
                    description=f"{spec.name} target [deg] (CAN {spec.device_id} on {spec.bus})",
                    floating_point_range=[FloatingPointRange(
                        from_value=float(spec.lower_deg), to_value=float(spec.upper_deg), step=0.0)]))
        self.add_on_set_parameters_callback(self._on_parameters)

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(JointState, "~/target", self._on_target, 10)
        self.create_service(SetBool, "~/enable", self._on_enable)
        self.create_service(Trigger, "~/estop", self._on_estop)
        self.create_service(Trigger, "~/reset_estop", self._on_reset_estop)
        self.create_service(Trigger, "~/home", self._on_home)
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.1, self._publish_status)
        self.create_timer(1.0, self._publish_diagnostics)

        mode = "DRY RUN (no CAN opened)" if dry_run else f"LIVE on {sorted(set(config.buses.values()))}"
        self.get_logger().info(
            f"{mode}: {len(config.joints)} joints, {config.control_rate_hz:.0f} Hz, "
            f"max {config.max_speed_deg:.0f} deg/s, zero at {config.zero}. "
            "Output is off -- call ~/enable.")

    # ------------------------------------------------------------- parameters
    def _on_parameters(self, params: list[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name.startswith("target."):
                name = param.name[len("target."):]
                if name not in self.arm.joints:
                    return SetParametersResult(successful=False, reason=f"unknown joint {name}")
                if param.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    return SetParametersResult(successful=False, reason="target must be a number [deg]")
                spec = self.arm.joints[name].spec
                value = float(param.value)
                if not spec.lower_deg <= value <= spec.upper_deg:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{name}: {value} deg outside {spec.lower_deg}..{spec.upper_deg}")
                self.arm.set_goal(name, value)
            elif param.name == "max_speed_deg":
                if float(param.value) <= 0:
                    return SetParametersResult(successful=False, reason="max_speed_deg must be > 0")
                self.arm.max_speed_deg = float(param.value)
            elif param.name in ("config", "dry_run"):
                return SetParametersResult(successful=False, reason=f"{param.name} is fixed at start-up")
        return SetParametersResult(successful=True)

    def _sync_target_params(self, goals: dict[str, float]) -> None:
        """Keep target.<joint> equal to what was actually commanded by topic / service."""
        self.set_parameters([Parameter(f"target.{n}", Parameter.Type.DOUBLE, float(v)) for n, v in goals.items()])

    # ------------------------------------------------------------------ inputs
    def _on_target(self, message: JointState) -> None:
        goals = {n: math.degrees(p) for n, p in zip(message.name, message.position) if n in self.arm.joints}
        self._sync_target_params(self.arm.set_goals(goals))

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            up = self.arm.enable()
            # start from where the arm is: targets follow the measured pose
            self._sync_target_params({n: self.arm.joints[n].goal_deg for n in up})
            faults = {n: s.fault for n, s in self.arm.joints.items() if s.fault}
            response.success = bool(up)
            response.message = f"enabled {up}" + (f"; not enabled {faults}" if faults else "")
        else:
            self.arm.disable()
            response.success = True
            response.message = "disabled (damping)"
        self.get_logger().warn(response.message)
        return response

    def _on_estop(self, _request, response: Trigger.Response) -> Trigger.Response:
        self.arm.estop()
        response.success, response.message = True, "E-STOP: every joint in damping, latched"
        self.get_logger().error(response.message)
        return response

    def _on_reset_estop(self, _request, response: Trigger.Response) -> Trigger.Response:
        self.arm.reset_estop()
        response.success, response.message = True, self.arm.message
        return response

    def _on_home(self, _request, response: Trigger.Response) -> Trigger.Response:
        goals = self.arm.set_goals({n: 0.0 for n in self.arm.joints})
        self._sync_target_params(goals)
        response.success, response.message = True, "every target set to 0 deg"
        return response

    # -------------------------------------------------------------------- loop
    def _tick(self) -> None:
        was_estopped = self.arm.estopped
        self.arm.step(self.dt)
        if self.arm.estopped and not was_estopped:
            self.get_logger().error(self.arm.message)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        for name, state in self.arm.joints.items():
            message.name.append(name)
            message.position.append(math.radians(state.measured_deg))
            message.velocity.append(math.radians(state.velocity_dps))
        self.joint_pub.publish(message)

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(self.arm.status())))

    def _publish_diagnostics(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for entry in diagnostics(self.arm.status()):
            status = DiagnosticStatus(level=DIAGNOSTIC_LEVELS[entry["level"]], name=entry["name"],
                                      message=entry["message"], hardware_id=entry["hardware_id"])
            status.values = [KeyValue(key=k, value=v) for k, v in entry["values"].items()]
            message.status.append(status)
        self.diagnostics_pub.publish(message)

    def destroy_node(self) -> None:
        self.arm.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
