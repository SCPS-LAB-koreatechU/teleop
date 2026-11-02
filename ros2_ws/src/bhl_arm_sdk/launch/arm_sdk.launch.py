"""arm_controller (+ slider GUI, + RViz showing the arms move).

    ros2 launch bhl_arm_sdk arm_sdk.launch.py                   # dry run + GUI + RViz
    ros2 launch bhl_arm_sdk arm_sdk.launch.py dry_run:=false    # real arms on CAN
    ros2 launch bhl_arm_sdk arm_sdk.launch.py rviz:=false gui:=false config:=/path/to/arms.yaml

RViz needs the robot model. By default it is the full upper body,
berkeley_humanoid_lite_v1arm.urdf from the berkeley_humanoid_lite_description
package of the Berkeley-Humanoid-Lite ros2_ws, whose meshes are package://
paths -- so that workspace must be sourced too:

    source ~/Berkeley-Humanoid-Lite/ros2_ws/install/setup.bash

or pass `urdf:=/path/to/robot.urdf`. Without a URDF, RViz is skipped with a
warning and the controller and GUI start as usual.

arm_controller's /joint_states only has the ten arm joints; with RViz on it is
remapped to /arm_controller/joint_states and joint_state_publisher merges it
with zeros for every other movable joint (wrists, fingers), which RViz's
RobotModel needs to draw the whole robot.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "bhl_arm_sdk"
ARM_STATES = "/arm_controller/joint_states"


def default_urdf() -> str:
    try:
        share = Path(get_package_share_directory("berkeley_humanoid_lite_description"))
    except Exception:  # noqa: BLE001 - PackageNotFoundError
        return ""
    path = share / "urdf" / "berkeley_humanoid_lite_v1arm.urdf"
    return str(path) if path.is_file() else ""


def truthy(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def launch_setup(context, *_args, **_kwargs):
    share = Path(get_package_share_directory(PACKAGE))
    actions = []
    rviz = truthy(context, "rviz")
    urdf = LaunchConfiguration("urdf").perform(context) or default_urdf()
    if rviz and not (urdf and Path(urdf).is_file()):
        actions.append(LogInfo(msg=(
            "[bhl_arm_sdk] RViz skipped: no URDF. Source the Berkeley-Humanoid-Lite ros2_ws "
            "(source ~/Berkeley-Humanoid-Lite/ros2_ws/install/setup.bash) or pass urdf:=<file>.")))
        rviz = False

    actions.append(Node(
        package=PACKAGE, executable="arm_controller", name="arm_controller", output="screen",
        parameters=[{"dry_run": truthy(context, "dry_run"),
                     "config": LaunchConfiguration("config").perform(context)}],
        remappings=[("/joint_states", ARM_STATES)] if rviz else []))
    if truthy(context, "gui"):
        actions.append(Node(package=PACKAGE, executable="slider_gui", name="arm_slider_gui", output="screen"))
    if rviz:
        actions += [
            Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
                 parameters=[{"robot_description": Path(urdf).read_text()}]),
            Node(package="joint_state_publisher", executable="joint_state_publisher", output="screen",
                 parameters=[{"source_list": [ARM_STATES], "rate": 50}]),
            Node(package="rviz2", executable="rviz2", name="arm_sdk_rviz", output="screen",
                 arguments=["-d", str(share / "config" / "arm_sdk.rviz")]),
        ]
    return actions


def generate_launch_description() -> LaunchDescription:
    default_config = Path(get_package_share_directory(PACKAGE)) / "config" / "bhl_arms.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true: no CAN is opened, angles follow the command"),
        DeclareLaunchArgument("config", default_value=str(default_config)),
        DeclareLaunchArgument("gui", default_value="true", description="start the slider panel"),
        DeclareLaunchArgument("rviz", default_value="true", description="show the robot in RViz"),
        DeclareLaunchArgument("urdf", default_value="",
                              description="robot URDF for RViz; default: v1arm from "
                                          "berkeley_humanoid_lite_description if sourced"),
        OpaqueFunction(function=launch_setup),
    ])
