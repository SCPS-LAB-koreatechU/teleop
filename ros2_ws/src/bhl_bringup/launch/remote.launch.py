"""Remote PC side: look at, and poke, the robot that robot.launch.py brought up.

    ros2 launch bhl_bringup remote.launch.py                     # RViz + slider panel
    ros2 launch bhl_bringup remote.launch.py gui:=false           # RViz only
    ros2 launch bhl_bringup remote.launch.py rviz:=false          # slider panel only
    ros2 launch bhl_bringup remote.launch.py rviz_config:=/path/to/other.rviz

Nothing here talks to the hardware. RViz draws the robot from /robot_description
and /tf (the robot side's robot_state_publisher); the slider panel talks to
/arm_controller over its target topic and services. Meshes are package://
paths, so berkeley_humanoid_lite_description has to be installed on this PC too
(humanoid.repos) -- without it RViz still starts, with the frames but no shapes.
"""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "bhl_bringup"
DESCRIPTION_PACKAGE = "berkeley_humanoid_lite_description"


def truthy(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context, *_args, **_kwargs):
    actions = []
    if truthy(context, "rviz"):
        try:
            get_package_share_directory(DESCRIPTION_PACKAGE)
        except PackageNotFoundError:
            actions.append(LogInfo(msg=f"[bhl_bringup] {DESCRIPTION_PACKAGE} is not installed on this PC: "
                                       "RViz will show frames without meshes (see humanoid.repos)."))
        rviz_config = LaunchConfiguration("rviz_config").perform(context) or \
            str(Path(get_package_share_directory(PACKAGE)) / "config" / "bringup.rviz")
        actions.append(Node(package="rviz2", executable="rviz2", name="bringup_rviz", output="screen",
                            arguments=["-d", rviz_config]))
    if truthy(context, "gui"):
        actions.append(Node(package="bhl_arm_sdk", executable="slider_gui", name="arm_slider_gui",
                            output="screen"))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="show the robot in RViz"),
        DeclareLaunchArgument("gui", default_value="true", description="start the slider panel"),
        DeclareLaunchArgument("rviz_config", default_value="",
                              description="RViz config; default: bhl_bringup/config/bringup.rviz"),
        OpaqueFunction(function=launch_setup),
    ])
