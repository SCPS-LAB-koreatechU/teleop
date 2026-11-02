"""Everything on one PC, no hardware: robot.launch.py (dry run) + remote.launch.py.

    ros2 launch bhl_bringup demo.launch.py
    ros2 launch bhl_bringup demo.launch.py model:=stock gui:=false
    ros2 launch bhl_bringup demo.launch.py dry_run:=false       # bench: PC wired to the arms

Same two launch files as the real split, so what works here works across the
network once ROS_DOMAIN_ID matches (env.sh).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PACKAGE = "bhl_bringup"


def generate_launch_description() -> LaunchDescription:
    launch_dir = Path(get_package_share_directory(PACKAGE)) / "launch"
    robot_args = ("dry_run", "config", "model", "urdf", "joint_state_sources", "rate")
    remote_args = ("rviz", "gui", "rviz_config")
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("model", default_value="v1arm"),
        DeclareLaunchArgument("urdf", default_value=""),
        DeclareLaunchArgument("joint_state_sources", default_value="/arm_controller/joint_states"),
        DeclareLaunchArgument("rate", default_value="50"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "robot.launch.py")),
            launch_arguments=[(name, LaunchConfiguration(name)) for name in robot_args]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "remote.launch.py")),
            launch_arguments=[(name, LaunchConfiguration(name)) for name in remote_args]),
    ])
