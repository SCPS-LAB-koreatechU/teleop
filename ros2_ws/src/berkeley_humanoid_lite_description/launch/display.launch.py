"""Show a robot model in RViz with a slider per joint.

No MoveIt and no IK: just robot_state_publisher and joint_state_publisher_gui.
This is the URDF sanity check, and with model:=tuning it is also how the
DexHand mount transform gets dialled in.

    ros2 launch berkeley_humanoid_lite_description display.launch.py
    ros2 launch berkeley_humanoid_lite_description display.launch.py
    ros2 launch berkeley_humanoid_lite_description display.launch.py model:=tuning

Models:
    stock    arms only, original end effectors
    dexhand  DexHand v2 welded to the right forearm
    tuning   same, but the mount is six sliders; pair with read_mount.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "berkeley_humanoid_lite_description"

# v1arm is the build -- V1 forearm and wrist carrying a DexHand V2 -- and so the
# default. It was missing here entirely, which meant the URDF sanity check could
# not be run against the robot that exists.
MODELS = {
    "v1arm": "berkeley_humanoid_lite_v1arm.urdf",
    "stock": "berkeley_humanoid_lite.urdf",
    "dexhand": "berkeley_humanoid_lite_dexhand.urdf",
    "tuning": "berkeley_humanoid_lite_tuning.urdf",
}


def launch_setup(context, *_args, **_kwargs):
    model = LaunchConfiguration("model").perform(context)
    if model not in MODELS:
        raise RuntimeError(
            f"unknown model '{model}'; choose one of {sorted(MODELS)}"
        )

    use_gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1", "yes")

    share = Path(get_package_share_directory(PACKAGE))
    urdf_path = share / "urdf" / MODELS[model]
    if not urdf_path.is_file():
        raise RuntimeError(
            f"{urdf_path} is missing; run generate_urdf.py and rebuild"
        )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": urdf_path.read_text()}],
        ),
    ]
    if use_gui:
        nodes.append(Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ))
    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(share / "rviz" / "display.rviz")],
    ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="v1arm",
            description=f"which URDF to show: {', '.join(sorted(MODELS))}",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="start joint_state_publisher_gui (false when another node publishes /joint_states)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
