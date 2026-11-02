"""Bring up a second DexHand as the robot's left hand.

Two nodes:

  * `dexhand_bringup`'s own `hand_driver`, in the `/left_hand` namespace and
    pointed at a second serial port. It still thinks it is a right hand.
  * `left_hand_adapter.py`, which translates `L_` names to the driver's `R_`
    names and back, and republishes the driver's joint states.

The driver publishes to an **absolute** `/joint_states`, so a namespace alone
would not separate the two hands -- the remap below is what keeps the left
driver's raw `R_` names off the global topic. Only the adapter's translated
`L_` names get there.

    ros2 launch berkeley_humanoid_lite_moveit_config left_hand.launch.py \\
        serial_port:=/dev/ttyACM1

This needs the dexhand workspace on the environment as well as this one:

    source ~/Desktop/dexhand_moveit_ws/install/setup.bash

Servo output starts off, as it does for the right hand:

    ros2 service call /left_hand_adapter/enable std_srvs/srv/SetBool '{data: true}'
"""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = "left_hand"
RAW_STATES = f"/{NAMESPACE}/raw_joint_states"


def launch_setup(context, *_args, **_kwargs):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("dexhand_bringup")
    except Exception as error:  # noqa: BLE001 - want the message, not the class
        raise RuntimeError(
            "dexhand_bringup not found. Source the dexhand workspace too:\n"
            "  source ~/Desktop/dexhand_moveit_ws/install/setup.bash\n"
            f"({error})"
        )

    serial_port = LaunchConfiguration("serial_port").perform(context)
    link = LaunchConfiguration("link").perform(context)
    yaw_dir = [float(v) for v in
               LaunchConfiguration("yaw_dir").perform(context).split(",")]
    flex_dir = [float(v) for v in
                LaunchConfiguration("flex_dir").perform(context).split(",")]

    # Same calibration file the right hand runs with (spans, pitch/yaw ranges),
    # minus the two keys that are not valid ROS parameters.
    with open(Path(get_package_share_directory("dexhand_bringup")) / "config"
              / "servo_map.yaml", encoding="utf-8") as fp:
        servo_params = yaml.safe_load(fp)["dexhand_driver"]["ros__parameters"]
    for key in ("servo_cal", "servo_names"):
        servo_params.pop(key, None)

    return [
        Node(
            package="dexhand_bringup",
            executable="hand_driver",
            name="dexhand_driver",
            namespace=NAMESPACE,
            output="screen",
            parameters=[servo_params, {
                "link": link,
                "serial_port": serial_port,
                # The left hand is a mirrored assembly, so spread runs the other
                # way while curl does not. Verify on the bench before trusting it:
                # command a small spread and check the fingers move apart.
                "yaw_dir": yaw_dir,
                "flex_dir": flex_dir,
                "auto_enable_output": False,
            }],
            # The driver publishes to an absolute /joint_states; without this
            # both hands would publish R_ names onto the same topic.
            remappings=[("/joint_states", RAW_STATES)],
        ),
        Node(
            package="berkeley_humanoid_lite_moveit_config",
            executable="left_hand_adapter.py",
            name="left_hand_adapter",
            output="screen",
            parameters=[{
                "driver_action":
                    f"/{NAMESPACE}/hand_controller/follow_joint_trajectory",
                "driver_states": RAW_STATES,
                "driver_enable": f"/{NAMESPACE}/dexhand_driver/enable",
                "controller_action":
                    "/left_hand_controller/follow_joint_trajectory",
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyACM1",
            description="serial port of the LEFT hand (the right one is usually ACM0)",
        ),
        DeclareLaunchArgument(
            "link", default_value="serial",
            description="serial for the real board, none for a dry run of the "
                        "adapter/driver pair with no port opened",
        ),
        DeclareLaunchArgument(
            "yaw_dir", default_value="-1,-1,-1,-1",
            description="per-finger spread direction; flipped for the mirrored hand",
        ),
        DeclareLaunchArgument(
            "flex_dir", default_value="1,1,1,1",
            description="per-finger curl direction; unchanged by mirroring",
        ),
        OpaqueFunction(function=launch_setup),
    ])
