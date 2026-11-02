"""Bring up MoveIt and RViz for the Berkeley Humanoid Lite arms, in simulation.

Starts move_group, robot_state_publisher, RViz with the MotionPlanning panel,
and the simulated trajectory controllers.  No hardware and no CAN bus is touched.

    ros2 launch berkeley_humanoid_lite_moveit_config demo.launch.py
    ros2 launch berkeley_humanoid_lite_moveit_config demo.launch.py model:=stock

Prefer ../run_demo.sh, which also pins ROS_DOMAIN_ID away from the dexhand
session on the default domain.

Models:
    v1arm    V1 forearm + 3-DOF wrist + DexHand v2 fingers on BOTH arms.
             9 DOF per arm, so this is the only model with full 6-DOF pose IK.
             Groups: left_arm, right_arm, both_arms, left_hand, right_hand
    dexhand  DexHand v2 welded to the right arm only; 5 DOF, position-only IK
    stock    original end effectors; 5 DOF, position-only IK

Drag the interactive marker on either hand and hit "Plan & Execute".  On v1arm
the marker's rotation rings work too; on the 5-DOF models orientation is ignored.

Hardware mode (hardware:=true) swaps the simulated controllers for the real
ones while MoveIt and RViz stay exactly the same:

    arms + wrist  -> scripts/hardware_bridge.py   (recoil CAN + Feetech serial)
    right hand    -> dexhand_bringup hand_driver  (Arduino, /dev/ttyACM0)
    left hand     -> left_hand.launch.py          (second Arduino + name adapter)

Everything starts in dry run: the bridge opens no bus (`live:=false`) and the
hand drivers use `link:=none`, so the whole v1arm plan -> execute -> joint_states
loop can be exercised with nothing plugged in.  Then, deliberately:

    ros2 launch ... demo.launch.py hardware:=true                       # dry run
    ros2 launch ... demo.launch.py hardware:=true right_hand:=serial    # hand real, arm dry
    ros2 launch ... demo.launch.py hardware:=true live:=true buses:=wrist right_hand:=serial
    ros2 launch ... demo.launch.py hardware:=true live:=true right_hand:=serial left_hand:=serial

Output stays off until each enable service is called; see COMMANDS.md.
The hand drivers come from the dexhand workspace, so source it as well
(run_demo.sh does that when it exists).
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE = "berkeley_humanoid_lite_moveit_config"
DESCRIPTION_PACKAGE = "berkeley_humanoid_lite_description"

MODELS = {
    "v1arm": {
        "urdf": "berkeley_humanoid_lite_v1arm.urdf",
        "srdf": "config/berkeley_humanoid_lite_v1arm.srdf",
        "kinematics": "config/kinematics_v1arm.yaml",
        "limits": "config/joint_limits_v1arm.yaml",
        "controllers": "config/moveit_controllers_v1arm.yaml",
        "server_args": ["--model", "v1arm"],
        "hands": ("right", "left"),
    },
    "dexhand": {
        "urdf": "berkeley_humanoid_lite_dexhand.urdf",
        "srdf": "config/berkeley_humanoid_lite_dexhand.srdf",
        "kinematics": "config/kinematics.yaml",
        "limits": "config/joint_limits_dexhand.yaml",
        "controllers": "config/moveit_controllers_dexhand.yaml",
        "server_args": ["--model", "dexhand"],
        "hands": ("right",),
    },
    "stock": {
        "urdf": "berkeley_humanoid_lite.urdf",
        "srdf": "config/berkeley_humanoid_lite.srdf",
        "kinematics": "config/kinematics.yaml",
        "limits": "config/joint_limits.yaml",
        "controllers": "config/moveit_controllers.yaml",
        "server_args": ["--model", "stock"],
        "hands": (),
    },
}

# One controller file for hardware: arms + wrist on the bridge, each hand on
# its dexhand driver. MoveIt ignores controller joints the loaded model lacks.
HARDWARE_CONTROLLERS = "config/moveit_controllers_hardware.yaml"
# onboard:=true -- the robot's RDK X5 runs bhl_onboard (~/bhl_onboard_ws), which serves
# /arms_controller/follow_joint_trajectory and publishes /bhl/joint_states.
ONBOARD_CONTROLLERS = "config/moveit_controllers_onboard.yaml"


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true", "yes")


def _dexhand_servo_params() -> dict:
    """servo_map.yaml from dexhand_bringup, minus the two keys that are not
    valid ROS parameters (the dexhand launch does the same)."""
    try:
        bringup = get_package_share_directory("dexhand_bringup")
    except Exception as error:  # noqa: BLE001 - want the message, not the class
        raise RuntimeError(
            "hardware:=true needs dexhand_bringup for the hand drivers. Source "
            "the dexhand workspace too:\n"
            "  source ~/Desktop/dexhand_moveit_ws/install/setup.bash\n"
            f"({error})"
        )
    with open(Path(bringup) / "config" / "servo_map.yaml", encoding="utf-8") as fp:
        params = yaml.safe_load(fp)["dexhand_driver"]["ros__parameters"]
    for key in ("servo_cal", "servo_names"):
        params.pop(key, None)
    return params


def _hardware_nodes(context, spec: dict) -> list:
    """The real-controller side of hardware mode: bridge plus hand drivers."""
    live = _flag(context, "live")
    buses = LaunchConfiguration("buses").perform(context)
    bridge_args = ["--buses", buses] + (["--live"] if live else [])
    bridge_config = LaunchConfiguration("bridge_config").perform(context)
    if bridge_config:
        bridge_args += ["--config", bridge_config]
    nodes = [
        Node(
            package=PACKAGE,
            executable="hardware_bridge.py",
            output="screen",
            arguments=bridge_args,
        ),
    ]

    right = LaunchConfiguration("right_hand").perform(context)
    left = LaunchConfiguration("left_hand").perform(context)
    for value, name in ((right, "right_hand"), (left, "left_hand")):
        if value not in ("none", "serial", "off"):
            raise RuntimeError(f"{name} must be none, serial or off (got '{value}')")

    if "right" in spec["hands"] and right != "off":
        nodes.append(Node(
            package="dexhand_bringup",
            executable="hand_driver",
            name="dexhand_driver",
            output="screen",
            parameters=[
                _dexhand_servo_params(),
                {"link": right,
                 "serial_port": LaunchConfiguration("right_hand_port").perform(context),
                 "auto_enable_output": False},
            ],
        ))
    if "left" in spec["hands"] and left != "off":
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory(PACKAGE)) / "launch" / "left_hand.launch.py")),
            launch_arguments={
                "link": left,
                "serial_port": LaunchConfiguration("left_hand_port").perform(context),
            }.items(),
        ))
    return nodes


def launch_setup(context, *_args, **_kwargs):
    model = LaunchConfiguration("model").perform(context)
    if model not in MODELS:
        raise RuntimeError(f"unknown model '{model}'; choose one of {sorted(MODELS)}")
    spec = MODELS[model]
    hardware = _flag(context, "hardware")
    onboard = _flag(context, "onboard")
    if onboard and hardware:
        raise RuntimeError("onboard:=true and hardware:=true are exclusive: the board "
                           "drives the motors, the PC bridge must not")
    if onboard:
        controllers = ONBOARD_CONTROLLERS
    else:
        controllers = HARDWARE_CONTROLLERS if hardware else spec["controllers"]

    urdf = (
        Path(get_package_share_directory(DESCRIPTION_PACKAGE)) / "urdf" / spec["urdf"]
    )
    if not urdf.is_file():
        raise RuntimeError(f"{urdf} is missing; run generate_urdf.py and rebuild")

    moveit_config = (
        MoveItConfigsBuilder("berkeley-humanoid-lite", package_name=PACKAGE)
        .robot_description(file_path=str(urdf))
        .robot_description_semantic(file_path=spec["srdf"])
        .robot_description_kinematics(file_path=spec["kinematics"])
        .joint_limits(file_path=spec["limits"])
        .trajectory_execution(file_path=controllers)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    rviz_config = str(
        Path(get_package_share_directory(PACKAGE)) / "config" / "moveit.rviz"
    )

    if onboard:
        # The board knows only its motors; RViz and MoveIt need every URDF joint
        # in /joint_states. joint_state_publisher passes the board's values
        # through and fills the rest (fingers, unmapped wrist) with defaults.
        executors = [Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description,
                        {"source_list": ["/bhl/joint_states"], "rate": 50}],
        )]
    elif hardware:
        executors = _hardware_nodes(context, spec)
    else:
        executors = [Node(
            package=PACKAGE,
            executable="sim_trajectory_server.py",
            output="screen",
            arguments=spec["server_args"],
        )]

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                moveit_config.planning_pipelines,
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        ),
        *executors,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="v1arm",
            description=f"which robot to load: {', '.join(sorted(MODELS))}",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="set false to run move_group headless, e.g. for tests",
        ),
        DeclareLaunchArgument(
            "hardware", default_value="false",
            description="true: real controllers (hardware_bridge + dexhand drivers) "
                        "instead of sim_trajectory_server",
        ),
        DeclareLaunchArgument(
            "onboard", default_value="false",
            description="true: the robot's RDK X5 runs bhl_onboard; MoveIt sends "
                        "trajectories to it over the network, nothing local drives motors",
        ),
        DeclareLaunchArgument(
            "live", default_value="false",
            description="hardware only. true: the bridge opens CAN / wrist serial; "
                        "false: dry run, nothing is touched",
        ),
        DeclareLaunchArgument(
            "buses", default_value="all",
            description="hardware+live only: all | can | wrist -- which buses the "
                        "bridge opens; joints on the other stay read-only",
        ),
        DeclareLaunchArgument(
            "bridge_config", default_value="",
            description="hardware only: alternative hardware_joints.yaml for the "
                        "bridge, e.g. a copy with more joints enabled for a dry run",
        ),
        DeclareLaunchArgument(
            "right_hand", default_value="none",
            description="hardware only: none (driver in dry run) | serial | off",
        ),
        DeclareLaunchArgument("right_hand_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument(
            "left_hand", default_value="none",
            description="hardware only: none (driver in dry run) | serial | off",
        ),
        DeclareLaunchArgument("left_hand_port", default_value="/dev/ttyACM1"),
        OpaqueFunction(function=launch_setup),
    ])
