"""Drag the hand in RViz, the real arm follows. No MoveIt.

    robot_state_publisher   URDF -> TF, from the merged /joint_states
    joint_state_publisher   bridge's measured joints + zeros for the rest
    hardware_bridge.py      CAN, slew limit, URDF + operator limits, E-STOP
    drag_teleop.py          interactive marker on the hand -> IK -> ~/joint_command
    arm_panel.py            arm diagram, link status, E-STOP button, per-motor limits
    rviz2

    ros2 launch berkeley_humanoid_lite_moveit_config drag_teleop.launch.py            # dry run
    ros2 launch berkeley_humanoid_lite_moveit_config drag_teleop.launch.py live:=true # real arm
    ... model:=dexhand  # or model:=plain, instead of the default full upper body

model:=v1arm (default) loads berkeley_humanoid_lite_v1arm.urdf, the full upper
body MoveIt uses: V1 forearm + 3-DOF wrist + DexHand v2 fingers on BOTH arms.
Each marker drags the arm's DexHand mount link `arm_<side>_index_bulk` (the
SRDF arm tip). The wrist joints are Feetech serial servos (bus `wrist`, not
opened here) and `enabled: false` in the joint map, so IK holds them at their
measured angle -- 0 from joint_state_publisher -- and moves only the CAN joints.
model:=dexhand: berkeley_humanoid_lite_dexhand.urdf, DexHand welded to the
right elbow roll, no forearm; right tip `base_link`.
model:=plain: the bare arms, tips `arm_<side>_hand_link`.

DexHand fingers (models v1arm: both hands, dexhand: right hand) are driven by
dexhand_bringup's hand_driver, the same one demo.launch.py uses, and moved with
finger_panel.py (curl + spread sliders per finger). right_hand / left_hand pick
the link: `none` (default) runs the driver without opening a port, so the
sliders move the hand on screen only; `serial` drives the real board on
right_hand_port / left_hand_port; `off` starts no driver. Hand output starts
off; switch it on in the finger panel. `live:=true` is about the arms only.
The drivers' /joint_states are remapped into joint_state_publisher's source
list so the fingers show up in the one merged /joint_states.

Use ../../run_drag_teleop.sh, which sets ROS_DOMAIN_ID and the recoil PYTHONPATH.
IK moves only the joints `enabled: true` in the joint map that lie on the
chosen arm; the other joints of that arm are held at their measured angles.
"""

from pathlib import Path

import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "berkeley_humanoid_lite_moveit_config"
DESCRIPTION_PACKAGE = "berkeley_humanoid_lite_description"
DRY_RUN_CONFIG = Path("~/.cache/bhl/hardware_joints_dry_run.yaml")
HAND_STATES = {"right": "/hand/right/joint_states", "left": "/hand/left/joint_states"}

# URDF file and the link each arm's marker drags, per model.
MODELS = {
    "v1arm": {"urdf": "berkeley_humanoid_lite_v1arm.urdf", "hands": ("right", "left"),
              "tips": {"left": "arm_left_index_bulk", "right": "arm_right_index_bulk"}},
    "plain": {"urdf": "berkeley_humanoid_lite.urdf", "hands": (),
              "tips": {"left": "arm_left_hand_link", "right": "arm_right_hand_link"}},
    "dexhand": {"urdf": "berkeley_humanoid_lite_dexhand.urdf", "hands": ("right",),
                "tips": {"left": "arm_left_hand_link", "right": "base_link"}},
}


def _servo_params() -> dict | None:
    """dexhand_bringup's servo_map.yaml minus the keys that are not valid ROS
    parameters (as demo.launch.py does); None when the dexhand workspace is not
    sourced."""
    try:
        bringup = Path(get_package_share_directory("dexhand_bringup"))
    except Exception:  # noqa: BLE001 - PackageNotFoundError and friends
        return None
    params = yaml.safe_load((bringup / "config" / "servo_map.yaml").read_text())
    params = params["dexhand_driver"]["ros__parameters"]
    for key in ("servo_cal", "servo_names"):
        params.pop(key, None)
    return params


def _hand_nodes(context, hands: tuple) -> tuple[list, list[str]]:
    """hand_driver per hand (+ the L_/R_ adapter for the left one); returns the
    nodes and the hands that actually got a driver."""
    wanted = [side for side in hands
              if LaunchConfiguration(f"{side}_hand").perform(context) != "off"]
    if not wanted:
        return [], []
    servo = _servo_params()
    if servo is None:
        print("[drag_teleop] dexhand_bringup not found -- fingers are drawn but not driven. "
              "Source ~/Desktop/dexhand_moveit_ws/install/setup.bash (run_drag_teleop.sh does).")
        return [], []
    nodes = []
    for side in wanted:
        link = LaunchConfiguration(f"{side}_hand").perform(context)
        port = LaunchConfiguration(f"{side}_hand_port").perform(context)
        if link not in ("none", "serial"):
            raise RuntimeError(f"{side}_hand must be none, serial or off (got '{link}')")
        params = {**servo, "link": link, "serial_port": port, "auto_enable_output": False}
        if side == "right":
            nodes.append(Node(package="dexhand_bringup", executable="hand_driver",
                              name="dexhand_driver", output="screen", parameters=[params],
                              remappings=[("/joint_states", HAND_STATES["right"])]))
        else:
            # Same as launch/left_hand.launch.py: a second driver that still thinks
            # it is a right hand, mirrored spread, and the adapter renaming R_ -> L_.
            raw = "/left_hand/raw_joint_states"
            params.update(yaw_dir=[-1.0, -1.0, -1.0, -1.0], flex_dir=[1.0, 1.0, 1.0, 1.0])
            nodes.append(Node(package="dexhand_bringup", executable="hand_driver",
                              name="dexhand_driver", namespace="left_hand", output="screen",
                              parameters=[params], remappings=[("/joint_states", raw)]))
            nodes.append(Node(package=PACKAGE, executable="left_hand_adapter.py",
                              name="left_hand_adapter", output="screen",
                              parameters=[{
                                  "driver_action": "/left_hand/hand_controller/follow_joint_trajectory",
                                  "driver_states": raw,
                                  "driver_enable": "/left_hand/dexhand_driver/enable",
                                  "controller_action": "/left_hand_controller/follow_joint_trajectory",
                              }],
                              remappings=[("/joint_states", HAND_STATES["left"])]))
    return nodes, wanted


def launch_setup(context, *_args, **_kwargs):
    share = Path(get_package_share_directory(PACKAGE))
    arm = LaunchConfiguration("arm").perform(context)
    if arm not in ("left", "right", "both"):
        raise RuntimeError(f"arm must be left, right or both (got '{arm}')")
    sides = ["left", "right"] if arm == "both" else [arm]
    live = LaunchConfiguration("live").perform(context).lower() in ("true", "1", "yes")
    config = Path(LaunchConfiguration("config").perform(context)
                  or share / "config" / "hardware_joints_left_distal.yaml")
    limits_file = Path(LaunchConfiguration("limits_file").perform(context)).expanduser()
    model = LaunchConfiguration("model").perform(context)
    if model not in MODELS:
        raise RuntimeError(f"model must be one of {', '.join(MODELS)} (got '{model}')")
    urdf = (Path(get_package_share_directory(DESCRIPTION_PACKAGE)) / "urdf"
            / MODELS[model]["urdf"])
    tips = MODELS[model]["tips"]
    hand_nodes, hands = _hand_nodes(context, MODELS[model]["hands"])

    full_map = yaml.safe_load(config.read_text())
    urdf_joints = {j.get("name") for j in ET.fromstring(urdf.read_text()).findall("joint")}
    wrists = (full_map.get("wrist_servos") or {}).get("joints") or {}
    if not live and wrists and LaunchConfiguration("dry_run_wrist").perform(context).lower() in (
            "true", "1", "yes"):
        # Dry run: nothing is energised, so let the 3-DOF wrist into the solve and
        # the hand's roll/pitch/yaw rings have something to turn. The real servos
        # stay `enabled: false` in the yaml until they are verified on the arm.
        for name, spec in wrists.items():
            if name in urdf_joints:
                spec["enabled"] = True
        config = DRY_RUN_CONFIG.expanduser()
        config.parent.mkdir(parents=True, exist_ok=True)
        config.write_text("# generated by drag_teleop.launch.py for a dry run -- wrist joints enabled\n"
                          + yaml.safe_dump(full_map, sort_keys=False, allow_unicode=True))
    joint_map = {**full_map["joints"], **wrists}
    free = {side: [name for name, spec in joint_map.items()
                   if name.startswith(f"arm_{side}_") and spec.get("enabled")
                   and name in urdf_joints]
            for side in sides}
    sides = [side for side in sides if free[side]]
    if not sides:
        raise RuntimeError(f"{config.name}: no enabled joint on the requested arm(s)")

    bridge_args = ["--buses", "can", "--config", str(config),
                   "--description-node", "/robot_state_publisher",
                   "--limits-file", str(limits_file)]
    if live:
        bridge_args.append("--live")
    if LaunchConfiguration("zero_at_start").perform(context).lower() in ("true", "1", "yes"):
        reference = Path(LaunchConfiguration("reference_file").perform(context)).expanduser()
        if LaunchConfiguration("new_reference").perform(context).lower() in ("true", "1", "yes"):
            reference.unlink(missing_ok=True)
        bridge_args += ["--zero-at-start", "--reference-file", str(reference)]

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": urdf.read_text()}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             output="screen",
             parameters=[{"source_list": ["/hw/joint_states", *[HAND_STATES[h] for h in hands]],
                          "rate": 50}]),
        Node(package=PACKAGE, executable="hardware_bridge.py", output="screen",
             arguments=bridge_args,
             remappings=[("/joint_states", "/hw/joint_states")]),
        # One RViz hand marker per arm, each IK-ing only its own arm.
        *[Node(package=PACKAGE, executable="drag_teleop.py", output="screen",
               name=f"drag_teleop_{side}",
               arguments=["--urdf", str(urdf), "--tip", tips[side],
                          "--name", f"drag_teleop_{side}", "--joints", *free[side],
                          "--limit-margin", str((full_map.get("safety") or {}).get("limit_margin", 0.05))])
          for side in sides],
        Node(package=PACKAGE, executable="arm_panel.py", output="screen",
             arguments=["--limits-file", str(limits_file), "--urdf", str(urdf),
                        "--tips", *[tips[side] for side in sides]]),
        *hand_nodes,
        *([Node(package=PACKAGE, executable="finger_panel.py", output="screen",
                arguments=["--urdf", str(urdf), "--hands", *hands])] if hands else []),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", str(share / "config" / "drag_teleop.rviz")]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("live", default_value="false",
                              description="true: open CAN and drive the motors"),
        DeclareLaunchArgument("zero_at_start", default_value="true",
                              description="take the arm's pose at start-up as 0 for every joint "
                                          "(false: use the yaml offsets)"),
        DeclareLaunchArgument("reference_file", default_value="~/.config/bhl/zero_reference.yaml",
                              description="where the reference taken at start-up is written"),
        # Default true (2026-10-02, the user's call): every start reads the motors and takes
        # the pose they are actually in as the reference. Keeping a days-old stored reference
        # made the reported angles disagree with the real arm -- CAN 10 once came up 30 deg
        # off between two restarts -- and enabling torque on a wrong angle moves the arm.
        # new_reference:=false reuses the stored reference instead.
        DeclareLaunchArgument("new_reference", default_value="true",
                              description="true (default): forget any stored reference and take a "
                                          "new one from the motors' current position at start-up"),
        DeclareLaunchArgument("model", default_value="v1arm",
                              description="v1arm (default): full upper body, forearms + wrists + "
                                          "DexHand on both arms; dexhand: DexHand on the right "
                                          "elbow, no forearm; plain: the bare arms"),
        DeclareLaunchArgument("dry_run_wrist", default_value="true",
                              description="dry run only: put the 3-DOF wrist in the IK so the hand "
                                          "marker's roll/pitch/yaw rings work (live keeps the yaml)"),
        DeclareLaunchArgument("right_hand", default_value="none",
                              description="DexHand right hand link: none (no port, on screen only), "
                                          "serial (real board on right_hand_port) or off"),
        DeclareLaunchArgument("right_hand_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("left_hand", default_value="none",
                              description="DexHand left hand link: none, serial or off"),
        DeclareLaunchArgument("left_hand_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("arm", default_value="both",
                              description="left (CAN 1/3/5/7/9), right (2/4/6/8/10) or both"),
        DeclareLaunchArgument("config", default_value="",
                              description="joint map; default hardware_joints_left_distal.yaml"),
        DeclareLaunchArgument("limits_file",
                              default_value="~/.config/bhl/operator_limits.yaml",
                              description="operator travel limits, written by the panel"),
        OpaqueFunction(function=launch_setup),
    ])
