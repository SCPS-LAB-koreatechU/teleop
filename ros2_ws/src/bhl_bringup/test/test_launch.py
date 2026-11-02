"""Launch-file logic without ROS: python3 -m pytest -q test

`launch`, `launch_ros` and `ament_index_python` are replaced by small stand-ins
that record what each launch file asks for, so this checks which nodes come up
with which parameters / remaps for each combination of arguments -- not that
ROS accepts them. For that, on a machine with ROS 2:

    ros2 launch bhl_bringup robot.launch.py --print
    ros2 launch bhl_bringup demo.launch.py
"""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path

import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_DIR / "launch"


# ------------------------------------------------------------------ stand-ins
class Action:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class LaunchDescription(list):
    def __init__(self, entities):
        super().__init__(entities)


class DeclareLaunchArgument(Action):
    def __init__(self, name, default_value="", description=""):
        super().__init__(name=name, default_value=default_value, description=description)
        self.name, self.default_value = name, default_value


class LogInfo(Action):
    def __init__(self, msg):
        super().__init__(msg=msg)
        self.msg = msg


class OpaqueFunction(Action):
    def __init__(self, function):
        super().__init__(function=function)
        self.function = function


class IncludeLaunchDescription(Action):
    def __init__(self, source, launch_arguments=()):
        super().__init__(source=source, launch_arguments=list(launch_arguments))
        self.source, self.launch_arguments = source, list(launch_arguments)


class PythonLaunchDescriptionSource:
    def __init__(self, path):
        self.path = path


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.launch_configurations[self.name]


class Node(Action):
    def __init__(self, package, executable, name=None, output=None, parameters=None, remappings=None,
                 arguments=None, condition=None, sigterm_timeout=None):
        super().__init__()
        self.package, self.executable, self.name = package, executable, name
        self.parameters = parameters or []
        self.remappings = remappings or []
        self.arguments = arguments or []
        self.sigterm_timeout = sigterm_timeout


class PackageNotFoundError(KeyError):
    pass


class Context:
    def __init__(self, description: LaunchDescription, **overrides):
        self.launch_configurations = {
            e.name: e.default_value for e in description if isinstance(e, DeclareLaunchArgument)}
        unknown = set(overrides) - set(self.launch_configurations)
        assert not unknown, f"not a launch argument: {unknown}"
        self.launch_configurations.update(overrides)


@pytest.fixture
def ros(tmp_path, monkeypatch):
    """Fake ROS: a share directory per package, loaded launch modules."""
    shares = {"bhl_bringup": PACKAGE_DIR, "bhl_arm_sdk": PACKAGE_DIR.parent / "bhl_arm_sdk"}

    def get_package_share_directory(name):
        if name not in shares:
            raise PackageNotFoundError(name)
        return str(shares[name])

    modules = {
        "launch": types.ModuleType("launch"),
        "launch.actions": types.ModuleType("launch.actions"),
        "launch.substitutions": types.ModuleType("launch.substitutions"),
        "launch.launch_description_sources": types.ModuleType("launch.launch_description_sources"),
        "launch_ros": types.ModuleType("launch_ros"),
        "launch_ros.actions": types.ModuleType("launch_ros.actions"),
        "ament_index_python": types.ModuleType("ament_index_python"),
        "ament_index_python.packages": types.ModuleType("ament_index_python.packages"),
    }
    modules["launch"].LaunchDescription = LaunchDescription
    for cls in (DeclareLaunchArgument, LogInfo, OpaqueFunction, IncludeLaunchDescription):
        setattr(modules["launch.actions"], cls.__name__, cls)
    modules["launch.substitutions"].LaunchConfiguration = LaunchConfiguration
    modules["launch.launch_description_sources"].PythonLaunchDescriptionSource = PythonLaunchDescriptionSource
    modules["launch_ros.actions"].Node = Node
    modules["ament_index_python.packages"].PackageNotFoundError = PackageNotFoundError
    modules["ament_index_python.packages"].get_package_share_directory = get_package_share_directory
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    def load(stem):
        spec = importlib.util.spec_from_file_location(f"bhl_bringup_test_{stem}", LAUNCH_DIR / f"{stem}.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def with_description(model="v1arm"):
        share = tmp_path / "berkeley_humanoid_lite_description"
        (share / "urdf").mkdir(parents=True, exist_ok=True)
        files = {"v1arm": "berkeley_humanoid_lite_v1arm.urdf", "stock": "berkeley_humanoid_lite.urdf"}
        path = share / "urdf" / files[model]
        path.write_text(f"<robot name='{model}'/>")
        shares["berkeley_humanoid_lite_description"] = share
        return path

    return types.SimpleNamespace(load=load, with_description=with_description, tmp_path=tmp_path)


def setup(module, **overrides):
    description = module.generate_launch_description()
    context = Context(description, **overrides)
    opaque = [e for e in description if isinstance(e, OpaqueFunction)]
    assert len(opaque) == 1
    return opaque[0].function(context)


def nodes(actions):
    return {a.executable: a for a in actions if isinstance(a, Node)}


def logs(actions):
    return [a.msg for a in actions if isinstance(a, LogInfo)]


# ---------------------------------------------------------------- robot.launch
def test_robot_full_stack_with_description(ros):
    urdf = ros.with_description()
    actions = setup(ros.load("robot"))
    started = nodes(actions)
    assert set(started) == {"arm_controller", "joint_state_publisher", "robot_state_publisher"}
    assert not logs(actions)

    arm = started["arm_controller"]
    assert arm.package == "bhl_arm_sdk"
    assert arm.remappings == [("/joint_states", "/arm_controller/joint_states")]
    assert arm.parameters == [{"dry_run": True, "config": str(PACKAGE_DIR.parent / "bhl_arm_sdk" / "config" / "bhl_arms.yaml")}]
    assert arm.sigterm_timeout == "5"

    jsp = started["joint_state_publisher"]
    assert jsp.arguments == [str(urdf)]
    assert jsp.parameters == [{"source_list": ["/arm_controller/joint_states"], "rate": 50}]

    rsp = started["robot_state_publisher"]
    assert rsp.parameters == [{"robot_description": "<robot name='v1arm'/>", "publish_frequency": 50.0}]


def test_robot_without_description_falls_back_to_the_bare_controller(ros):
    actions = setup(ros.load("robot"))
    started = nodes(actions)
    assert set(started) == {"arm_controller"}
    assert started["arm_controller"].remappings == []          # publishes /joint_states itself
    assert any("no /tf" in m and "berkeley_humanoid_lite_description" in m for m in logs(actions))


def test_robot_live_model_and_explicit_config(ros):
    ros.with_description("stock")
    actions = setup(ros.load("robot"), dry_run="false", model="stock", config="/tmp/arms.yaml", rate="100")
    started = nodes(actions)
    assert started["arm_controller"].parameters == [{"dry_run": False, "config": "/tmp/arms.yaml"}]
    assert started["joint_state_publisher"].arguments[0].endswith("berkeley_humanoid_lite.urdf")
    assert started["joint_state_publisher"].parameters[0]["rate"] == 100
    assert started["robot_state_publisher"].parameters[0]["publish_frequency"] == 100.0


def test_robot_explicit_urdf_wins_and_missing_urdf_is_reported(ros):
    ros.with_description()
    explicit = ros.tmp_path / "mine.urdf"
    explicit.write_text("<robot name='mine'/>")
    started = nodes(setup(ros.load("robot"), urdf=str(explicit), model="stock"))
    assert started["robot_state_publisher"].parameters[0]["robot_description"] == "<robot name='mine'/>"

    actions = setup(ros.load("robot"), urdf="/nowhere/x.urdf")
    assert set(nodes(actions)) == {"arm_controller"}
    assert any("urdf:=/nowhere/x.urdf does not exist" in m for m in logs(actions))


def test_robot_states_from_another_node(ros):
    """arm:=false joint_state_sources:=/bhl/joint_states -- the RDK X5 lowlevel feeds the state stack."""
    ros.with_description()
    actions = setup(ros.load("robot"), arm="false", joint_state_sources="/bhl/joint_states")
    started = nodes(actions)
    assert set(started) == {"joint_state_publisher", "robot_state_publisher"}
    assert started["joint_state_publisher"].parameters[0]["source_list"] == ["/bhl/joint_states"]
    assert not logs(actions)


def test_robot_several_sources_and_warning_when_arm_is_off_but_listed(ros):
    ros.with_description()
    module = ros.load("robot")
    started = nodes(setup(module, joint_state_sources="[/arm_controller/joint_states, /hand/joint_states]"))
    assert started["joint_state_publisher"].parameters[0]["source_list"] == [
        "/arm_controller/joint_states", "/hand/joint_states"]

    actions = setup(module, arm="false")
    assert set(nodes(actions)) == {"joint_state_publisher", "robot_state_publisher"}
    assert any("arm:=false" in m for m in logs(actions))


def test_robot_publish_tf_false_keeps_joint_states_on_the_controller(ros):
    ros.with_description()
    actions = setup(ros.load("robot"), publish_tf="false")
    started = nodes(actions)
    assert set(started) == {"arm_controller"}
    assert started["arm_controller"].remappings == []
    assert not logs(actions)


def test_parse_sources(ros):
    module = ros.load("robot")
    assert module.parse_sources("/a") == ["/a"]
    assert module.parse_sources("/a,/b") == ["/a", "/b"]
    assert module.parse_sources("[/a, /b]") == ["/a", "/b"]
    assert module.parse_sources(" [ /a ] ") == ["/a"]
    assert module.parse_sources("") == []


# --------------------------------------------------------------- remote.launch
def test_remote_default_is_rviz_and_panel(ros):
    ros.with_description()
    actions = setup(ros.load("remote"))
    started = nodes(actions)
    assert set(started) == {"rviz2", "slider_gui"}
    assert started["rviz2"].arguments == ["-d", str(PACKAGE_DIR / "config" / "bringup.rviz")]
    assert started["slider_gui"].package == "bhl_arm_sdk"
    assert not logs(actions)


def test_remote_without_description_warns_but_still_shows_rviz(ros):
    actions = setup(ros.load("remote"), gui="false", rviz_config="/tmp/x.rviz")
    started = nodes(actions)
    assert set(started) == {"rviz2"}
    assert started["rviz2"].arguments == ["-d", "/tmp/x.rviz"]
    assert any("without meshes" in m for m in logs(actions))


def test_remote_can_be_panel_only(ros):
    assert set(nodes(setup(ros.load("remote"), rviz="false"))) == {"slider_gui"}


# ----------------------------------------------------------------- demo.launch
def test_demo_includes_both_halves_and_forwards_every_argument(ros):
    description = ros.load("demo").generate_launch_description()
    includes = [e for e in description if isinstance(e, IncludeLaunchDescription)]
    assert [Path(i.source.path).name for i in includes] == ["robot.launch.py", "remote.launch.py"]
    declared = {e.name for e in description if isinstance(e, DeclareLaunchArgument)}
    forwarded = {name for i in includes for name, _ in i.launch_arguments}
    assert forwarded == declared
    robot_args = {name for name, _ in includes[0].launch_arguments}
    assert robot_args == {"dry_run", "config", "model", "urdf", "joint_state_sources", "rate"}
    assert {name for name, _ in includes[1].launch_arguments} == {"rviz", "gui", "rviz_config"}


def test_demo_forwards_only_arguments_the_halves_declare(ros):
    """Every forwarded name must exist on the included launch file, or ros2 launch rejects it."""
    ros.with_description()
    robot = {e.name for e in ros.load("robot").generate_launch_description() if isinstance(e, DeclareLaunchArgument)}
    remote = {e.name for e in ros.load("remote").generate_launch_description() if isinstance(e, DeclareLaunchArgument)}
    includes = [e for e in ros.load("demo").generate_launch_description() if isinstance(e, IncludeLaunchDescription)]
    assert {n for n, _ in includes[0].launch_arguments} <= robot
    assert {n for n, _ in includes[1].launch_arguments} <= remote


# ---------------------------------------------------------------- shipped files
def test_shipped_files_parse():
    import xml.etree.ElementTree as ET
    import yaml

    rviz = yaml.safe_load((PACKAGE_DIR / "config" / "bringup.rviz").read_text())
    classes = [d["Class"] for d in rviz["Visualization Manager"]["Displays"]]
    assert "rviz_default_plugins/RobotModel" in classes and "rviz_default_plugins/TF" in classes
    assert rviz["Visualization Manager"]["Global Options"]["Fixed Frame"] == "world"

    profiles = ET.parse(PACKAGE_DIR / "config" / "fastdds_unicast.xml").getroot()
    assert profiles.tag.endswith("profiles")

    package = ET.parse(PACKAGE_DIR / "package.xml").getroot()
    assert package.find("name").text == "bhl_bringup"
    assert package.find("export/build_type").text == "ament_python"

    unit = (PACKAGE_DIR / "system" / "bhl-robot@.service").read_text()
    assert "KillSignal=SIGINT" in unit and "robot.launch.py" in unit
    network = (PACKAGE_DIR / "system" / "80-can.network").read_text()
    assert "Name=can*" in network and "BitRate=1M" in network


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
