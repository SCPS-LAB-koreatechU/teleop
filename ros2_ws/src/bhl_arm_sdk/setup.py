from glob import glob

from setuptools import find_packages, setup

package_name = "bhl_arm_sdk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SCPS Lab",
    maintainer_email="jeong510@koreatech.ac.kr",
    description="Per-motor angle control of the Berkeley Humanoid Lite arms over CAN.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_controller = bhl_arm_sdk.node:main",
            "slider_gui = bhl_arm_sdk.slider_gui:main",
            "arm_sequence = bhl_arm_sdk.sequence:main",
        ],
    },
)
