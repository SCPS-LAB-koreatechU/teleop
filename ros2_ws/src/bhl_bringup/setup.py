from glob import glob

from setuptools import find_packages, setup

package_name = "bhl_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.rviz") + glob("config/*.xml")),
        ("share/" + package_name + "/system", glob("system/*")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SCPS Lab",
    maintainer_email="jeong510@koreatech.ac.kr",
    description="Bring-up of the Berkeley Humanoid Lite upper body: on-board state publishing, remote PC tools.",
    license="MIT",
)
