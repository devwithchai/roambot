from glob import glob

from setuptools import find_packages, setup

package_name = "roambot_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="chaitanyabelekar59@gmail.com",
    description="Unified Gazebo and Nav2 launch package for RoamBot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [],
    },
)