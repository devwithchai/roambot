from setuptools import find_packages, setup
from glob import glob
import os


package_name = "roambot_description"


setup(
    name=package_name,
    version="0.1.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),

        (
            "share/" + package_name,
            ["package.xml"],
        ),

        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),

        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*"),
        ),

        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="Chaitanya",

    maintainer_email="your_email@example.com",

    description=(
        "Learner-focused ROS 2 description package "
        "for the Tri-Rover mobile robot."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [],
    },
)