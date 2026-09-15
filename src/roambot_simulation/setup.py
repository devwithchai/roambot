from glob import glob
from setuptools import find_packages, setup

package_name = "roambot_simulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/models/inventory_tag_10", ["models/inventory_tag_10/model.config", "models/inventory_tag_10/model.sdf"]),
        ("share/" + package_name + "/models/inventory_tag_10/materials/textures", glob("models/inventory_tag_10/materials/textures/*.png")),
        ("share/" + package_name + "/models/inventory_tag_11", ["models/inventory_tag_11/model.config", "models/inventory_tag_11/model.sdf"]),
        ("share/" + package_name + "/models/inventory_tag_11/materials/textures", glob("models/inventory_tag_11/materials/textures/*.png")),
        ("share/" + package_name + "/models/inventory_tag_12", ["models/inventory_tag_12/model.config", "models/inventory_tag_12/model.sdf"]),
        ("share/" + package_name + "/models/inventory_tag_12/materials/textures", glob("models/inventory_tag_12/materials/textures/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chaitanya",
    maintainer_email="chaitanya@example.com",
    description="Gazebo Harmonic simulation for RoamBot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)