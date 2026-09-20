from setuptools import find_packages, setup

package_name = 'roambot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='chaitanyabelekar59@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "inventory_tag_detector = "
            "roambot_perception.inventory_tag_detector:main",
            "inventory_scan_mission = "
            "roambot_perception.inventory_scan_mission:main",
            "service_dispatch_mission = "
            "roambot_perception.service_dispatch_mission:main",
            "warehouse_task_coordinator = "
            "roambot_perception.warehouse_task_coordinator:main",
            "doorway_traffic_manager = "
            "roambot_perception.doorway_traffic_manager:main",
            "warehouse_mission_console = "
            "roambot_perception.warehouse_mission_console:main",
        ],
    },
)
