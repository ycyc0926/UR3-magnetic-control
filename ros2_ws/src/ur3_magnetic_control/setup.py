from glob import glob
from setuptools import find_packages, setup


package_name = "ur3_magnetic_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yc",
    maintainer_email="yc@localhost.localdomain",
    description="Safe ROS 2 scaffolding for UR3 visual magnetic control.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "camera_node = ur3_magnetic_control.camera_node:main",
            "target_tracker = ur3_magnetic_control.target_tracker:main",
            "path_publisher = ur3_magnetic_control.path_publisher:main",
            "mock_motor = ur3_magnetic_control.mock_motor:main",
            "safety_supervisor = ur3_magnetic_control.safety_supervisor:main",
            "cartesian_line_move = ur3_magnetic_control.cartesian_line_move:main",
            "cartesian_orientation_move = ur3_magnetic_control.cartesian_orientation_move:main",
            "level_magnet_axis = ur3_magnetic_control.level_magnet_axis:main",
        ],
    },
)
