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
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yc",
    maintainer_email="yc@localhost.localdomain",
    description="Reusable guarded UR3 vision and magnet-centre control.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "camera_node = ur3_magnetic_control.camera_node:main",
            "h_tracker_node = ur3_magnetic_control.h_tracker_node:main",
            "magnet_trajectory = ur3_magnetic_control.magnet_trajectory:main",
        ],
    },
)
