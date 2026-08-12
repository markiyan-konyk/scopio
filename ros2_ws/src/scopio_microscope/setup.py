import os
from glob import glob

from setuptools import find_packages, setup

package_name = "scopio_microscope"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mark Konyk",
    maintainer_email="mark.konyk@gmail.com",
    description="SCOPIO microscope driver nodes (camera, stage, galvo, "
                "temperature, calibration).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node = scopio_microscope.camera_node:main",
            "stage_node = scopio_microscope.stage_node:main",
            "galvo_node = scopio_microscope.galvo_node:main",
            "temperature_node = scopio_microscope.temperature_node:main",
            "calibration_node = scopio_microscope.calibration_node:main",
            "relay_node = scopio_microscope.relay_node:main",
        ],
    },
)
