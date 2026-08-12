from setuptools import find_packages, setup

package_name = "scopio_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mark Konyk",
    maintainer_email="mark.konyk@gmail.com",
    description="HTTP/WebSocket API gateway wrapping the SCOPIO ROS 2 graph.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gateway = scopio_gateway.main:main",
        ],
    },
)
