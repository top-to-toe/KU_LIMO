import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hello_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lee",
    maintainer_email="hansollee333@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hello_ros = hello_ros2.hello_ros:main",
            "move_turtle = hello_ros2.move_turtle:main",
            "simple_sub = hello_ros2.simple_sub:main",
            "simple_pub = hello_ros2.simple_pub:main",
            "simple_service_server = hello_ros2.simple_service_server:main",
            "simple_service_server2 = hello_ros2.simple_service_server2:main",
            "simple_service_client = hello_ros2.simple_service_client:main",
        ],
    },
)