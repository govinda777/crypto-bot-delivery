from setuptools import find_packages, setup
import os
from glob import glob

package_name = "delivery_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files from the launch directory
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.xml")),
        ),
        # Include all config files from the config directory
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.rviz")),
        ),
        # Include all resource files from the resource directory
        (
            os.path.join("share", package_name, "resource"),
            glob(os.path.join("resource", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="todo.todo@example.com",
    description="ROS2 package for crypto-bot-delivery navigation",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Example: 'my_node = delivery_navigation.my_node:main',
        ],
    },
)
