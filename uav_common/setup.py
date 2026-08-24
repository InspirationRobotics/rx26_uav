from setuptools import find_packages, setup

package_name = "uav_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Team Inspiration",
    maintainer_email="chase001cz@gmail.com",
    description="Shared plumbing for rx26_uav ROS 2 nodes",
    url="https://github.com/InspirationRobotics/rx26_uav",
    license="MIT",
    entry_points={
        "console_scripts": [
        ],
    },
)
