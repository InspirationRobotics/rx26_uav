from setuptools import find_packages, setup

package_name = "uav_camera"

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
    description="UAV gimbal camera gateway (SIYI A8 mini: stream, record, nadir)",
    url="https://github.com/InspirationRobotics/rx26_uav",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_node = uav_camera.camera_node:main",
        ],
    },
)
