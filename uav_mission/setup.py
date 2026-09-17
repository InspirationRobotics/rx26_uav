from setuptools import find_packages, setup

package_name = "uav_mission"

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
    description="The Task 1 buoy search: sweep the fence, hover over unknown "
                "buoys until their state locks, RTL when all are found",
    url="https://github.com/InspirationRobotics/rx26_uav",
    license="MIT",
    entry_points={
        "console_scripts": [
            "search_node = uav_mission.search_node:main",
        ],
    },
)
