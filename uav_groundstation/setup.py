from setuptools import find_packages, setup

package_name = "uav_groundstation"

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
    description="UAV ground station: one web page, plus the OCS heartbeat",
    url="https://github.com/InspirationRobotics/rx26_uav",
    license="MIT",
    entry_points={
        "console_scripts": [
            # NOT PROVEN IN FLIGHT. Both carry NOTE headers. They are entry
            # points because a ground station that cannot be started is not a
            # ground station, and because systemd starts both by name.
            "ground_station = uav_groundstation.gcs_node:main",
            "ocs_client = uav_groundstation.ocs_client_node:main",
            # NOT PROVEN IN FLIGHT, and not in core.launch.py. It is an
            # entry point because the state nobody can start is no state.
            "mission_planner = uav_groundstation.mission_planner:main",
        ],
    },
)
