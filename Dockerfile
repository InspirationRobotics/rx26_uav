# The `uav` image: ROS 2 Humble plus what telemetry_bridge and the ground
# station need, and nothing else.
#
#     docker build -t uav .
#
# NO DEVICE SDKs, deliberately. There is no camera or LiDAR on this airframe
# yet, and the ASV's image carries CUDA/TensorRT/depthai only because its
# perception nodes need them. Adding them "for later" costs a multi-GB image
# and a longer build on every Jetson that will never open a camera.
#
# MAVPROXY IS NOT IN HERE. It owns the Pixhawk serial device and runs on the
# HOST under uav-mavproxy.service; this container consumes its UDP rebroadcast
# on 14541. Installing MAVProxy here would invite someone to run a second one.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-yaml \
      ros-humble-rclpy \
      ros-humble-std-msgs \
      ros-humble-std-srvs \
      ros-humble-rcl-interfaces \
      ros-humble-launch-ros \
      ros-humble-rosidl-default-generators \
      python3-colcon-common-extensions \
      procps \
      iproute2 \
 && rm -rf /var/lib/apt/lists/*

# procps is not optional here: scripts/run_in_container.sh uses pkill/pgrep to
# sweep orphaned nodes, and without them `systemctl restart` leaves a second
# copy running and the unit fails with an address-in-use that reads like a
# crash. See that script's header.

# pymavlink is the only thing telemetry_bridge needs beyond ROS.
RUN python3 -m pip install --no-cache-dir pymavlink

WORKDIR /root/robotx_ws

# The workspace is BIND-MOUNTED at runtime, not copied in:
#   docker run -v ~/robotx_ws:/root/robotx_ws ...
# so a `git pull` on the host is visible in here with no rebuild of the image.
# Copying the source into the image instead would mean rebuilding the image for
# every code change, which is the workflow this whole arrangement avoids.

CMD ["tail", "-f", "/dev/null"]
