# The `uav` image: ROS 2 Humble plus what telemetry_bridge and the ground
# station need, and nothing else.
#
#     docker build -t uav .
#
# DEVICE SDKs: GSTREAMER ONLY, AND ONLY BECAUSE A CAMERA ARRIVED.
#
# This file used to say "NO DEVICE SDKs, deliberately… the ASV's image carries
# CUDA/TensorRT/depthai only because its perception nodes need them." The
# condition in that sentence is now met: a SIYI A8 mini is fitted and uav_camera
# opens it. So GStreamer and OpenCV are here, and the rule they were an instance
# of still stands — you add a device SDK when something in this workspace opens
# that device, not before.
#
# WHAT IS STILL NOT HERE: CUDA, TensorRT and any inference runtime. camera_node
# streams, records and serves video; it does not detect anything. When the
# perception node lands it will need them and that will be the argument for
# adding them, in this comment, at that time.
#
# HARDWARE DECODE COMES FROM THE HOST, NOT FROM THIS IMAGE. On L4T the NVIDIA
# container runtime bind-mounts the Tegra userspace — including the
# nvv4l2decoder and nvvidconv GStreamer plugins — into the container. That is
# why the run command needs `--runtime nvidia` (setup/install_jetson_host.sh
# sets it) and why the base image does not need to be an l4t one. Check it
# landed with:
#     docker exec uav_ekko gst-inspect-1.0 nvv4l2decoder
# If that finds nothing, the runtime flag is missing or the container predates
# it — `docker start` cannot add a runtime, so the container must be recreated.
#
# MAVPROXY IS NOT IN HERE. It owns the Pixhawk serial device and runs on the
# HOST under uav-mavproxy.service; this container consumes its UDP rebroadcast
# on 14541. Installing MAVProxy here would invite someone to run a second one.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

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
      iputils-ping \
      python3-gi \
      gir1.2-gstreamer-1.0 \
      gir1.2-gst-plugins-base-1.0 \
      gstreamer1.0-tools \
      gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      gstreamer1.0-libav \
      python3-opencv \
 && rm -rf /var/lib/apt/lists/*

# GStreamer notes, because the package split is not obvious and picking wrong
# produces "no element rtspsrc" three layers from the cause:
#   python3-gi + gir1.2-gstreamer-1.0   the Python bindings uav_camera imports
#   plugins-good                        rtspsrc, rtph265depay, matroskamux
#   plugins-bad                         h265parse
#   plugins-base                        videoconvert, videorate, appsink, tee
#   gstreamer1.0-tools                  gst-inspect-1.0 / gst-launch-1.0, which
#                                       are how you bisect a pipeline that will
#                                       not start without editing Python
# The nvv4l2decoder used for hardware decode is NOT in any of these; it arrives
# from the host via `--runtime nvidia` (see the header).
#
# python3-opencv is the distro build: no CUDA, and that is correct for now.
# camera_node does not process frames, it hands them on. An inference-capable
# build belongs with the perception node that needs it.

# procps is not optional here: scripts/run_in_container.sh uses pkill/pgrep to
# sweep orphaned nodes, and without them `systemctl restart` leaves a second
# copy running and the unit fails with an address-in-use that reads like a
# crash. See that script's header.

# pymavlink is the only thing telemetry_bridge needs beyond ROS.
RUN python3 -m pip install --no-cache-dir pymavlink

# siyi_sdk is not on PyPI and has no packaging, so it cannot be pip-installed.
# Cloned to a fixed path on PYTHONPATH: the pinned commit stays visible here
# rather than buried in a vendored copy of someone else's tree.
RUN apt-get update && apt-get install -y --no-install-recommends git \
 && git clone https://github.com/mzahana/siyi_sdk.git /opt/siyi_sdk \
 && git -C /opt/siyi_sdk checkout b645656b71e9d3fc49e101ac2caa91d924f60b81 \
 && rm -rf /opt/siyi_sdk/.git /var/lib/apt/lists/*
ENV PYTHONPATH=/opt/siyi_sdk:${PYTHONPATH}

WORKDIR /root/robotx_ws

# Source ROS in interactive shells.
#
# `docker exec` DOES NOT run the image's ENTRYPOINT, so /ros_entrypoint.sh never
# fires for `docker exec -it uav bash` and you land in a shell with no ros2 on
# PATH. That is the moment people reach for `docker attach` instead — which
# attaches to PID 1 (`tail -f /dev/null`): silent, unresponsive, and on Ctrl+C
# it kills PID 1 and takes the whole container down, along with every node
# exec'd into it.
#
# The overlay is guarded because it does not exist until the first build, and an
# unguarded source in .bashrc makes every shell open with an error on a fresh
# container.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash' >> /root/.bashrc

# The workspace is BIND-MOUNTED at runtime, not copied in:
#   docker run -v ~/robotx_ws:/root/robotx_ws ...
# so a `git pull` on the host is visible in here with no rebuild of the image.
# Copying the source into the image instead would mean rebuilding the image for
# every code change, which is the workflow this whole arrangement avoids.

CMD ["tail", "-f", "/dev/null"]
