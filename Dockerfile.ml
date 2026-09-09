# ============================================================================
# The `uav:ml` image — Ekko with an inference runtime.
#
#     docker build -t uav:ml -f Dockerfile.ml .
#
# WHY THIS FILE EXISTS, AND WHY IT IS NOT `Dockerfile` YET.
#
# The original Dockerfile says, of CUDA and TensorRT: "When the perception node
# lands it will need them and that will be the argument for adding them, in
# this comment, at that time." That time is now — a YOLO11n buoy detector is
# trained and has to run on this aircraft.
#
# It is a SEPARATE FILE because `uav` is the image the aircraft currently flies
# on and it works. This one is built to its own tag, proven against a real park
# still, and only then promoted. If the build is wrong we lose disk, not a
# sortie. Delete this file and fold it into Dockerfile once uav_ekko has been
# running on it for a few flights.
#
# THE BASE IS INVERTED FROM THE OLD IMAGE, AND THAT IS DELIBERATE.
# Old: ros:humble-ros-base, no CUDA. New: the Ultralytics JetPack 6 image, with
# ROS installed on top. This is not a preference — it is what rx26_asv
# (Crusader) and robotx_graey_2026 (Graey) both already do. Three vehicles, one
# pattern, one place to learn each trap. Getting CUDA + cuDNN + TensorRT + a
# Tegra PyTorch wheel onto a ros: base by hand is possible and is a bad use of
# anyone's week.
#
# WHAT STILL COMES FROM THE HOST, NOT FROM HERE: the Tegra userspace, including
# the nvv4l2decoder and nvvidconv GStreamer plugins and libcuda itself. The
# NVIDIA container runtime bind-mounts them, which is why the run command needs
# `--runtime nvidia`. `docker start` cannot add a runtime, so a container that
# predates the flag must be recreated. Check it landed with:
#     docker exec <ctr> gst-inspect-1.0 nvv4l2decoder
#
# MAVPROXY IS STILL NOT IN HERE. It owns /dev/uav-pixhawk and runs on the HOST
# under uav-mavproxy.service; this container consumes its UDP rebroadcast on
# 14541. Installing it here would invite someone to run a second one.
# ============================================================================
FROM ultralytics/ultralytics:latest-jetson-jetpack6

# ARG not ENV: ENV would persist into the running container, so every later apt
# run inside the aircraft's container would silently accept defaults instead of
# prompting. Build-time only is what we want.
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ----------------------------------------------------------------------------
# ROS 2 Humble apt source. The Ultralytics base ships no ROS repo or key.
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release software-properties-common ca-certificates \
 && add-apt-repository universe \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

# ----------------------------------------------------------------------------
# ROS 2 Humble, the build toolchain, and the GStreamer stack uav_camera opens.
#
# ros-dev-tools brings colcon, rosdep and the rosidl generators — ros-base
# alone cannot build a package that generates messages.
#
# GStreamer package split, because picking wrong produces "no element rtspsrc"
# three layers from the cause:
#   python3-gi + gir1.2-gstreamer-1.0   the Python bindings uav_camera imports
#   plugins-good                        rtspsrc, rtph264depay, matroskamux
#   plugins-bad                         h264parse / h265parse
#   plugins-base                        videoconvert, videorate, appsink, tee
#   gstreamer1.0-tools                  gst-inspect-1.0, which is how you bisect
#                                       a pipeline that will not start
#
# NOTE THE ABSENCE OF python3-opencv. The old image installed it; this one must
# NOT. The Ultralytics base already provides cv2 via pip, and adding the apt
# build puts a second OpenCV on the path. Which one wins is import-order
# roulette, and the apt build has no CUDA. One cv2, from the base.
# ----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-dev-tools \
        python3-colcon-common-extensions \
        ros-humble-std-srvs \
        ros-humble-rcl-interfaces \
        ros-humble-launch-ros \
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
 && rm -rf /var/lib/apt/lists/*

# procps is not optional: tools/scripts/run_in_container.sh uses pkill/pgrep to
# sweep orphaned nodes. Without them `systemctl restart` leaves a second copy
# running and the unit fails with an address-in-use that reads like a crash.

# ----------------------------------------------------------------------------
# Python deps. THE IMAGE IS THE ONLY PLACE RUNTIME DEPS ARE INSTALLED — nothing
# pip-installs into a running container, because that state is undocumented and
# lost on `docker rm`.
#
# NOTHING HERE MAY MOVE PROTOBUF. The Ultralytics base ships protobuf 5.29.x,
# which its TensorFlow requires (<6.0.0dev). An unpinned package that drags in
# a newer protobuf breaks the ML stack the detector runs on — that has already
# happened on Crusader's Jetson, via grpcio-tools. The guard below is what
# catches a recurrence.
#
# pymavlink is PINNED and matches Crusader's pin: it is the wire library for
# telemetry_bridge, and an upgrade that changes message parsing is not
# something to discover in flight.
# ----------------------------------------------------------------------------
#
# SETUPTOOLS IS PINNED, AND NOT FOR TASTE. The Ultralytics base ships
# setuptools 84, which removed `setup.py develop --uninstall`. colcon's
# --symlink-install -- the one blessed build path in rebuild.sh -- calls
# exactly that, so every ament_python package in this workspace fails with
#     error: option --uninstall not recognized
# and the message names neither colcon nor setuptools. 59.6.0 is what
# ros:humble shipped and what Humble expects. Verified on the aircraft that
# torch, ultralytics, cv2 and protobuf are all unaffected by the downgrade.
RUN uv pip install --system \
        "pymavlink==2.4.49" \
        "pyyaml==6.0.3" \
        "setuptools==59.6.0"

# ----------------------------------------------------------------------------
# siyi_sdk is not on PyPI and has no packaging, so it cannot be pip-installed.
# Cloned to a fixed path on PYTHONPATH: the pinned commit stays visible here
# rather than buried in a vendored copy of someone else's tree.
# ----------------------------------------------------------------------------
RUN git clone https://github.com/mzahana/siyi_sdk.git /opt/siyi_sdk \
 && git -C /opt/siyi_sdk checkout b645656b71e9d3fc49e101ac2caa91d924f60b81 \
 && rm -rf /opt/siyi_sdk/.git
ENV PYTHONPATH=/opt/siyi_sdk:${PYTHONPATH}

# ----------------------------------------------------------------------------
# Fail the BUILD, not the aircraft, if the ML stack, the MAVLink stack or the
# camera path is broken. Every one of these is imported by something that flies.
# ----------------------------------------------------------------------------
RUN python3 -c "\
import google.protobuf, torch, ultralytics, cv2, pymavlink, yaml, setuptools; \
import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst; \
v = google.protobuf.__version__; \
assert v.startswith('5.29'), 'protobuf moved to %s — TF requires <6.0.0dev' % v; \
sv = setuptools.__version__; \
assert int(sv.split('.')[0]) < 60, 'setuptools %s breaks colcon --symlink-install' % sv; \
print('dep guard ok: protobuf', v, '| torch', torch.__version__, \
      '| ultralytics', ultralytics.__version__, '| cv2', cv2.__version__, \
      '| pymavlink', pymavlink.__version__)"

# CUDA is NOT asserted at build time on purpose. The builder has no GPU access
# unless the daemon's default runtime is nvidia, so `torch.cuda.is_available()`
# here would fail on a perfectly good image and pass on a mis-run one. It is a
# RUNTIME property of how the container was created, and it is checked there:
#     docker exec <ctr> python3 -c "import torch; print(torch.cuda.is_available())"
# If that prints False, the container was made without `--runtime nvidia`.

# ----------------------------------------------------------------------------
# Environment sourcing: ROS, then the mounted workspace.
#
# `docker exec` DOES NOT run the image's ENTRYPOINT, so you land in a shell with
# no ros2 on PATH unless .bashrc sources it. That is the moment people reach for
# `docker attach` instead — which attaches to PID 1 and on Ctrl+C kills it,
# taking the container down along with every node exec'd into it.
#
# The overlay source is guarded because install/ does not exist until the first
# build, and an unguarded source makes every shell on a fresh container open
# with an error.
# ----------------------------------------------------------------------------
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /root/robotx_ws/install/setup.bash ] && source /root/robotx_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /root/robotx_ws

# The workspace is BIND-MOUNTED at runtime, not copied in:
#   docker run -v ~/robotx_ws:/root/robotx_ws ...
# so a `git pull` on the host is visible in here with no rebuild of the image.

CMD ["tail", "-f", "/dev/null"]
