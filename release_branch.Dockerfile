# This dockerfile expects proxies to be set via --build-arg if needed
# It also expects to be contained in the /navigation2 root folder for file copy
#
# Example build command:
# export FROM_IMAGE="ros:foxy"
# export OVERLAY_MIXINS="release ccache"
# docker build -t nav2:release_branch \
#   --build-arg FROM_IMAGE \
#   --build-arg OVERLAY_MIXINS \
#   -f Dockerfile.release_branch ./

ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/overlay_ws

FROM $FROM_IMAGE

RUN rosdep update

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./navigation2

# install overlay dependencies
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
ARG OVERLAY_MIXINS="release ccache"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS

# source overlay from entrypoint
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
