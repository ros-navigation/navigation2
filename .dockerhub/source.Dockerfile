# syntax=docker/dockerfile:experimental

# Use experimental buildkit for faster builds
# https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/experimental.md
# Use `--progress=plain` to use plane stdout for docker build
#
# Example build command:
# This determines which version of the ROS2 code base to pull
# export ROS2_BRANCH=main
# export DOCKER_BUILDKIT=1
# docker build \
#   --tag nav2:source \
#   --file source.Dockerfile ../
#
# Omit the `--no-cache` if you know you don't need to break the cache.
# We're only building on top of a ros2 devel image to get the basics
# prerequisites installed such as the apt source, rosdep, etc. We don't want to
# actually use any of the ros release packages. Instead we are going to build
# everything from source in one big workspace.

ARG FROM_IMAGE=osrf/ros2:devel
ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone ros2 source
ARG ROS2_BRANCH=master
ARG ROS2_REPO=https://github.com/ros2/ros2.git
WORKDIR $ROS2_WS/src
RUN git clone $ROS2_REPO -b $ROS2_BRANCH && \
    vcs import ./ < ros2/ros2.repos && \
    find ./ -name ".git" | xargs rm -rf

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./tools/ros2_dependencies.repos ../
RUN vcs import ./ < ../ros2_dependencies.repos && \
    find ./ -name ".git" | xargs rm -rf

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./ros-planning/navigation2
RUN colcon list --names-only | cat > ../packages.txt

# remove skiped packages
WORKDIR /opt
RUN find ./ \
      -name "AMENT_IGNORE" -o \
      -name "CATKIN_IGNORE" -o \
      -name "COLCON_IGNORE" \
      | xargs dirname | xargs rm -rf || true && \
    colcon list --paths-only \
      --packages-skip-up-to  \
        $(cat $OVERLAY_WS/packages.txt | xargs) \
      | xargs rm -rf

# copy manifests for caching
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt

# multi-stage for building
FROM $FROM_IMAGE AS builder

# edit apt for caching
RUN cp /etc/apt/apt.conf.d/docker-clean /etc/apt/ && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/docker-clean

# install packages
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -q -y \
      ccache \
      libasio-dev \
      libtinyxml2-dev \
    && rosdep update

ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

# install ros2 dependencies
WORKDIR $ROS2_WS
COPY --from=cacher /tmp/$ROS2_WS ./
COPY ./tools/skip_keys.txt /tmp/
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
      --skip-keys \
        "$(cat /tmp/skip_keys.txt | xargs)"

# build ros2 source
COPY --from=cacher $ROS2_WS ./
ARG ROS2_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    colcon build \
      --symlink-install \
      --mixin $ROS2_MIXINS

# install underlay dependencies
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY --from=cacher /tmp/$UNDERLAY_WS ./
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . $ROS2_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
        $ROS2_WS/src \
      --ignore-src \
      --skip-keys " \
        $(cat /tmp/skip_keys.txt | xargs) \
      "

# build underlay source
COPY --from=cacher $UNDERLAY_WS ./
ARG UNDERLAY_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . $ROS2_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS ./
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
        $ROS2_WS/src \
        $UNDERLAY_WS/src \
      --ignore-src \
      --skip-keys " \
        $(cat /tmp/skip_keys.txt | xargs) \
      "

# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"
RUN --mount=type=cache,target=/root/.ccache \
    . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS

# restore apt for docker
RUN mv /etc/apt/docker-clean /etc/apt/apt.conf.d/ && \
    rm -rf /var/lib/apt/lists/

# source overlay from entrypoint
ENV UNDERLAY_WS $UNDERLAY_WS
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# # test overlay source
# ARG RUN_TESTS
# ARG FAIL_ON_TEST_FAILURE
# RUN if [ ! -z "$RUN_TESTS" ]; then \
#         colcon test \
#           --packages-select \
#             $(cat src/overlay/packages.txt | xargs); \
#         if [ ! -z "$FAIL_ON_TEST_FAILURE" ]; then \
#             colcon test-result; \
#         else \
#             colcon test-result || true; \
#         fi \
#     fi
