# This dockerfile expects to be contained in the /navigation2 root folder for file copy
#
# Example build command:
# This determines which version of the ROS2 code base to pull
# export ROS2_BRANCH=master
# docker build \
#   --no-cache \
#   --tag nav2:full_ros_build \
#   --file full_ros_build.Dockerfile ./
#
# Omit the `--no-cache` if you know you don't need to break the cache.
# We're only building on top of a ros2 devel image to get the basics
# prerequisites installed such as the apt source, rosdep, etc. We don't want to
# actually use any of the ros release packages. Instead we are going to build
# everything from source in one big workspace.

ARG FROM_IMAGE=osrf/ros2:devel

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# clone underlay source
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS
COPY ./tools/ros2_dependencies.repos ./
RUN vcs import src < ros2_dependencies.repos

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./ src/navigation2

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp

# multi-stage for building
FROM $FROM_IMAGE AS build

# install packages
RUN apt-get update && apt-get install -q -y \
      libasio-dev \
      libtinyxml2-dev \
      wget \
    && rm -rf /var/lib/apt/lists/*

ARG ROS2_BRANCH=master
ENV ROS2_BRANCH=$ROS2_BRANCH
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

WORKDIR $ROS2_WS

# get ros2 source code
RUN wget https://raw.githubusercontent.com/ros2/ros2/$ROS2_BRANCH/ros2.repos \
    && vcs import src < ros2.repos

# get skip keys
COPY ./tools/skip_keys.txt ./

RUN rosdep update

# copy underlay manifests
COPY --from=cache /tmp/underlay_ws src/underlay
RUN cd src/underlay && colcon list --names-only | \
      cat > packages.txt && \
    cd ../../ && colcon list --names-only \
      --packages-up-to \
        $(cat src/underlay/packages.txt | xargs) | \
          cat > packages.txt

# install underlay dependencies
RUN apt-get update && rosdep install -y \
      --from-paths src \
      --ignore-src \
      --skip-keys \
        "$(cat skip_keys.txt | xargs)" \
      src/underlay \
    && rm -rf /var/lib/apt/lists/*

# build ros2 source
ARG ROS2_MIXINS="release"
RUN colcon build \
      --symlink-install \
      --mixin \
        $ROS2_MIXINS \
      --packages-up-to \
        $(cat src/underlay/packages.txt | xargs) \
      --packages-skip \
        $(cat src/underlay/packages.txt | xargs) \
      --cmake-args --no-warn-unused-cli

# copy underlay source
COPY --from=cache /opt/underlay_ws src/underlay

# build underlay source
ARG UNDERLAY_MIXINS="release"
RUN colcon build \
      --symlink-install \
      --mixin \
        $UNDERLAY_MIXINS \
      --packages-up-to \
        $(cat src/underlay/packages.txt | xargs) \
      --packages-skip-build-finished \
      --cmake-args --no-warn-unused-cli

# copy overlay manifests
COPY --from=cache /tmp/overlay_ws src/overlay
RUN cd src/overlay && colcon list --names-only | \
      cat > packages.txt && \
    cd ../../ && colcon list --names-only \
      --packages-up-to \
        $(cat src/overlay/packages.txt | xargs) | \
          cat > packages.txt

# install overlay dependencies
RUN apt-get update && rosdep install -y \
      --from-paths src \
      --ignore-src \
      --skip-keys \
        "$(cat skip_keys.txt | xargs)" \
      src/overlay \
    && rm -rf /var/lib/apt/lists/*

# build ros2 source
RUN colcon build \
      --symlink-install \
      --mixin \
        $ROS2_MIXINS \
      --packages-up-to \
        $(cat src/overlay/packages.txt | xargs) \
      --packages-skip \
        $(cat src/overlay/packages.txt | xargs) \
      --packages-skip-build-finished \
      --cmake-args --no-warn-unused-cli

# copy overlay source
COPY --from=cache /opt/overlay_ws src/overlay

# build overlay source
ARG OVERLAY_MIXINS="build-testing-on release"
RUN colcon build \
      --symlink-install \
      --mixin \
        $OVERLAY_MIXINS \
      --packages-up-to \
        $(cat src/overlay/packages.txt | xargs) \
      --packages-skip-build-finished \
      --cmake-args --no-warn-unused-cli

# test overlay source
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE
RUN if [ ! -z "$RUN_TESTS" ]; then \
        colcon test \
          --packages-select \
            $(cat src/overlay/packages.txt | xargs); \
        if [ ! -z "$FAIL_ON_TEST_FAILURE" ]; then \
            colcon test-result; \
        else \
            colcon test-result || true; \
        fi \
    fi
