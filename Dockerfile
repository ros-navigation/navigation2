# This dockerfile can be configured via --build-arg
# Build context must be the /navigation2 root folder for COPY.
# Example build command:
# export UNDERLAY_MIXINS="debug ccache"
# export OVERLAY_MIXINS="debug ccache coverage-gcc"
# docker build -t nav2:latest \
#   --build-arg UNDERLAY_MIXINS \
#   --build-arg OVERLAY_MIXINS ./
ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for dependabot
# https://github.com/dependabot/dependabot-core/issues/2057
FROM osrf/ros2:testing-20210605003201 AS from_image

# multi-stage for caching
FROM from_image AS cacher

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./tools/underlay.repos ../
RUN vcs import ./ < ../underlay.repos

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./navigation2

# copy manifests for caching
WORKDIR /opt
RUN find . -name "src" -type d \
      -mindepth 1 -maxdepth 2 -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/opt/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/opt && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM from_image AS builder

# config dependencies install
ARG DEBIAN_FRONTEND=noninteractive
RUN echo '\
APT::Install-Recommends "0";\n\
APT::Install-Suggests "0";\n\
' > /etc/apt/apt.conf.d/01norecommend

# install CI dependencies
ARG RTI_NC_LICENSE_ACCEPTED=yes
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
      ccache \
      lcov \
      python3-pip \
      ros-$ROS_DISTRO-rmw-fastrtps-cpp \
      ros-$ROS_DISTRO-rmw-connextdds \
      ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    && pip3 install \
      fastcov \
      git+https://github.com/ruffsl/colcon-cache.git@c1cedadc1ac6131fe825d075526ed4ae8e1b473c \
      git+https://github.com/ruffsl/colcon-clean.git@87dee2dd1e47c2b97ac6d8300f76e3f607d19ef6 \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# install underlay dependencies
ARG UNDERLAY_WS
ENV UNDERLAY_WS $UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY --from=cacher /tmp/$UNDERLAY_WS ./
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --skip-keys " \
        slam_toolbox \
        " \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build underlay source
COPY --from=cacher $UNDERLAY_WS ./
ARG UNDERLAY_MIXINS="release ccache"
ARG FAIL_ON_BUILD_FAILURE=True
ARG CCACHE_DIR="$UNDERLAY_WS/.ccache"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon cache lock && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS \
      --event-handlers console_direct+ \
    || ([ -z "$FAIL_ON_BUILD_FAILURE" ] || exit 1)

# install overlay dependencies
ARG OVERLAY_WS
ENV OVERLAY_WS $OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS ./
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --skip-keys " \
        slam_toolbox \
        "\
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# multi-stage for testing
FROM builder AS tester

# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"
ARG CCACHE_DIR="$OVERLAY_WS/.ccache"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon cache lock && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
    || ([ -z "$FAIL_ON_BUILD_FAILURE" ] || exit 1)

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=True
RUN if [ -n "$RUN_TESTS" ]; then \
        . install/setup.sh && \
        colcon test && \
        colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi
