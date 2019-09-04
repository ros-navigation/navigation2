# This dockerfile can be configured via --build-arg
# Build context must be the /navigation2 root folder for COPY.
# Example build command:
# export UNDERLAY_MIXINS="debug ccache"
# export OVERLAY_MIXINS="debug ccache coverage"
# docker build -t nav2:latest \
#   --build-arg UNDERLAY_MIXINS \
#   --build-arg OVERLAY_MIXINS ./
ARG FROM_IMAGE=osrf/ros2:nightly
FROM $FROM_IMAGE

# install CI dependencies	
RUN apt-get update && \
    apt-get install -q -y \	
      ccache \
      lcov \
      python3-colcon-mixin \
    && rm -rf /var/lib/apt/lists/*

# setup colcon mixin / meta
RUN colcon mixin add upstream \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add upstream \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# clone underlay source
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS
COPY ./tools/ros2_dependencies.repos ./
RUN vcs import src < ros2_dependencies.repos

# install underlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build underlay source
ARG UNDERLAY_MIXINS="release ccache"
ARG FAIL_ON_BUILD_FAILURE=True
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
        $UNDERLAY_MIXINS \
    || touch build_failed && \
    if [ -f build_failed ] && [ -n "$FAIL_ON_BUILD_FAILURE" ]; then \
      exit 1; \
    fi

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./ src/navigation2/

# install overlay dependencies
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        $UNDERLAY_WS/src \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
ARG OVERLAY_MIXINS="release ccache"
RUN rm $OVERLAY_WS/src/navigation2/nav2_system_tests/COLCON_IGNORE
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
        $OVERLAY_MIXINS \
    || touch build_failed && \
    if [ -f build_failed ] && [ -n "$FAIL_ON_BUILD_FAILURE" ]; then \
      exit 1; \
    fi

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
