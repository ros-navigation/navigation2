# This dockerfile can be configured via --build-arg
# Build context must be the /navigation2 root folder for COPY.
# Example build command:
# export UNDERLAY_MIXINS="debug ccache"
# export OVERLAY_MIXINS="debug ccache coverage"
# docker build -t nav2:latest \
#   --build-arg UNDERLAY_MIXINS \
#   --build-arg OVERLAY_MIXINS ./
ARG FROM_IMAGE=osrf/ros2:nightly

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# clone underlay source
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS
COPY ./tools/ros2_minimal_dependencies.repos ./
RUN vcs import src < ros2_minimal_dependencies.repos

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./ src/navigation2

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp

# multi-stage for building
FROM $FROM_IMAGE AS build

# install CI dependencies
RUN apt-get update && apt-get install -q -y \
      ccache \
      lcov \
    && rm -rf /var/lib/apt/lists/*

# copy underlay manifests
ENV UNDERLAY_WS /opt/underlay_ws
COPY --from=cache /tmp/underlay_ws $UNDERLAY_WS
WORKDIR $UNDERLAY_WS

# install underlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy underlay source
COPY --from=cache $UNDERLAY_WS ./

# build underlay source
ARG UNDERLAY_MIXINS="release ccache"
ARG FAIL_ON_BUILD_FAILURE=True
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
        $UNDERLAY_MIXINS \
      --event-handlers console_direct+ \
    || touch build_failed && \
    if [ -f build_failed ] && [ -n "$FAIL_ON_BUILD_FAILURE" ]; then \
      exit 1; \
    fi

# copy overlay manifests
ENV OVERLAY_WS /opt/overlay_ws
COPY --from=cache /tmp/overlay_ws $OVERLAY_WS

WORKDIR $OVERLAY_WS
COPY ./minimal_repos.txt ./

# install overlay dependencies
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths $(awk '$0="src/navigation2/"$0' ./minimal_repos.txt) \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# copy overlay source
COPY --from=cache $OVERLAY_WS ./

# build overlay source
ARG OVERLAY_MIXINS="release ccache"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin \
        $OVERLAY_MIXINS \
      --packages-select $(cat ./minimal_repos.txt) \
    || touch build_failed && \
    if [ -f build_failed ] && [ -n "$FAIL_ON_BUILD_FAILURE" ]; then \
      exit 1; \
    fi

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
