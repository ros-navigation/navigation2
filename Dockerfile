# This dockerfile expects proxies to be set via --build-arg if needed
# It also expects to be contained in the /navigation2 root folder for file copy
# Example build command:
# export CMAKE_BUILD_TYPE=Debug
# docker build -t nav2:latest --build-arg CMAKE_BUILD_TYPE ./
ARG FROM_IMAGE=osrf/ros2:nightly
FROM $FROM_IMAGE

# install CI dependencies	
RUN apt-get update && \
    apt-get install -q -y \	
      ccache \
    && rm -rf /var/lib/apt/lists/*

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
ARG CMAKE_BUILD_TYPE=Release
ARG FAIL_ON_BUILD_FAILURE=True
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache || \
    touch build_failed && \
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
ARG COVERAGE_ENABLED=False
RUN rm $OVERLAY_WS/src/navigation2/nav2_system_tests/COLCON_IGNORE
RUN . $UNDERLAY_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
         -DCMAKE_C_COMPILER_LAUNCHER=ccache \
         -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
         -DCOVERAGE_ENABLED=$COVERAGE_ENABLED || \
    touch build_failed && \
    if [ -f build_failed ] && [ -n "$FAIL_ON_BUILD_FAILURE" ]; then \
      exit 1; \
    fi

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
