# This dockerfile expects proxies to be set via --build-arg if needed
# It also expects to be contained in the /navigation2 root folder for file copy
# Example build command:
# export CMAKE_BUILD_TYPE=Debug
# docker build -t nav2:latest --build-arg CMAKE_BUILD_TYPE ./
ARG FROM_IMAGE=osrf/ros2:nightly
FROM $FROM_IMAGE

# install CI dependencies	
RUN apt-get update && apt-get install -q -y \	
      ccache \	
    && rm -rf /var/lib/apt/lists/*

# copy ros package repo
ENV NAV2_WS /opt/nav2_ws
RUN mkdir -p $NAV2_WS/src
WORKDIR $NAV2_WS/src
COPY ./ navigation2/

# clone underlay package repos
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
RUN vcs import src < $NAV2_WS/src/navigation2/tools/ros2_dependencies.repos

# install underlay package dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build underlay package source
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

# install overlay package dependencies
WORKDIR $NAV2_WS
RUN . $ROS_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        $ROS_WS/src \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay package source
RUN rm $NAV2_WS/src/navigation2/nav2_system_tests/COLCON_IGNORE
ARG COVERAGE_ENABLED=False
RUN . $ROS_WS/install/setup.sh && \
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

# source overlay workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$NAV2_WS/install/setup.bash"|' \
      /ros_entrypoint.sh
