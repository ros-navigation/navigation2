# This dockerfile expects proxies to be set via --build-arg if needed
# It also expects to be contained in the /navigation2 root folder for file copy
# Example build command:
# sudo docker build -t nav2:latest --build-arg http_proxy=http://my.proxy.com:### .
FROM osrf/ros2:nightly

# setup keys
# check if proxy is set and get keys, using proxy if it is set
RUN if [ "$http_proxy" != "" ]; \
    then \
      apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --keyserver-options http-proxy=$http_proxy \
      --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116; \
    fi

# install ROS2 dependencies
RUN apt-get update && apt-get install -q -y \
      build-essential \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-vcstool \
      wget \
    && rm -rf /var/lib/apt/lists/*

# clone ros package repo
ENV NAV2_WS /opt/nav2_ws
RUN mkdir -p $NAV2_WS/src
WORKDIR $NAV2_WS/src
ARG GIT_REPO_URL=https://github.com/ros-planning/navigation2.git
RUN git clone $GIT_REPO_URL navigation2

# change to correct branch if $BRANCH is not = master
ARG PULLREQ=false
ARG BRANCH=master
RUN echo "pullreq is $PULLREQ"
RUN if [ "$PULLREQ" = "false" ] && [ "$BRANCH" = "master" ]; then \
      echo "No pull request number given - defaulting to master branch"; \
    elif [ "$BRANCH" != "master" ]; then \
      cd navigation2; \
      git fetch origin $BRANCH:temp_branch; \
      git checkout temp_branch; \
      echo "No pull request number given - defaulting to $BRANCH branch"; \
    else \
      cd navigation2; \
      git fetch origin pull/$PULLREQ/head:pr_branch; \
      git checkout pr_branch; \
    fi

# clone dependency package repos
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
RUN vcs import src < $NAV2_WS/src/navigation2/tools/ros2_dependencies.repos

# install dependency package dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build dependency package source
ARG CMAKE_BUILD_TYPE=Release
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# install navigation2 package dependencies
WORKDIR $NAV2_WS
RUN . $ROS_WS/install/setup.sh && \
    apt-get update && \
    rosdep install -q -y \
      --from-paths \
        $ROS_WS/src \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build navigation2 package source
RUN rm $NAV2_WS/src/navigation2/nav2_system_tests/COLCON_IGNORE
RUN . $ROS_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# source navigation2 workspace from entrypoint
RUN sed --in-place --expression \
      '$isource "$NAV2_WS/install/setup.bash"' \
      /ros_entrypoint.sh

ENV ROS_DOMAIN_ID 22
COPY tools/ctest_retry.bash $NAV2_WS/build/nav2_system_tests
