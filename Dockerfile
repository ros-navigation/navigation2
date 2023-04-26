# This dockerfile can be configured via --build-arg
# Build context must be the /navigation2 root folder for COPY.
# Example build command:
# export UNDERLAY_MIXINS="debug ccache lld"
# export OVERLAY_MIXINS="debug ccache coverage-gcc lld"
# docker build -t nav2:latest \
#   --build-arg UNDERLAY_MIXINS \
#   --build-arg OVERLAY_MIXINS ./
ARG FROM_IMAGE=ros:rolling
ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

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
FROM $FROM_IMAGE AS builder

# config dependencies install
ARG DEBIAN_FRONTEND=noninteractive
RUN echo '\
APT::Install-Recommends "0";\n\
APT::Install-Suggests "0";\n\
' > /etc/apt/apt.conf.d/01norecommend
ENV PYTHONUNBUFFERED 1

# install CI dependencies
ARG RTI_NC_LICENSE_ACCEPTED=yes
RUN apt-get update && \
    apt-get upgrade -y --with-new-pkgs && \
    apt-get install -y \
      ccache \
      lcov \
      lld \
      python3-pip \
      ros-$ROS_DISTRO-rmw-fastrtps-cpp \
      ros-$ROS_DISTRO-rmw-connextdds \
      ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    && pip3 install \
      fastcov \
      git+https://github.com/ruffsl/colcon-cache.git@a937541bfc496c7a267db7ee9d6cceca61e470ca \
      git+https://github.com/ruffsl/colcon-clean.git@a7f1074d1ebc1a54a6508625b117974f2672f2a9 \
    && rosdep update \
    && colcon mixin update \
    && colcon metadata update \
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
ARG UNDERLAY_MIXINS="release ccache lld"
ARG CCACHE_DIR="$UNDERLAY_WS/.ccache"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon cache lock && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS \
      --event-handlers console_direct+

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
ARG OVERLAY_MIXINS="release ccache lld"
ARG CCACHE_DIR="$OVERLAY_WS/.ccache"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon cache lock && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE
RUN if [ -n "$RUN_TESTS" ]; then \
        . install/setup.sh && \
        colcon test && \
        colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi

# multi-stage for developing
FROM builder AS dever

# edit apt for caching
RUN mv /etc/apt/apt.conf.d/docker-clean /etc/apt/

# install developer dependencies
RUN apt-get update && \
    apt-get install -y \
      bash-completion \
      gdb

# source underlay for shell
RUN echo 'source "$UNDERLAY_WS/install/setup.bash"' >> /etc/bash.bashrc

# multi-stage for visualizing
FROM dever AS visualizer

# install foxglove dependacies
RUN echo "deb https://dl.cloudsmith.io/public/caddy/stable/deb/debian any-version main" > /etc/apt/sources.list.d/caddy-stable.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 65760c51edea2017cea2ca15155b6d79ca56ea34

# install demo dependencies
RUN apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-aws-robomaker-small-warehouse-world \
      ros-$ROS_DISTRO-rviz2 \
      ros-$ROS_DISTRO-turtlebot3-simulations

# install gzweb dependacies
RUN apt-get install -y --no-install-recommends \
      imagemagick \
      libboost-all-dev \
      libgazebo-dev \
      libgts-dev \
      libjansson-dev \
      libtinyxml-dev \
      nodejs \
      npm \
      psmisc \
      xvfb

# clone gzweb
ENV GZWEB_WS /opt/gzweb
RUN git clone --branch python3 https://github.com/ruffsl/gzweb.git $GZWEB_WS

# build gzweb
RUN cd $GZWEB_WS && . /usr/share/gazebo/setup.sh && \
    GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(find /opt/ros/$ROS_DISTRO/share \
      -mindepth 1 -maxdepth 2 -type d -name "models" | paste -s -d: -) && \
    xvfb-run -s "-screen 0 1280x1024x24" ./deploy.sh -m local && \
    ln -s $GZWEB_WS/http/client/assets http/client/assets/models

# patch gzsever
RUN GZSERVER=$(which gzserver) && \
    mv $GZSERVER $GZSERVER.orig && \
    echo '#!/bin/bash' > $GZSERVER && \
    echo 'xvfb-run -s "-screen 0 1280x1024x24" gzserver.orig "$@"' >> $GZSERVER && \
    chmod +x $GZSERVER

# install foxglove dependacies
RUN apt-get install -y --no-install-recommends \
      caddy \
      ros-$ROS_DISTRO-foxglove-bridge

# copy foxglove
ENV FOXGLOVE_WS /opt/foxglove
COPY --from=ghcr.io/foxglove/studio /src $FOXGLOVE_WS

# multi-stage for exporting
FROM tester AS exporter
