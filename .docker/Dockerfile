# syntax=docker/dockerfile:1.7
ARG DEV_FROM_STAGE=tooler
ARG EXPORT_FROM_STAGE=builder
ARG PREP_FROM_STAGE=runner
ARG SHIP_FROM_STAGE=runner
ARG WS_CACHE_ID=nav2

ARG FROM_IMAGE=base
# Stage from full image tag name for dependabot detection
FROM ros:rolling as base

################################################################################
# MARK: baser - setup base image using snapshots
################################################################################
FROM $FROM_IMAGE as baser
ENV FROM_IMAGE=${FROM_IMAGE}

# Configure ubuntu snapshot
RUN UBUNTU_DEB_SNAPSHOT=$(date -r /var/lib/dpkg/info +%Y%m%dT%H%M%SZ) && \
  sed -i "s|http://archive.ubuntu.com/ubuntu/|http://snapshot.ubuntu.com/ubuntu/${UBUNTU_DEB_SNAPSHOT}|g" /etc/apt/sources.list && \
  sed -i "s|http://security.ubuntu.com/ubuntu/|http://snapshot.ubuntu.com/ubuntu/${UBUNTU_DEB_SNAPSHOT}|g" /etc/apt/sources.list

# Edit apt config for caching
RUN mv /etc/apt/apt.conf.d/docker-clean /etc/apt/ && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache && \
    # Given fixed snapshots, just cache apt update once
    apt-get update && echo v1

# Configure overlay workspace
ENV OVERLAY_WS=/opt/nav2_ws
WORKDIR $OVERLAY_WS

# install bootstrap tools
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get install -y --no-install-recommends \
      gettext-base \
      wget \
      zstd

# init and update rosdep
ENV ROS_HOME=/opt/.ros
RUN rosdep update \
      --rosdistro $ROS_DISTRO

################################################################################
# MARK: cacher - cache source and dependency instructions
################################################################################
FROM baser AS cacher

# clone underlay source
COPY ./tools/underlay.repos ./
RUN mkdir src && \
    vcs import src < underlay.repos

# copy overlay source
COPY ./ ./src/nav2

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN dep_types=(\
      "exec:--dependency-types=exec" \
      "test:--dependency-types=test" \
      "build:"\
    ) && \
    for dep_type in "${dep_types[@]}"; do \
      IFS=":"; set -- $dep_type; \
      rosdep install -y \
        --from-paths src \
        --ignore-src \
        --skip-keys " \
          # From Nav2
            slam_toolbox \
            turtlebot3_gazebo \
          "\
        --reinstall \
        --simulate \
        ${2} \
        | sed -E "s/'//g; s/ \(alternative .*\)//g" \
        | tail -n +2 \
        | awk '{print $NF}' \
        | sort > /tmp/rosdep_${1}_debs.txt; \
    done

################################################################################
# MARK: runner - setup runtime dependencies for deployment
################################################################################
FROM baser as runner

# install packages for field work
COPY .docker/runner_apt_debs.txt /tmp/runner_apt_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    cut -d# -f1 < /tmp/runner_apt_debs.txt | envsubst \
      | xargs apt-get install -y --no-install-recommends

COPY --from=cacher /tmp/rosdep_exec_debs.txt /tmp/rosdep_exec_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    < /tmp/rosdep_exec_debs.txt xargs apt-get install -y --no-install-recommends

################################################################################
# MARK: prepper - bootstrap general dependencies for development
################################################################################
FROM $PREP_FROM_STAGE as prepper

# install bootstrap tools
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get install -y --no-install-recommends \
      git \
      python3-colcon-clean \
      python3-pip \
    && pip3 install --break-system-packages \
      git+https://github.com/ruffsl/colcon-cache.git@6076815bbb574da028d270cf6eb93bdd5b29c7f4
ENV COLCON_EXTENSION_BLOCKLIST="$COLCON_EXTENSION_BLOCKLIST:colcon_core.package_augmentation.cache_git"

# setup colcon mixin and metadata
ENV COLCON_HOME=/opt/.colcon
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

################################################################################
# MARK: validator - setup test dependencies for validation 
################################################################################
FROM prepper as validator

COPY --from=cacher /tmp/rosdep_test_debs.txt /tmp/rosdep_test_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    < /tmp/rosdep_test_debs.txt xargs apt-get install -y --no-install-recommends

################################################################################
# MARK: tooler - setup build dependencies for compilation
################################################################################
FROM validator as tooler

COPY --from=cacher /tmp/rosdep_build_debs.txt /tmp/
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    < /tmp/rosdep_build_debs.txt xargs apt-get install -y --no-install-recommends

# install packages for build work
COPY .docker/tooler_apt_deps.txt /tmp/tooler_apt_deps.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
  cut -d# -f1 < /tmp/tooler_apt_deps.txt | envsubst \
    | xargs apt-get install -y --no-install-recommends

# setup default ccache configuration
COPY .docker/.ccache/ccache.conf /opt/.ccache/
ENV CCACHE_CONFIGPATH="/opt/.ccache/ccache.conf"
ENV CCACHE_DIR="$OVERLAY_WS/.ccache"

# setup default colcon configuration
COPY .docker/.colcon/defaults.yaml $COLCON_HOME/
ARG OVERLAY_MIXINS="release ccache lld"
ENV OVERLAY_MIXINS=${OVERLAY_MIXINS}

# capture environment to modify layer digest
RUN env > /tmp/env.txt

################################################################################
# MARK: seeder - seed workspace artifacts for caching
################################################################################
FROM baser as seeder
ARG WS_CACHE_ID

ARG CLEAR_WS_CACHE
RUN --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    if [ -n "$CLEAR_WS_CACHE" ]; then \
      echo "Clearing cache!" && \
      rm -rf $OVERLAY_WS/* && \
      echo "Cache cleared!"; \
    fi

ARG SEED_WS_CACHE
RUN --mount=type=bind,source=./,target=/seeder/\
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    if [ -n "$SEED_WS_CACHE" ]; then \
      echo "Seeding cache!" && \
      cp -rT /seeder/cache/$OVERLAY_WS $OVERLAY_WS && \
      echo "Cache seeded!"; \
    fi

RUN --mount=type=cache,from=cacher,target=/cacher \
  --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
  rm -rf ./src && \
  cp -r /cacher/$OVERLAY_WS/src ./ && \
  echo $(date) > /tmp/seeder_stamp.txt

################################################################################
# MARK: builder - build workspace artifacts for deployment
################################################################################
FROM tooler as builder
ARG WS_CACHE_ID

# build overlay source
ARG BUST_BUILD_CACHE
RUN --mount=type=cache,from=seeder,target=/seeder \
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon cache lock \
      --dirhash-reset && \
    colcon clean packages -y \
      --packages-select-cache-invalid \
      --packages-select-cache-key build \
      --base-select install && \
    colcon build \
      --packages-skip-cache-valid \
      --mixin $OVERLAY_MIXINS \
      || true && \
    echo $(date) > /tmp/builder_stamp.txt

ARG FAIL_ON_BUILD_FAILURE=1
RUN --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    BUILD_FAILED=$( \
      colcon list \
        --packages-select-build-failed) && \
    if [ -n "$BUILD_FAILED" ]; then \
      echo "BUILD_FAILED: \n$BUILD_FAILED" && \
      ([ -z "$FAIL_ON_BUILD_FAILURE" ] || exit 1); \
    fi

################################################################################
# MARK: dever - setup user account for development
################################################################################
FROM $DEV_FROM_STAGE as dever

# add default user for devcontainer
ENV DEV_USER=ubuntu
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

################################################################################
# MARK: tester - test workspace artifacts for development
################################################################################
FROM validator as tester
ARG WS_CACHE_ID

# test overlay build
ARG BUST_TEST_CACHE
RUN --mount=type=cache,from=builder,target=/builder \
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    . install/setup.sh && \
    colcon test \
      --packages-skip-cache-valid && \
    echo $(date) > /tmp/tester_stamp.txt

ARG FAIL_ON_TEST_FAILURE=1
RUN --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    . install/setup.sh && \
    colcon test-result \
      --verbose \
      || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1)

################################################################################
# MARK: dancer - multi-stage for caches dancing
################################################################################
FROM $EXPORT_FROM_STAGE as dancer
ARG WS_CACHE_ID

RUN --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS,readonly \
    echo "Exporting cache!" && \
    mkdir -p /dancer/$OVERLAY_WS && \
    cp -rT $OVERLAY_WS /dancer/$OVERLAY_WS && \
    echo "Cache exported!"

################################################################################
# MARK: exporter - multi-stage for exporting caches
################################################################################
FROM $EXPORT_FROM_STAGE as exporter

COPY --link --from=dancer /dancer/$OVERLAY_WS $OVERLAY_WS

################################################################################
# MARK: shipper - setup production images using shared instructions 
################################################################################
FROM $SHIP_FROM_STAGE as shipper

################################################################################
# MARK: debugger - stage target for debuggin in production
################################################################################
FROM shipper as debugger

COPY --link --from=dancer /dancer/$OVERLAY_WS $OVERLAY_WS

################################################################################
# MARK: releaser - stage target for releasing in production
################################################################################
FROM shipper as releaser

COPY --link --from=dancer /dancer/$OVERLAY_WS/install $OVERLAY_WS/install