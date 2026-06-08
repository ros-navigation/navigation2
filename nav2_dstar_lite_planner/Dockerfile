FROM ros:rolling

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rosdep init && rosdep update \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY nav2_dstar_lite_planner /ws/src/nav2_dstar_lite_planner

RUN . /opt/ros/rolling/setup.sh && \
    rosdep install --from-paths src --ignore-src --rosdistro rolling -y && \
    colcon build --packages-select nav2_dstar_lite_planner --symlink-install && \
    . install/setup.sh && \
    colcon test --packages-select nav2_dstar_lite_planner && \
    colcon test-result --verbose
