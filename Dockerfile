# This dockerfile expects proxies to be set via --build-arg if needed
# It also expects to be contained in the /navigation2 root folder for file copy
# Example build command:
# sudo docker build -t nav2:latest --build-arg http_proxy=http://my.proxy.com:### .
FROM osrf/ros2:bouncy-desktop

SHELL ["/bin/bash", "-c"]

# setup keys 
# check if proxy is set and get keys, using proxy if it is set
RUN if [ "$http_proxy" == "" ]; \ 
    then \
      apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116; \
    else \
      apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
      --keyserver-options http-proxy=$http_proxy \
      --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116; \
    fi

# setup sources.list 
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

ENV ROS1_DISTRO melodic
ENV ROS2_DISTRO bouncy
ENV ROSDISTRO_INDEX_URL 'https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'

# update latest package versions
RUN apt-get update

# install ROS1 dependencies
RUN apt-get install -y \
    ros-$ROS1_DISTRO-desktop \ 
    ros-$ROS1_DISTRO-urdf \
    ros-$ROS1_DISTRO-interactive-markers \
    ros-$ROS1_DISTRO-gazebo-ros

# install ROS2 dependencies
RUN apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python-rosdep \
    python3-vcstool \
    wget

# install Fast-RTPS dependencies
RUN apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev

# install map_server dependencies
RUN apt-get install -y \
    libsdl-image1.2 \
    libsdl-image1.2-dev \
    libsdl1.2debian \
    libsdl1.2-dev

# get the latest nightly ROS2 build -> ros2_ws/ros2_linux
WORKDIR /ros2_ws
RUN wget -nv -t 5 https://ci.ros2.org/view/packaging/job/packaging_linux/lastSuccessfulBuild/artifact/ws/ros2-package-linux-x86_64.tar.bz2
RUN tar -xjf ros2-package-linux-x86_64.tar.bz2

# clone navigation2 repo
WORKDIR /ros2_ws/navigation2_ws/src
ARG USER=ros-planning
RUN git clone https://github.com/$USER/navigation2.git

# change to correct branch if $BRANCH is not = master
WORKDIR /ros2_ws/navigation2_ws/src/navigation2
ARG PULLREQ=false
ARG BRANCH=master
RUN echo "pullreq is $PULLREQ"
RUN if [ "$PULLREQ" == "false" ] && [ "$BRANCH" == "master" ]; then \ 
      echo "No pull request number given - defaulting to master branch"; \
    elif [ "$BRANCH" != "master" ]; then \
      cd navigation2; \
      git fetch origin $BRANCH:temp_branch; \
      git checkout temp_branch; \
      cd -; \
      echo "No pull request number given - defaulting to $BRANCH branch"; \
    else \
      git fetch origin pull/$PULLREQ/head:pr_branch; \
      git checkout pr_branch; \
    fi

# Download dependencies
RUN echo "Downloading the ROS 2 navstack dependencies workspace"
WORKDIR /ros2_ws/navstack_dependencies_ws/src
RUN vcs import . < /ros2_ws/navigation2_ws/src/navigation2/tools/ros2_dependencies.repos

# Build ROS 2 dependencies
WORKDIR /ros2_ws/navstack_dependencies_ws
RUN rosdep init && rosdep update && rosdep install -q -y -r --from-paths src --ignore-src --rosdistro $ROS2_DISTRO --as-root=apt:false --as-root=pip:false --skip-keys "catkin"
# source 
RUN (source /opt/ros/$ROS2_DISTRO/setup.bash && . /ros2_ws/ros2-linux/setup.bash && colcon build --symlink-install)

# Build navigation2 code
WORKDIR /ros2_ws/navigation2_ws
RUN rosdep install -q -y -r --from-paths src --ignore-src --rosdistro $ROS2_DISTRO --as-root=apt:false --as-root=pip:false
RUN (. /ros2_ws/navstack_dependencies_ws/install/setup.bash && colcon build --symlink-install)

CMD ["bash"]
