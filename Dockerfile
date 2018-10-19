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

# install basic build script dependencies 
RUN apt-get update
RUN apt-get install -y \
    python3-colcon-common-extensions \
    git \
    wget \
    python3-vcstool

# install ROS1 dependencies
RUN apt-get install -y \
    ros-$ROS1_DISTRO-desktop \ 
    ros-$ROS1_DISTRO-urdf \
    ros-$ROS1_DISTRO-interactive-markers \
    ros-$ROS1_DISTRO-gazebo-ros

# install map_server dependencies
RUN apt-get install -y \
    libsdl-image1.2 \
    libsdl-image1.2-dev \
    libsdl1.2debian \
    libsdl1.2-dev

WORKDIR /ros2_dependencies_ws

# setup build script and run it
# assume script is in tools on build system unless SCRIPTPATH is set
ARG SCRIPTPATH=./tools
COPY $SCRIPTPATH/initial_ros_setup.sh init/
COPY $SCRIPTPATH/*.repos ./

# run setup script to download source
RUN chmod +x init/initial_ros_setup.sh
RUN yes | ./init/initial_ros_setup.sh --no-ros2 --download-only

WORKDIR /ros2_dependencies_ws/navigation2_ws/src/navigation2

# change to correct branch if $BRANCH is not = master
ARG PULLREQ=false
RUN echo "pullreq is $PULLREQ"
RUN if [ "$PULLREQ" == "false" ]; \
    then \
      echo "No pull request number given - defaulting to master branch"; \
    else \
      git fetch origin pull/$PULLREQ/head:pr_branch; \
      git checkout pr_branch; \
    fi

# build
WORKDIR /ros2_dependencies_ws
RUN chmod +x navigation2_ws/src/navigation2/tools/build_all.sh
RUN ./navigation2_ws/src/navigation2/tools/build_all.sh
CMD ["bash"]
