FROM ubuntu:16.04
MAINTAINER Artem Smirnov "urpylka@gmail.com"

RUN apt-get update && apt-get install -y \
    apt-utils \
    mesa-utils \
    gnupg2 \
    net-tools \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim


# Environment configuration

ENV OS_DISTRO xenial
ENV ROS_DISTRO kinetic
ENV ROS_INSTALL_PATH "/opt/ros/$ROS_DISTRO"
ENV CATKIN_WS /catkin_ws
ENV DRIVERS_WS /drivers_ws

ENV PATH              "$PATH:$ROS_INSTALL_PATH/bin"
ENV PYTHONPATH        "$ROS_INSTALL_PATH/lib/python2.7/dist-packages"
ENV LD_LIBRARY_PATH   "$LD_LIBRARY_PATH:$ROS_INSTALL_PATH/lib"
ENV PKG_CONFIG_PATH   "$PKG_CONFIG_PATH:$ROS_INSTALL_PATH/lib/pkgconfig"
ENV CMAKE_PREFIX_PATH "$ROS_INSTALL_PATH"

RUN echo "source ${ROS_INSTALL_PATH}/setup.bash" >> "/root/.bashrc"
RUN echo "source ${DRIVERS_WS}/devel/setup.bash" >> "/root/.bashrc"
RUN echo "source ${CATKIN_WS}/devel/setup.bash" >> "/root/.bashrc"


# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


# Install ROS desktop and catkin tools

RUN echo "deb http://packages.ros.org/ros/ubuntu $OS_DISTRO main" > /etc/apt/sources.list.d/ros-latest.list && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && apt-get install -y ros-kinetic-desktop python-catkin-tools


# Additional ROS packages

RUN apt install -y \
    ros-kinetic-teleop-twist-keyboard \
    ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-hector-models \
    ros-kinetic-navigation \
    ros-kinetic-explore-lite \
    ros-kinetic-gmapping \
    ros-kinetic-ira-laser-tools \
    ros-kinetic-pointcloud-to-laserscan \
    ros-kinetic-geometry2 \
    ros-kinetic-teb-local-planner \
    ros-kinetic-stage-ros \
    ros-kinetic-find-object-2d \
    ros-kinetic-rgbd-launch \
    ros-kinetic-libuvc \
    ros-kinetic-libuvc-camera \
    ros-kinetic-libuvc-ros


# Additional python packages

RUN apt install -y \
    python3-pip \
    python3-yaml \
    python-rosdep

RUN pip3 install --upgrade pip

RUN pip3 install rospkg catkin_pkg tensorflow==1.15

RUN apt insatll nano

RUN pip install aiohttp telepot

# User WS dependings

COPY ${DRIVERS_WS} ${DRIVERS_WS}
RUN rosdep init && rosdep update && \
    rosdep install --from-paths ${DRIVERS_WS} --ignore-src --rosdistro=kinetic -y
RUN rm -R ${DRIVERS_WS}
