FROM osrf/ros:kinetic-desktop-full
MAINTAINER Alexander Karavaev "karavaev.aa@starline.ru"

# Export the turtlebot variables
ENV TURTLEBOT_BASE kobuki
ENV TURTLEBOT_STACKS hexagons
ENV TURTLEBOT_3D_SENSOR astra
ENV TURTLEBOT_SERIAL_PORT /dev/ttyUSB0

RUN apt-get update && apt-get install -y \
    apt-utils \
    gnupg2 \
    net-tools \
    wget \
    unzip \
    curl \
    git \
    mc \
    vim

# Install catkin tools
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && apt-get install -y python-catkin-tools


# Create and init catkin tools ws
ENV  DRIVERS_WS /drivers_tws
ENV  ROS_DISTRO kinetic
RUN  mkdir -p "${DRIVERS_WS}/src" && cd "${DRIVERS_WS}" && catkin init
COPY drivers_tws/src ./drivers_tws/src

# Checkout to ROS distro
RUN cd ${DRIVERS_WS}/src/kobuki && git checkout ${ROS_DISTRO} && cd .. && cd kobuki_core && git checkout ${ROS_DISTRO}

ENV PATH              "$PATH:$ROS_INSTALL_PATH/bin"
ENV PYTHONPATH        "$ROS_INSTALL_PATH/lib/python2.7/dist-packages"
ENV LD_LIBRARY_PATH   "$LD_LIBRARY_PATH:$ROS_INSTALL_PATH/lib"
ENV PKG_CONFIG_PATH   "$PKG_CONFIG_PATH:$ROS_INSTALL_PATH/lib/pkgconfig"
ENV CMAKE_PREFIX_PATH "$ROS_INSTALL_PATH"


# Install deps for astra-camera
RUN apt install -y ros-kinetic-rgbd-launch \
    ros-kinetic-libuvc \
    ros-kinetic-libuvc-camera \
    ros-kinetic-libuvc-ros && \
    rosdep update && \
    rosdep install --from-paths ${DRIVERS_WS} --ignore-src --rosdistro=kinetic -y


# Install package into workspace, install their deps and build whole workspace
RUN apt-get update
RUN bash -c "source /opt/ros/kinetic/setup.bash && cd ${DRIVERS_WS}/src/ && \
             catkin build &&\
             echo "${DRIVERS_WS}/devel/setup.bash" >> "/root/.bash.rc""
