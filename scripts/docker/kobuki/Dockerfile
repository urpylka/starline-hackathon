FROM osrf/ros:kinetic-desktop-full
MAINTAINER Nikolay Dema "ndema2301@gmail.com"

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


# Create and catkin tools ws
ENV  DRIVERS_WS /drivers_ws
ENV  ROS_DISTRO kinetic
ENV  ROS_INSTALL_PATH "/opt/ros/$ROS_DISTRO/"

RUN echo "source ${ROS_INSTALL_PATH}/setup.bash" >> "/root/.bashrc"
RUN echo "source ${DRIVERS_WS}/devel/setup.bash" >> "/root/.bashrc"

COPY ${DRIVERS_WS} ${DRIVERS_WS}

ENV PATH              "$PATH:$ROS_INSTALL_PATH/bin"
ENV PYTHONPATH        "$ROS_INSTALL_PATH/lib/python2.7/dist-packages"
ENV LD_LIBRARY_PATH   "$LD_LIBRARY_PATH:$ROS_INSTALL_PATH/lib"
ENV PKG_CONFIG_PATH   "$PKG_CONFIG_PATH:$ROS_INSTALL_PATH/lib/pkgconfig"
ENV CMAKE_PREFIX_PATH "$ROS_INSTALL_PATH"


# Install deps for astra-camera
RUN apt install -y ros-kinetic-rgbd-launch \
    ros-kinetic-libuvc \
    ros-kinetic-libuvc-camera \
    ros-kinetic-libuvc-ros \
    ros-kinetic-teleop-twist-keyboard && \
    rosdep update && \
    rosdep install --from-paths ${DRIVERS_WS} --ignore-src --rosdistro=kinetic -y

RUN rm -R ${DRIVERS_WS}
