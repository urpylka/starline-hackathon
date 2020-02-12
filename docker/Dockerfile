FROM nvidia/opengl:1.1-glvnd-runtime-ubuntu16.04
MAINTAINER Nikolay Dema "ndema2301@gmail.com"


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
ENV ROS_INSTALL_PATH "/opt/ros/$ROS_DISTRO/"
ENV CATKIN_WS /catkin_tws



# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone



# Install ROS desktop and Gazebo

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $OS_DISTRO main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y ros-kinetic-desktop gazebo7 && rosdep init && rosdep update && \
    echo "source $ROS_INSTALL_PATH/setup.bash" >> "/root/.bashrc"


# Additional ROS packages

RUN apt-get install -y ros-kinetic-teleop-twist-keyboard



# Install catkin tools

RUN echo "deb http://packages.ros.org/ros/ubuntu $OS_DISTRO main" > /etc/apt/sources.list.d/ros-latest.list && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && apt-get install -y python-catkin-tools



#RUN echo "source ${DRIVERS_WS}/devel/setup.bash" >> "/root/.bashrc"


#RUN  mkdir -p "${DRIVERS_WS}/src" && cd "${DRIVERS_WS}" && catkin init
#COPY drivers_tws/src ./drivers_tws/src


# Install deps for astra-camera
#RUN apt install -y ros-kinetic-rgbd-launch \
#    ros-kinetic-libuvc \
#    ros-kinetic-libuvc-camera \
#    ros-kinetic-libuvc-ros \
#    ros-kinetic-teleop-twist-keyboard && \
#    rosdep update && \
#    rosdep install --from-paths ${DRIVERS_WS} --ignore-src --rosdistro=kinetic -y

#RUN rm -R ${DRIVERS_WS}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility
