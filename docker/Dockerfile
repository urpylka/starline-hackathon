FROM nvidia/opengl:1.1-glvnd-runtime-ubuntu16.04
#osrf/ros:kinetic-desktop-full
MAINTAINER Nikolay Dema "ndema2301@gmail.com"

RUN apt-get update && apt-get install -y \
    apt-utils \
    mesa-utils \
    gnupg2 \
    net-tools \
    wget \
    libxv1 va-driver-all \
    unzip \
    curl \
    git \
    mc \
    vim

# Install catkin tools
#RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list && \
#    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
#    apt-get update && apt-get install -y python-catkin-tools


# Install Gazebo

#RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
#RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
#
#RUN apt-get update && apt-get install -y gazebo7 \
#                                         libgazebo7-dev \
#                                         libignition-math4


# Create and init catkin tools ws
#ENV  DRIVERS_WS /drivers_tws
#ENV  ROS_DISTRO kinetic
#ENV  ROS_INSTALL_PATH "/opt/ros/$ROS_DISTRO/"

#RUN echo "source ${ROS_INSTALL_PATH}/setup.bash" >> "/root/.bashrc"
#RUN echo "source ${DRIVERS_WS}/devel/setup.bash" >> "/root/.bashrc"


#RUN  mkdir -p "${DRIVERS_WS}/src" && cd "${DRIVERS_WS}" && catkin init
#COPY drivers_tws/src ./drivers_tws/src

#ENV PATH              "$PATH:$ROS_INSTALL_PATH/bin"
#ENV PYTHONPATH        "$ROS_INSTALL_PATH/lib/python2.7/dist-packages"
#ENV LD_LIBRARY_PATH   "$LD_LIBRARY_PATH:$ROS_INSTALL_PATH/lib"
#ENV PKG_CONFIG_PATH   "$PKG_CONFIG_PATH:$ROS_INSTALL_PATH/lib/pkgconfig"
#ENV CMAKE_PREFIX_PATH "$ROS_INSTALL_PATH"


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

#RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
#    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
#ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64


# nvidia-docker hooks
#LABEL com.nvidia.volumes.needed="nvidia_driver"
#ENV PATH /usr/local/nvidia/bin:${PATH}
#ENV LD_LIBRARY_PATH /usr/lib:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
