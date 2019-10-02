FROM hackathon-base

WORKDIR ${catkin_ws}/src

# clone source code
RUN git clone -b kinetic-devel https://github.com/ros-visualization/rviz

# install build dependacies
RUN apt-get -qq update && \
    apt-get -qq install -y \
        python-catkin-tools && \
    rosdep update && \
    rosdep install -y \
      --from-paths . \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
      --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# build from source
WORKDIR ${catkin_ws}
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN bash -c "source ${catkin_ws}/devel/setup.bash;catkin_make"

# set environment variables
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics