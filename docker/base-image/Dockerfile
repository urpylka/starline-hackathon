FROM ros:kinetic
MAINTAINER Alexander Karavaev "karavaev.aa@starline.ru"

# Fix inability to launch avahi-daemon correctly
# Modified instructions from https://github.com/dockerimages/docker-systemd
RUN apt-get update && \
    apt-get install -y  wget \
                        libnss-mdns \
                        netatalk \
                        systemd &&\
                        apt-get clean &&\
                        rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN mkdir -p /var/run/dbus
RUN cd /lib/systemd/system/sysinit.target.wants/; ls | grep -v systemd-tmpfiles-setup | xargs rm -f $1 \
    rm -f /lib/systemd/system/multi-user.target.wants/*;\
    rm -f /etc/systemd/system/*.wants/*;\
    rm -f /lib/systemd/system/local-fs.target.wants/*; \
    rm -f /lib/systemd/system/sockets.target.wants/*udev*; \
    rm -f /lib/systemd/system/sockets.target.wants/*initctl*; \
    rm -f /lib/systemd/system/basic.target.wants/*;\
    rm -f /lib/systemd/system/anaconda.target.wants/*; \
    rm -f /lib/systemd/system/plymouth*; \
    rm -f /lib/systemd/system/systemd-update-utmp*
ENV init /lib/systemd/systemd
VOLUME ["/sys/fs/cgroup"]


# Export the turtlebot variables
ENV TURTLEBOT_BASE kobuki
ENV TURTLEBOT_STACKS hexagons
ENV TURTLEBOT_3D_SENSOR astra
ENV TURTLEBOT_SERIAL_PORT /dev/ttyUSB0

# Create a catkin workspace
ENV catkin_ws /catkin_ws
WORKDIR ${catkin_ws}
RUN mkdir -p "${catkin_ws}/src" && \ 
    bash -c " source /opt/ros/${ROS_DISTRO}/setup.bash && \
              cd ${catkin_ws}/src && \
              catkin_init_workspace && \
              cd .. && \
              catkin_make"

# Install package into workspace, install their deps and build whole workspace
RUN apt-get update
RUN bash -c "cd ${catkin_ws}/src && \
             git clone https://github.com/yujinrobot/kobuki.git && cd kobuki && git checkout kinetic && cd .. && \
             source /opt/ros/${ROS_DISTRO}/setup.bash &&\
             rosdep install --from-paths /catkin_ws --ignore-src --rosdistro=kinetic -y &&\
             cd ${catkin_ws} && catkin_make"

RUN bash -c "git clone https://github.com/robopeak/rplidar_ros.git && \
              source /catkin_ws/devel/setup.bash &&\
              rosdep install --from-paths /catkin_ws --ignore-src --rosdistro=kinetic -y &&\
              cd ${catkin_ws} && catkin_make"

RUN bash -c " git clone https://github.com/orbbec/ros_astra_camera.git && \
              source /catkin_ws/devel/setup.bash &&\
              rosdep install --from-paths /catkin_ws --ignore-src --rosdistro=kinetic -y &&\
              cd ${catkin_ws} && catkin_make"

# Add entrypoint
ADD turtlebot_entrypoint.sh /turtlebot_entrypoint.sh
ENTRYPOINT ["/turtlebot_entrypoint.sh"]
