ARG from

FROM ${from}
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
    vim \
    nano



# Environment configuration

ENV OS_DISTRO xenial
ENV ROS_DISTRO kinetic
ENV ROS_INSTALL_PATH "/opt/ros/$ROS_DISTRO"
ENV SIM_WS /simulator_ws



# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone



# Install ROS desktop and Gazebo

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $OS_DISTRO main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y ros-kinetic-desktop gazebo7 && \
    echo "source $ROS_INSTALL_PATH/setup.bash"  >> "/root/.bashrc" && \
    echo "source ${SIM_WS}/devel/setup.bash" >> "/root/.bashrc" && \
    echo "source /catkin_ws/devel/setup.bash" >> "/root/.bashrc" && \
    rosdep init && rosdep update



# Additional ROS packages

RUN apt-get install -y \
    ros-kinetic-teleop-twist-keyboard \
    ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-hector-models \
    ros-kinetic-navigation \
    ros-kinetic-explore-lite \
    ros-kinetic-gmapping \
    ros-kinetic-ira-laser-tools \
    ros-kinetic-pointcloud-to-laserscan


# Install catkin tools

RUN echo "deb http://packages.ros.org/ros/ubuntu $OS_DISTRO main" > /etc/apt/sources.list.d/ros-latest.list && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt-get update && apt-get install -y python-catkin-tools


# Install broken dependence of ros-kinetic-turtlebot-navigation manual
# More info at https://github.com/IntelRealSense/librealsense/issues/4781

RUN apt-get update && \
    apt-get install -y lsb-release && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && \
    apt-get install -y binutils cpp cpp-5 dkms fakeroot gcc gcc-5 kmod libasan2 libatomic1 libc-dev-bin libc6-dev libcc1-0 libcilkrts5 libfakeroot libgcc-5-dev libgmp10 libgomp1 libisl15 libitm1 liblsan0 libmpc3 libmpfr4 libmpx0 libquadmath0 libssl-dev libssl-doc libtsan0 libubsan0 libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic linux-libc-dev make manpages manpages-dev menu patch zlib1g-dev && \
    apt-get install -y libssl-dev libssl-doc libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic zlib1g-dev && \
    apt-get download ros-kinetic-librealsense && \
    dpkg-deb -R ros-kinetic-librealsense*.deb ros-rslib/ && \
    wget https://gist.githubusercontent.com/dizz/404ef259a15e1410d692792da0c27a47/raw/3769e80a051b5f2ce2a08d4ee6f79c766724f495/postinst && \
    chmod +x postinst && \
    cp postinst ros-rslib/DEBIAN && \
    dpkg-deb -b ./ros-rslib/ ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb && \
    dpkg -i ros-kinetic-librealsense_1.12.1-0xenial-20190830_icrlab_amd64.deb && \
    apt-mark hold ros-kinetic-librealsense


# Install navigation module

RUN apt-get install -y ros-kinetic-turtlebot-navigation


# User WS dependings

COPY ${SIM_WS} ${SIM_WS}
RUN rosdep install --from-paths ${SIM_WS} --ignore-src --rosdistro=${ROS_DISTRO} -y

RUN rm -R ${SIM_WS}
