#!/bin/bash

xhost +local:docker || true

HOST_IP_LIST=(`hostname -I`)  #Get a list of host ip-adresses
HOST_IP=${HOST_IP_LIST[0]}    #Get the first ip from list: it is wifi ip

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

sudo docker run -ti --rm \
                --env="DISPLAY" \
                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                -v /dev:/dev \
                -v $ROOT_DIR/drivers_ws:/drivers_ws \
                -v $ROOT_DIR/catkin_ws:/catkin_ws \
                --net=host \
                -e ROS_IP="$HOST_IP" \
                --privileged \
                --name kobuki kobuki-img
