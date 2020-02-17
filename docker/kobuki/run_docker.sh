#!/bin/bash

xhost +local:docker || true

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

sudo docker run -ti --rm \
                --env="DISPLAY" \
                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                -v /dev:/dev \
                -v $ROOT_DIR/drivers_ws:/drivers_ws \
                -v $ROOT_DIR/catkin_ws:/catkin_ws \
                --net=host \
                --privileged \
                --name kobuki kobuki-img
