#!/bin/bash

xhost +local:docker || true

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

docker run --gpus all \
            -ti --rm \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -e DISPLAY=$DISPLAY \
            -e XAUTHORITY \
            -v /dev:/dev \
            -v $ROOT_DIR/catkin_tws:/catkin_tws \
            --net=host \
            --privileged \
            --name kobuki-sim kobuki-sl-hackathon-sim
            #-e NVIDIA_DRIVER_CAPABILITIES=all \
            #--env="DISPLAY" \
            #-v $ROOT_DIR/drivers_tws:/drivers_tws \
