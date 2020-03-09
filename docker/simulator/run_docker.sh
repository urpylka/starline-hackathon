#!/bin/bash

xhost +local:docker || true

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker run --gpus all \
                -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v /dev:/dev \
                -v $ROOT_DIR/simulator_ws:/simulator_ws \
                -v $ROOT_DIR/catkin_ws:/catkin_ws \
               --net=host \
               --privileged \
               --name kobuki-sim kobuki-sim-img

else

    echo "[!] If you wanna use nvidia gpu, please use script with -n or --nvidia argument"
    docker run  -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v /dev:/dev \
                -v $ROOT_DIR/simulator_ws:/simulator_ws \
                -v $ROOT_DIR/catkin_ws:/catkin_ws \
               --net=host \
               --privileged \
               --name kobuki-sim kobuki-sim-img
fi
