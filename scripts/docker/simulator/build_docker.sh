#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
EXEC_PATH=$PWD

cd $ROOT_DIR

if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker build -t kobuki-sim-img -f $ROOT_DIR/docker/simulator/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  --build-arg from=nvidia/opengl:1.1-glvnd-runtime-ubuntu16.04

else
    echo "[!] If you wanna use nvidia gpu, please rebuild with -n or --nvidia argument"
    docker build -t kobuki-sim-img -f $ROOT_DIR/docker/simulator/Dockerfile $ROOT_DIR \
                                  --network=host \
                                  --build-arg from=ubuntu:16.04
fi

cd $EXEC_PATH
