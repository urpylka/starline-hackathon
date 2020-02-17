#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

docker build -t kobuki-base-img -f $ROOT_DIR/docker/kobuki/Dockerfile $ROOT_DIR --network=host
