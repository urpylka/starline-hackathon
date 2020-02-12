#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

docker build -t kobuki-sl-hackathon-sim -f $ROOT_DIR/docker/Dockerfile $ROOT_DIR --network=host
