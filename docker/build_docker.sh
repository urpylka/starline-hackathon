#!/usr/bin/env bash

docker build -t kobuki-sl-hackathon-base -f docker/Dockerfile . --network=host
