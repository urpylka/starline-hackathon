#!/bin/bash

KOBUKI_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

sudo apt-get install -y vim mc ssh

sudo bash $KOBUKI_ROOT_DIR/scripts/docker_install.bash

sudo bash $KOBUKI_ROOT_DIR/scripts/udev_rules/create_udev.bash

echo " " >> ~/.bashrc
echo "alias kobuki_docker_build='bash $KOBUKI_ROOT_DIR/docker/kobuki/build_docker.sh'" >> ~/.bashrc
echo "alias kobuki_docker_into='bash $KOBUKI_ROOT_DIR/docker/kobuki/into_docker.sh'" >> ~/.bashrc
echo "alias kobuki_docker_run='bash $KOBUKI_ROOT_DIR/docker/kobuki/run_docker.sh'" >> ~/.bashrc
