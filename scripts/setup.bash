#!/bin/bash

#
# Installing some useful stuff
#

sudo apt-get install -y vim mc ssh

# Install docker
# sudo apt-get remove docker docker-engine docker.io containerd runc
sudo apt-get update -y
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update -y
sudo apt-get install -y docker-ce docker-ce-cli containerd.io
sudo groupadd docker
sudo usermod -aG docker $USER

# Install docker-compose
#sudo curl -L "https://github.com/docker/compose/releases/download/1.24.1/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
#sudo chmod +x /usr/local/bin/docker-compose
#docker-compose --version

KOBUKI_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

sudo bash $KOBUKI_ROOT_DIR/scripts/udev_rules/create_udev.bash

echo " " >> ~/.bashrc
echo "alias kobuki_docker_build='bash $KOBUKI_ROOT_DIR/docker/build_docker.sh'" >> ~/.bashrc
echo "alias kobuki_docker_into='bash $KOBUKI_ROOT_DIR/docker/into_docker.sh'" >> ~/.bashrc
echo "alias kobuki_docker_run='bash $KOBUKI_ROOT_DIR/docker/run_docker.sh'" >> ~/.bashrc
