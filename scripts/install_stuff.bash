#!/bin/bash

#
# Installing some useful stuff
#

sudo apt-get install vim
sudo apt-get install ssh
sudo apt-get install mc

# Install docker-compose
sudo curl -L "https://github.com/docker/compose/releases/download/1.24.1/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
docker-compose --version