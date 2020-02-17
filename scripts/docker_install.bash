#!/bin/bash

# Install docker
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


# NVIDIA Container Toolkit
if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
      distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
      curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
      curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

<<<<<<< HEAD:scripts/setup.bash
echo " " >> ~/.bashrc
echo "alias kobuki_docker_build='bash $KOBUKI_ROOT_DIR/docker/kobuki/build_docker.sh'" >> ~/.bashrc
echo "alias kobuki_docker_into='bash $KOBUKI_ROOT_DIR/docker/kobuki/into_docker.sh'" >> ~/.bashrc
echo "alias kobuki_docker_run='bash $KOBUKI_ROOT_DIR/docker/kobuki/run_docker.sh'" >> ~/.bashrc
=======
      sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
      sudo systemctl restart docker
fi
>>>>>>> 1e7450e5abb86eccef9060308dcbfd16b420e14a:scripts/host_setup.bash
