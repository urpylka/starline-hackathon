#!/bin/bash
echo "[INFO] Setting up udev rules..."

# RPLidar
sudo cp 99-rplidar.rules /etc/udev/rules.d
# Kobuki
#sudo cp 99-kobuki.rules /etc/udev/rules.d
# Astra
#sudo cp 99-astra.rules /etc/udev/rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
