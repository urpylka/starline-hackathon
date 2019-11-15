#!/bin/bash
echo "[INFO] Setting up udev rules..."

# RPLidar
sudo cp 20-rplidar.rules /etc/udev/rules.d
# Kobuki
sudo cp 30-kobuki.rules /etc/udev/rules.d
# Astra
sudo cp 10-astra.rules /etc/udev/rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
