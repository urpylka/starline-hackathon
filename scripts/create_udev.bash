#!/bin/bash
echo "[INFO] Setting up udev rules..."

# RPLidar
sudo cp rplidar.rules /etc/udev/rules.d
# Kobuki
sudo cp kobuki.rules /etc/udev/rules.d
# Astra
sudo cp astra.rules /etc/udev/rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
