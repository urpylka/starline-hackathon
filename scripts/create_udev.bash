#!/bin/bash
echo "[INFO] Setting up udev rules..."

# RPLidar
sudo cp scripts/rplidar.rules /etc/udev/rplidar_rules.d
# Kobuki
sudo cp scripts/kobuki.rules /etc/udev/kobuki_rules.d
# Astra
sudo cp scripts/astra.rules /etc/udev/astra_rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
