#!/bin/bash
echo "[INFO] Setting up udev rules..."

# RPLidar
sudo cp scripts/rplidar.rules /etc/udev/rules.d/rplidar_rules.d
# Kobuki
sudo cp scripts/kobuki.rules /etc/udev/rules.d/kobuki_rules.d
# Astra
sudo cp scripts/astra.rules /etc/udev/rules.d/astra_rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
