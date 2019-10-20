#!/bin/bash
echo "[INFO] Setting up udev rules..."

# RPLidar
sudo cp rplidar.rules /etc/udev/rules.d/rplidar_rules.d
# Kobuki
sudo cp kobuki.rules /etc/udev/rules.d/kobuki_rules.d
# Astra
sudo cp astra.rules /etc/udev/rules.d/astra_rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
