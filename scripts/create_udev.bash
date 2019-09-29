#!/bin/bash
echo "[INFO] Setting up udev rules..."

# RPLidar
sudo cp /catkin_ws/src/rplidar_ros/scripts/rplidar.rules /etc/udev/rplidar_rules.d
# Kobuki
sudo cp kobuki.rules /etc/udev/kobuki_rules.d
# Astra
sudo cp /catkin_ws/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/astra_rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
