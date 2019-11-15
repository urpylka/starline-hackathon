#!/bin/bash

ROBOT_IP=$1
ROBOT_ROS_URI="http://$ROBOT_IP:11311"

HOST_IP_LIST=(`hostname -I`)
HOST_IP=${HOST_IP_LIST[0]}

export ROS_MASTER_URI="$ROBOT_ROS_URI"
export ROS_IP="$HOST_IP"
