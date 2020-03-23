#!/usr/bin/env python

# Title:        Turtlebot Navigation Stack
# File:         rviz_interface.py
# Date:         2017-02-13
# Author:       Preben Jensen Hoel and Paul-Edouard Sarlin
# Description:  Publishes the map and path as standard ROS messages to be
#               displayed in Rviz.

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

import roslib
roslib.load_manifest('occupancy_grid_utils')
import occupancy_grid_utils as gu
import math


def createGrid(cfg, frame_id="world"):
    map = OccupancyGrid()
    map.header.frame_id = frame_id
    map.info.resolution = cfg['RESOLUTION']
    map.info.width = int(cfg['MAP_WIDTH'])
    map.info.height = int(cfg['MAP_HEIGHT'])
    map.info.origin.position.x = cfg['ORIGIN_X']
    map.info.origin.position.y = cfg['ORIGIN_Y']
    map.info.origin.position.z = cfg['ORIGIN_Z']
    map.data = []
    return map


def getValue(g, y, x):
    if (y < g.info.height) and (x < g.info.width):
            return g.data[y*g.info.height + x]
    else:
        return 0

def getShift(g, y, x):
    return g, y - int(g.info.origin.position.y / g.info.resolution), x - int(g.info.origin.position.x / g.info.resolution)

def mergeGrids(g, g1, g2):
    g.data = []
    for i in range(g.info.height):
        for j in range(g.info.width):
            g.data.append(max(getValue(g1, i, j), getValue(*getShift(g2, i, j))))
    return g

f1 = True
f2 = True

def cb1(map):
    global map1
    map1 = map
    global f1
    f1 = False

def cb2(map):
    global map2
    map2 = map
    global f2
    f2 = False

if __name__ == "__main__":
    rospy.init_node("occupancy_grid_merger")

    map1 = OccupancyGrid()
    map2 = OccupancyGrid()

    sub_map_1 = rospy.Subscriber('/robot_0/move_base/local_costmap/costmap', OccupancyGrid, cb1, queue_size=1)
    sub_map_2 = rospy.Subscriber('/robot_0/move_base/local_costmap/costmap2', OccupancyGrid, cb2, queue_size=1)
    merged_map = rospy.Publisher("/robot_0/move_base/local_costmap/costmap3", OccupancyGrid, queue_size=1, latch=True)

    import time
    while(f1 and f2):
        print("Waiting for maps")
        time.sleep(1)

    result_grid_cfg = {
        'MAP_WIDTH': 6,
        'MAP_HEIGHT': 6,
        'RESOLUTION': 0.1,
        'ORIGIN_X': -7.9,
        'ORIGIN_Y': -5.9,
        'ORIGIN_Z': 0
    }

    map2_cfg = {
        'offset_x':-7.9,
        'offset_y':-5.9
    }

    result_grid_cfg['MAP_HEIGHT'] = max(map1.info.height, map2.info.height)
    result_grid_cfg['MAP_WIDTH'] = max(map1.info.width, map2.info.width)
    result_grid_cfg['RESOLUTION'] = min(map1.info.resolution, map2.info.resolution)

    print(map1.info)
    print(map2.info)

    m_map = createGrid(result_grid_cfg, 'map')

    try:
        r = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            merged_map.publish(mergeGrids(m_map, map1, map2))
            r.sleep()
    except rospy.ROSInterruptException:
        pass
