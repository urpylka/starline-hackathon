#!/usr/bin/env python

# Title:        The OccupancyGrids Merger
# File:         merge_grids.py
# Date:         2020-03-23
# Author:       Artem Smirnov @urpylka
# Description:  Merge two OccupancyGrid topic to the third

import time
import rospy
from nav_msgs.msg import OccupancyGrid


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


def getShiftOff(g, y, x):
    return g, y, x


def getShift(g, y, x):
    return g, (y - 1 + 30), (x - 1 + 50)
    # return g, y + int(g.info.origin.position.y / g.info.resolution), x + int(g.info.origin.position.x / g.info.resolution)



def mergeGrids(g, g1, g2):
    g.data = []
    for i in range(g.info.height):
        for j in range(g.info.width):
            v_g1 = getValue(*getShiftOff(g1, i, j))
            v_g2 = getValue(*getShiftOff(g2, i, j))
            g.data.append(max(v_g1, v_g2))
    return g


f1 = True
f2 = True


def cb1(map):
    global map1
    map1 = map
    global f1
    f1 = False

    # data = []
    # for i in range(map.info.height):
    #     for j in range(map.info.width):
    #         data.append(0)
    # map1.data = data


def cb2(map):
    global map2
    map2 = map
    global f2
    f2 = False

    # data = []
    # for i in range(map.info.height):
    #     for j in range(map.info.width):
    #         data.append(0)
    # map2.data = data


if __name__ == "__main__":
    rospy.init_node("occupancy_grid_merger")

    map1 = OccupancyGrid()
    map2 = OccupancyGrid()

    sub_map_1 = rospy.Subscriber('/map', OccupancyGrid, cb1, queue_size=1)
    sub_map_2 = rospy.Subscriber('/robot_0/move_base/local_costmap/costmap2', OccupancyGrid, cb2, queue_size=1)
    merged_map = rospy.Publisher("/robot_0/move_base/local_costmap/costmap3", OccupancyGrid, queue_size=1, latch=True)

    while(f1 and f2):
        print("Waiting for maps")
        time.sleep(1)

    result_grid_cfg = {
        'MAP_WIDTH': 12,
        'MAP_HEIGHT': 12,
        'RESOLUTION': 0.05,
        'ORIGIN_X': -3,
        'ORIGIN_Y': -3,
        'ORIGIN_Z': 0
    }

    result_grid_cfg['MAP_HEIGHT'] = max(map1.info.height, map2.info.height)
    result_grid_cfg['MAP_WIDTH'] = max(map1.info.width, map2.info.width)
    # result_grid_cfg['RESOLUTION'] = min(map1.info.resolution, map2.info.resolution)

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
