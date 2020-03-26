#!/usr/bin/env python

# Title:        The OccupancyGrids Merger
# File:         merge_grids.py
# Date:         2020-03-23
# Author:       Artem Smirnov @urpylka
# Description:  Merge two OccupancyGrid topic to the third

import time
import rospy
import threading
from nav_msgs.msg import OccupancyGrid

def cleanGrid(g):
    if g is None:
        return

    data = []
    for i in range(map.info.height * map.info.width):
        data.append(0)
    g.data = data


def getValue(g, y, x):
    if g is None:
        return 0
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


def cb1(map):
    global map1
    global ts1
    map1 = map
    ts1 = time.time()


def cb2(map):
    global map2
    global ts2
    map2 = map
    ts2 = time.time()


def checkForClean():
    global map1
    global map2
    global alive
    global ts1
    global ts2
    global one_upd_m1
    global one_upd_m2

    while alive:
        now = time.time()
        if (now - ts1 > 2) and one_upd_m1:
            cleanGrid(map1)
        if (now - ts2 > 2) and one_upd_m2:
            cleanGrid(map2)
        time.sleep(2)


if __name__ == "__main__":
    rospy.init_node("occupancy_grid_merger")

    global alive
    global ts1
    global ts2
    global map1
    global map2
    global one_upd_m1
    global one_upd_m2

    # If map is updating
    one_upd_m1 = rospy.get_param("~merger/source_map_1/updating", False)
    one_upd_m2 = rospy.get_param("~merger/source_map_2/updating", True)

    alive = True
    ts1 = 0
    ts2 = 0
    map1 = OccupancyGrid()
    map2 = OccupancyGrid()

    sub_map_1 = rospy.Subscriber("/merger/source_map_1", OccupancyGrid, cb1, queue_size=1)
    sub_map_2 = rospy.Subscriber("/merger/source_map_2", OccupancyGrid, cb2, queue_size=1)
    merged_map = rospy.Publisher("/merger/target_map", OccupancyGrid, queue_size=1, latch=True)

    # while(ts1 == 0 or ts2 == 0):
    while(ts1 == 0):
        print("Waiting for /merger/source_map_1")
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

    # print(map1.info)
    # print(map2.info)

    map = OccupancyGrid()
    map.header.frame_id = 'map'
    map.info.resolution = 0.05
    map.info.width = 220
    map.info.height = 220
    map.info.origin.position.x = -1.5
    map.info.origin.position.y = -5.5
    map.info.origin.position.z = 0
    map.data = []

    try:
        t = threading.Thread(target=checkForClean, args=())
        t.start()

        r = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            merged_map.publish(mergeGrids(map, map1, map2))
            r.sleep()
    except rospy.ROSInterruptException:
        alive = False
