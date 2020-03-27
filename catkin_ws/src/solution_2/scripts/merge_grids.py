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


def mergeGrids(g, grids):
    g.data = []
    for i in range(g.info.height):
        for j in range(g.info.width):
            max_v = 0
            for grid in grids:
                max_v = max(max_v, getValue(*getShiftOff(grid, i, j)))
            g.data.append(max_v)
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


def cb3(map):
    global map3
    global ts3
    map3 = map
    ts3 = time.time()


def checkForClean():
    global map1
    global map2
    global map3
    global alive
    global ts1
    global ts2
    global ts3
    global one_upd_m1
    global one_upd_m2
    global one_upd_m3

    while alive:
        now = time.time()
        if (now - ts1 > 1) and one_upd_m1:
            cleanGrid(map1)
        if (now - ts2 > 1) and one_upd_m2:
            cleanGrid(map2)
        if (now - ts3 > 1) and one_upd_m3:
            cleanGrid(map3)
        time.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("occupancy_grid_merger")

    global alive
    global ts1
    global ts2
    global ts3
    global map1
    global map2
    global map3
    global one_upd_m1
    global one_upd_m2
    global one_upd_m3

    # If map is updating
    one_upd_m1 = rospy.get_param("~merger/source_map_1/updating", False)
    one_upd_m2 = rospy.get_param("~merger/source_map_2/updating", True)
    one_upd_m2 = rospy.get_param("~merger/source_map_3/updating", True)


    alive = True
    ts1 = 0
    ts2 = 0
    ts3 = 0
    map1 = OccupancyGrid()
    map2 = OccupancyGrid()
    map3 = OccupancyGrid()

    sub_map_1 = rospy.Subscriber("/merger/source_map_1", OccupancyGrid, cb1, queue_size=1)
    sub_map_2 = rospy.Subscriber("/merger/source_map_2", OccupancyGrid, cb2, queue_size=1)
    sub_map_3 = rospy.Subscriber("/merger/source_map_3", OccupancyGrid, cb3, queue_size=1)
    merged_map = rospy.Publisher("/merger/target_map", OccupancyGrid, queue_size=1, latch=True)

    # while(ts1 == 0 or ts2 == 0):
    while(ts1 == 0):
        print("Waiting for /merger/source_map_1")
        time.sleep(1)

    # print(map1.info)
    # print(map2.info)

    map = OccupancyGrid()
    map.header.frame_id = 'map'
    map.info.resolution = 0.0502257
    map.info.width = 220
    map.info.height = 220
    map.info.origin.position.x = 0.0
    map.info.origin.position.y = 0.0
    map.info.origin.position.z = 0
    map.data = []

    try:
        t = threading.Thread(target=checkForClean, args=())
        t.start()

        r = rospy.Rate(10) # 1hz
        while not rospy.is_shutdown():
            merged_map.publish(mergeGrids(map, map1, map2, map3))
            r.sleep()
    except rospy.ROSInterruptException:
        alive = False
