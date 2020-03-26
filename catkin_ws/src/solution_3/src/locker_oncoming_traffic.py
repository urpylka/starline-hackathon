#!/usr/bin/env python3
# -*- coding: utf8 -*-

# Title:        The locker of oncoming traffic on crossroads for turtleworl at StarLine competition
# File:         locker_oncoming_traffic.py
# Date:         2020-03-26
# Author:       Artem Smirnov @urpylka
# Description:  Locker of oncoming traffic on crossroads

import math
from publish_wall_to_costmap import WallBuilder

def getDistance((x1, y1), (x2, y2)):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(pow(dx, 2) + pow(dy, 2))


class LockerWays():

    alive = True
    error = 1.5

    crossroads = [
        {(1, 1), [{(20, 0), (40, 40)}, {(20, 0), (40, 40)}]},
        {(2, 4), [{(20, 0), (40, 40)}, {(20, 0), (40, 40)}]}
    ]

    def __init__(self, _moving_stack):
        t = threading.Thread(target=self.checker, args=(_moving_stack))
        t.start()


    def checker(self, moving_stack):
        while self.alive:
            r = rospy.Rate(10) # 10hz
            pose = moving_stack.getPose()

            for crossroad in seld.crossroads:
                getError(crossroad[1]) < self.error:
                    w.publishMap(crossroad[2])
                    r.sleep()


    def stopLocker(self):
        self.alive = False


    def __del__(self):
        self.stopLocker()
