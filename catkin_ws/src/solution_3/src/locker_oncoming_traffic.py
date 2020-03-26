#!/usr/bin/env python
# -*- coding: utf8 -*-

# Title:        The locker of oncoming traffic on crossroads for turtleworl at StarLine competition
# File:         locker_oncoming_traffic.py
# Date:         2020-03-26
# Author:       Artem Smirnov @urpylka
# Description:  Locker of oncoming traffic on crossroads

import rospy
import threading
import math
from publish_wall_to_costmap import WallBuilder


def getDistance((x1, y1), (x2, y2)):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(pow(dx, 2) + pow(dy, 2))


class LockerWays():

    alive = True
    error_income = 1.5      # Lock the crossroad when distance to center will be less than 1.5
    error_outcome = 3.5     # Unlock the crossroad when distance to center will be more than error_outcome

    def __init__(self, _moving_stack, _topic, _crossroads = []):
        self.w = WallBuilder(_topic)
        self.crossroads = _crossroads

        # Stop robot when catched Ctrl-C or failure
        rospy.on_shutdown(self.stopLocker)

        t = threading.Thread(target=self.checker, args=(_moving_stack,))
        t.start()


    def checker(self, moving_stack):

        checker_rate = rospy.Rate(10) # 10hz
        publication_rate = rospy.Rate(10) # 10hz

        while self.alive:

            pose = moving_stack.getPose()
            xy = (pose.position.x, pose.position.y)

            for crossroad in self.crossroads:
                if getDistance(xy, crossroad['xy']) < self.error_income:
                    while getDistance(xy, crossroad['xy']) > self.error_outcome:
                        self.w.publishMap(crossroad['walls'])
                        publication_rate.sleep()

            checker_rate.sleep()


    def stopLocker(self):
        self.alive = False


    def __del__(self):
        self.stopLocker()
