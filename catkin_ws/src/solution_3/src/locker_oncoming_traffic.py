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

from moving_stack import MovingStack
from publish_wall_to_costmap import WallBuilder


def getDistance((x1, y1), (x2, y2)):
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx**2 + dy**2)


class LockerWays():

    alive = True
    error_income = 0.5      # Lock the crossroad when distance to center will be less than 1.5
    error_outcome = 1     # Unlock the crossroad when distance to center will be more than error_outcome

    def __init__(self, _moving_stack, _topic, _crossroads = []):
        self.w = WallBuilder(_topic)
        self.crossroads = _crossroads

        # Stop robot when catched Ctrl-C or failure
        rospy.on_shutdown(self.stopLocker)

        t = threading.Thread(target=self.checker, args=(_moving_stack,))
        t.start()


    def getXy(self, moving_stack):
        pose = moving_stack.getPose()
        return (pose.position.x, pose.position.y)


    def checker(self, moving_stack):
        checker_rate = rospy.Rate(1)

        while self.alive:
            xy = self.getXy(moving_stack)
            rospy.loginfo("Locker crossroads: " + str(xy))

            for crossroad in self.crossroads:
                if not entered and (getDistance(xy, crossroad['xy']) < self.error_income):
                    rospy.loginfo("Locker crossroads: Lock the crossroad: " + str(crossroad['xy']))

                    while getDistance(xy, crossroad['xy']) < self.error_outcome:
                        self.w.publishMap(crossroad['walls'])
                        rospy.loginfo("Locker crossroads: Distance: " + str(getDistance(xy, crossroad['xy'])))
                        xy = self.getXy(moving_stack)
                        checker_rate.sleep()

                    rospy.loginfo("Locker crossroads: Unlock the crossroad: " + str(crossroad['xy']))
            checker_rate.sleep()


    def stopLocker(self):
        self.alive = False


    def __del__(self):
        self.stopLocker()

if __name__ == "__main__":
    rospy.init_node("locker_crossroads")

    crossroads = [
        {'xy': (3.7, 6.05), 'walls': [[(3.7, 6.55), (3.28, 6.55)]]}, #block to #4
        {'xy': (3.74, 8.08), 'walls': [[(3.3, 8.1), (3.3, 7.64)], [(3.73, 7.63), (4.16, 7.65)], [(4.15, 8.1), (4.13, 8.5)]]}, #4
        {'xy': (6.8, 6.06), 'walls': [[(6.32, 8.1), (6.32, 7.65)]]}, #6
        {'xy': (1.66, 8.07), 'walls': [[(2.14, 8.1), (2.14, 8.5)]]}  #5
    ]

    moving_stack = MovingStack()
    locker_ways = LockerWays(moving_stack, "/maps/crossroads", crossroads)
    rospy.spin()
