#!/usr/bin/env python
# -*- coding: utf8 -*-

# Title:        The finite state machine for StarLine competition
# File:         fm.py
# Date:         2020-03-25
# Author:       Artem Smirnov @urpylka
# Description:  Simple state machine

import time
import rospy
import threading

from locker_oncoming_traffic import LockerWays
from moving_stack import MovingStack
from publish_wall_to_costmap import WallBuilder
from state_machine import AbstractState, StateMachine
from detect_objects import SemaphoreState, DetectObjects

class INIT(AbstractState):

    def run(self, M):
        rospy.loginfo("INIT: Robot's initializating...")
        M.S.moving_stack = MovingStack()

        # M.S.moving_stack.resetOdometry()
        # init_pose = Pose(Point(1, 1, 0.000), Quaternion(0, 0, 0, 1)
        # M.S.moving_stack.initAmcl(init_pose)

        crossroads = [
            {'xy': (3.74, 8.08), 'walls': [[(3.3, 8.1), (3.3, 7.64)], [(3.73, 7.63), (4.16, 7.65)], [(4.15, 8.1), (4.13, 8.5)]]}, #4
            {'xy': (6.8, 6.06), 'walls': [[(6.32, 8.1), (6.32, 7.65)]]}, #6
            {'xy': (1.66, 8.07), 'walls': [[(2.14, 8.1), (2.14, 8.5)]]}  #5
        ]

        M.S.locker_ways = LockerWays(M.S.moving_stack, "/maps/crossroads", crossroads)

        M.S.target_zero_point = Pose(Point(7.76, 4.05, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)
        M.S.target_first_point = Pose(Point(7.95, 5.4, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)

        M.S.detect_objects = DetectObjects()

        rospy.loginfo("INIT: State has changed to GOTO_1.")
        M.new_state(GOTO_1(M))

    def command(self, array_command):
        rospy.loginfo("INIT: It cannot been executed: " + array_command[0])


class GOTO_0(AbstractState):

    def run(self, M):
        M.S.moving_stack.asyncGoTo(M.S.target_zero_point)

        r = rospy.Rate(10)
        while not M.S.moving_stack.getState():
            r.sleep()

            if M.S.detect_objects.detectedStopSign():
                rospy.loginfo("GOTO_0: Stop sign has detected!")

                M.S.moving_stack.cancelGoal()
                rospy.sleep(1.5)
                M.S.moving_stack.asyncGoTo(M.S.target_zero_point)

            if M.S.detect_objects.detectedSemaphore():
                rospy.loginfo("GOTO_0: Semaphore has detected!")

                M.S.moving_stack.cancelGoal()
                r = rospy.Rate(10)
                while M.S.detect_objects.getStateSemaphore() == SemaphoreState.RED:
                    r.sleep()
                M.S.moving_stack.asyncGoTo(M.S.target_zero_point)

        rospy.loginfo("GOTO_0: Another one cycle has done!")
        M.new_state(GOTO_1(M))


class GOTO_1(AbstractState):

    def run(self, M):
        M.S.moving_stack.asyncGoTo(M.S.target_first_point)

        r = rospy.Rate(10)
        while not M.S.moving_stack.getState():
            r.sleep()

            if M.S.detect_objects.detectedStopSign():
                rospy.loginfo("GOTO_1: Stop sign has detected!")

                M.S.moving_stack.cancelGoal()
                rospy.sleep(1.5)
                M.S.moving_stack.asyncGoTo(M.S.target_first_point)

            if M.S.detect_objects.detectedSemaphore():
                rospy.loginfo("GOTO_1: Semaphore has detected!")

                M.S.moving_stack.cancelGoal()
                r = rospy.Rate(10)
                while M.S.detect_objects.getStateSemaphore() == SemaphoreState.RED:
                    r.sleep()
                M.S.moving_stack.asyncGoTo(M.S.target_first_point)

        rospy.loginfo("GOTO_1: The first point has done!")
        M.new_state(GOTO_0(M))


if __name__ == "__main__":
    rospy.init_node("finite_stimulator")
    fm = StateMachine(INIT)
