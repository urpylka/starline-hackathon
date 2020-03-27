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

from move_base_msgs.msg import *    # MoveBaseActionGoal MoveBaseGoal
from std_msgs.msg import *          # Header, Empty
from geometry_msgs.msg import *     # PoseWithCovarianceStamped
import actionlib
from actionlib_msgs.msg import *    # Goal ID

# from locker_oncoming_traffic import LockerWays
from moving_stack import MovingStack
from publish_wall_to_costmap import WallBuilder
from state_machine import AbstractState, StateMachine
from detect_objects import DetectObjects
from user_control import UserControl

class INIT(AbstractState):
    def run(self):
        rospy.loginfo(str(self) + ": Robot's initializating...")

        self.M.S.moving_stack = MovingStack()
        self.M.S.WB = WallBuilder("/maps/start_wall")
        self.M.S.start_wall = [[(7.3, 4.5), (8.24, 4.5)]]

        self.M.S.target_zero_point = Pose(Point(7.76, 4.05, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
        self.M.S.target_first_point = Pose(Point(7.95, 5.4, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))

        self.M.S.detect_objects = DetectObjects()

        rospy.loginfo(str(self) + ": The state is changing to IDLE.")
        self.M.new_state(IDLE(self.M))

    def command(self, com):
        rospy.loginfo(str(self) + ": Command {} is not supported.", com[0])


class IDLE(AbstractState):
    def run(self):
        self.M.S.stop_state = False
        while not self.M.S.stop_state:
            rospy.sleep(1)

        rospy.loginfo(str(self) + ": State has changed to GOTO_1.")
        self.M.new_state(GOTO_1(self.M))

    def command(self, com):
        if com[0] == 'goto':
            self.M.S.stop_state = True
        elif com[0] == 'reset_odometry':
            self.M.S.moving_stack.resetOdometry()
        elif com[0] == 'init_pose':
            init_pose = Pose(Point(int(com[1]), int(com[2]), 0.000), Quaternion(0, 0, 0, 1))
            self.M.S.moving_stack.initAmcl(init_pose)
        else:
            rospy.loginfo(str(self) + ": Command {} is not supported.", com[0])


class GOTO_0(AbstractState):

    def tryToGotoIdle(self):
        if self.M.S.stop_state:
            self.M.S.moving_stack.cancelGoal()
            rospy.loginfo(str(self) + ": It goes to state IDLE!")
            self.M.new_state(IDLE(self.M))

    def makeWall(self):
        try:
            r = rospy.Rate(10) # 10hz
            while not self.stop_state:
                self.M.S.WB.publishMap(self.M.S.start_wall)
                r.sleep()
        except rospy.ROSInterruptException:
            pass

    def run(self):
        self.M.S.stop_state = False

        t = threading.Thread(target=self.makeWall, args=(,))
        t.start()

        self.M.S.moving_stack.asyncGoTo(self.M.S.target_zero_point)

        r = rospy.Rate(3)
        while not self.M.S.moving_stack.getState():
            r.sleep()
            self.tryToGotoIdle()

            if self.M.S.detect_objects.detectedStopSign():
                rospy.loginfo(str(self) + ": Stop sign detected!")

                self.M.S.moving_stack.cancelGoal()
                rospy.sleep(1.5)
                self.M.S.moving_stack.asyncGoTo(self.M.S.target_zero_point)
                rospy.sleep(1) # time for going away

            if self.M.S.detect_objects.detectedSemaphoreRed():
                rospy.loginfo(str(self) + ": Semaphore detected!")
                self.M.S.moving_stack.cancelGoal()

                while self.M.S.detect_objects.detectedSemaphoreRed():
                    r.sleep()
                    self.tryToGotoIdle()

                self.M.S.moving_stack.asyncGoTo(self.M.S.target_zero_point)
                rospy.sleep(1) # time for going away

        rospy.loginfo(str(self) + ": Another one cycle has done!")
        self.M.S.stop_state = True
        self.M.new_state(GOTO_1(self.M))

    def command(self, com):
        if com[0] == 'idle':
            self.M.S.stop_state = True
        else:
            rospy.loginfo(str(self) + ": Command {} is not supported.", com[0])


class GOTO_1(GOTO_0):

    def run(self):
        self.M.S.stop_state = False
        self.M.S.moving_stack.asyncGoTo(self.M.S.target_first_point)

        r = rospy.Rate(3)
        while not self.M.S.moving_stack.getState():
            r.sleep()
            self.tryToGotoIdle()

            if self.M.S.detect_objects.detectedStopSign():
                rospy.loginfo(str(self) + ": Stop sign has detected!")

                self.M.S.moving_stack.cancelGoal()
                rospy.sleep(1.5)
                self.M.S.moving_stack.asyncGoTo(self.M.S.target_first_point)
                rospy.sleep(1) # time for going away

            if self.M.S.detect_objects.detectedSemaphoreRed():
                rospy.loginfo(str(self) + ": Semaphore has detected!")
                self.M.S.moving_stack.cancelGoal()

                while self.M.S.detect_objects.detectedSemaphoreRed():
                    r.sleep()
                    self.tryToGotoIdle()

                self.M.S.moving_stack.asyncGoTo(self.M.S.target_first_point)
                rospy.sleep(1) # time for going away

        rospy.loginfo(str(self) + ": The first point has done!")
        self.M.new_state(GOTO_0(self.M))


if __name__ == "__main__":
    rospy.init_node("finite_stimulator")

    fm = StateMachine(INIT)
    ui = UserControl(fm)

    rospy.spin()
