#!/usr/bin/env python3
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

class INIT(AbstractState):

    def run(self, M):
        rospy.loginfo("INIT: Robot's initializating...")
        M.S.moving_stack = MovingStack()

        crossroads = [
            {'xy': (44, 11, 'walls': [[(20, 0), (40, 40)], [(20, 0), (40, 40)]]},
            {'xy': (2, 4), 'walls': [[(20, 0), (40, 40)], [(20, 0), (40, 40)]]}
        ]

        M.S.locker_ways = LockerWays(M.S.moving_stack, "/crossroads", crossroads)

        M.S.target_zero_point = Pose(Point(1, 1, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)
        M.S.target_first_point = Pose(Point(1, 10, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)

        # Change state
        M.new_state(IDLE(M))

    def command(self, array_command):
        rospy.loginfo("INIT: It cannot been executed: " + array_command[0])


class IDLE(AbstractState):

    def run(self, M):
        while True:
            if not self.stop_state:
                time.sleep(1)
            else:
                M.new_state(GOTO(M))
                break

    def command(self, array_command):
        rospy.loginfo("IDLE: Command: " + array_command[0])
        command = array_command[0]

        if command == 'status':
            pass
        elif command == 'go':
            self.stop_state = True
        elif command == 'save':
            pass
        else:
            # Incorrect command
            pass


class GOTO_1(AbstractState):

    def run(self, M):
        M.S.moving_stack.asyncGoTo(M.S.target_first_point)

        r = rospy.Rate(10)
        while not M.S.moving_stack.getState():
            r.sleep()
            if M.S.stop_sign_checker.getState():
                M.new_state(GOTO_0(M))
            if M.S.semaphore_checker.getState():
                M.new_state(GOTO_0(M))


        rospy.loginfo("GOTO_1: Another one cycle done!")
        M.new_state(GOTO_0(M))

    def command(self, array_command):
        rospy.loginfo("GOTO_1: Can't execute the command: " + array_command[0])


class GOTO_0(AbstractState):

    def run(self, M):
        M.S.moving_stack.asyncGoTo(M.S.target_zero_point)

        r = rospy.Rate(10)
        while not M.S.moving_stack.getState():
            r.sleep()

        M.new_state(IDLE(M))

    def command(self, array_command):
        rospy.loginfo("GOTO_0: Can't execute the command: " + array_command[0])


class WAIT_SEMAPHORE(AbstractState):

    def run(self, M):
        success = navigator.goto(my_poses[cur_pose])
        if success:
            rospy.loginfo("GOTO: Hooray, reached the desired pose")
        else:
            rospy.loginfo("GOTO: The base failed to reach the desired pose")

        M.new_state(IDLE(M))

    def command(self, array_command):
        rospy.loginfo("GOTO: Can't execute the command: " + array_command[0])


class WAIT_SIGN_STOP(AbstractState):

    def run(self, M):
        success = MovingStack.goTo(my_poses[cur_pose])
        if success:
            rospy.loginfo("GOTO: Hooray, reached the desired pose")
        else:
            rospy.loginfo("GOTO: The base failed to reach the desired pose")

        M.new_state(IDLE(M))

    def command(self, array_command):
        rospy.loginfo("GOTO: Can't execute the command: " + array_command[0])


if __name__ == "__main__":
    rospy.init_node("finite_stimulator")

    fm = StateMachine(INIT)

    try:
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # merged_map.publish(mergeGrids(map, map1, map2))
            r.sleep()
    except rospy.ROSInterruptException:
        pass
        # alive = False
