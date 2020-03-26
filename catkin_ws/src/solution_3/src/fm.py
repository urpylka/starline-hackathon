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

class INIT(AbstractState):

    def run(self, M):
        rospy.loginfo("INIT: Robot's initializating...")
        M.S.moving_stack = MovingStack()

        crossroads = [
            {'xy': (1, 1), 'walls': [[(20, 0), (40, 40)], [(20, 0), (40, 40)]]},
            {'xy': (2, 4), 'walls': [[(20, 0), (40, 40)], [(20, 0), (40, 40)]]}
        ]

        M.S.locker_ways = LockerWays(M.S.moving_stack, "/crossroads", crossroads)

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


class GOTO(AbstractState):

    def run(self, M):
        success = M.S.moving_stack.goTo(my_poses[cur_pose])
        if success:
            rospy.loginfo("GOTO: Hooray, reached the desired pose")
        else:
            rospy.loginfo("GOTO: The base failed to reach the desired pose")

        M.new_state(IDLE(M))

    def command(self, array_command):
        rospy.loginfo("GOTO: Can't execute the command: " + array_command[0])


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
