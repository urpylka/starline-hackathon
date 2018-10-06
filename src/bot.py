#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import os.path, rospy, time

import telepot
from telegrambot import TelegramBot

# Header
from std_msgs.msg import *

# PoseWithCovarianceStamped
from geometry_msgs.msg import *

# MoveBaseActionGoal MoveBaseGoal 
from move_base_msgs.msg import *

# GoalID
from actionlib_msgs.msg import *

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def my_goto(self, _pose):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = _pose

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
        Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

temp_pose = null

def get_cur_pose(data):
    temp_pose = data.pose.pose

def main():
    rospy.init_node('turtle_express')

    TOKEN = load_param('~token')
    CHAT_ID = load_param('~chat_id')
    PROXY = load_param('~proxy')
    DEBUG = load_param('~debug', True)

    # Params of info-messages
    # self.bot = TelegramBot(TOKEN, CHAT_ID, PROXY, DEBUG, self)
    # self.debug_message = self.bot.debug_message

    rospy.loginfo('Inited turtle_express')
    rospy.Subscriber('/amcl_pose', _PoseWithCovarianceStamped, get_cur_pose)

    navigator = GoToPose()

    # Customize the following values so they are appropriate for your location
    # position = {'x': 1.22, 'y' : 2.56}
    # quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

    my_pose = temp_pose

    time.sleep(10)

    success = navigator.my_goto(my_pose)

    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base failed to reach the desired pose")

    # goto = rospy.Publisher('/move_base/goal', _MoveBaseActionGoal)
    # goto.publish()

    rospy.spin()

if __name__ == '__main__':
    main()