#!/usr/bin/env python
# -*- coding: utf8 -*-

# Title:        The moving stack for state machine at StarLine competition
# File:         moving_stack.py
# Date:         2020-03-25
# Author:       Artem Smirnov @urpylka
# Description:  Moving stack

import rospy
from move_base_msgs.msg import *    # MoveBaseActionGoal MoveBaseGoal

from std_msgs.msg import *          # Header, Empty
from geometry_msgs.msg import *     # PoseWithCovarianceStamped
import actionlib
from actionlib_msgs.msg import *    # Goal ID

class MovingStack():

    cur_pose = None

    def __init__(self):

        # Stop robot when catched Ctrl-C or failure
        rospy.on_shutdown(self.cancelGoal)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, lambda data: self.cur_pose = data.pose.pose)

        # move_base API
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Moving stack: Wait for the action server to come up")
        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))


    def initAmcl(self, _pose):
        """
        https://answers.ros.org/question/263835/amcl-and-odometry/
        """
        # latch - если сообщение было опубликовано до запуска ноды, то оно не будет получено
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped)
        p = PoseWithCovarianceStamped()
        p.pose = _pose
        pub.publish(p)

    def resetOdometry(self):
        """
        https://answers.ros.org/question/203088/reset-turtlebot-odometry-in-a-python-script/
        """

        # set up the odometry reset publisher
        reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

        # reset odometry (these messages take a few iterations to get through)
        timer = time()
        while time() - timer < 0.25:
            reset_odom.publish(Empty())


    def cancelGoal(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Moving stack: Robot has stopped")
        rospy.sleep(1)


    def getPose(data):
        return self.cur_pose


    def goTo(self, _pose):
        """
        # Customize the following values so they are appropriate for your location
        # pos = {'x': 1.22, 'y' : 2.56}
        # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4'])
        goto(pose)
        """

        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = _pose

        # Send the goal and start moving
        self.move_base.send_goal(goal, done_cb= self.done_cb)

        # Allow 1 minute to get there (Blocking function)
        timeout = self.move_base.wait_for_result(rospy.Duration(60))
        state = self.move_base.get_state()

        if timeout and getState():
            return True
        else:
            self.goal_sent = False
            self.move_base.cancel_goal()
            return False


    def asyncGoTo(self, _pose):

        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = _pose

        # Send the goal and start moving
        # https://github.com/auviitkgp/kraken_3.0/blob/indigo-devel/mission_planner_stack/mission_planner/src/marker.py
        # self.ipClient.send_goal(goal,done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        self.move_base.send_goal(goal)


    def getState(self):
        return self.move_base.get_state() == GoalStatus.SUCCEEDED
