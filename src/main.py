#! /usr/bin/env python
# -*- coding: utf8 -*-

import sys
reload(sys)
sys.setdefaultencoding('utf8')

from espeak import espeak

import time
import threading
import traceback

import os, json

import telepot
from telepot.loop import MessageLoop

import rospy
# Header
from std_msgs.msg import *
# PoseWithCovarianceStamped
from geometry_msgs.msg import *
# MoveBaseActionGoal MoveBaseGoal 
from move_base_msgs.msg import *
import actionlib
# GoalID
from actionlib_msgs.msg import *

# https://dev.to/karn/building-a-simple-state-machine-in-python

class AbstractState(object):
    stop_state = False

    def __init__(self, _sm):
        print "Processing current state: " + str(self)

        t = threading.Thread(target=self.run, args=(_sm,))
        t.daemon = True
        t.start()

    def run(self, _sm):
        raise NotImplementedError()

    def exec_command(self, array_command):
        raise NotImplementedError()

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__

class INIT(AbstractState):
    def run(self, _sm):
        _sm.new_state(IDLE(_sm))
    def exec_command(self, array_command):
        print "Не могу выполнить команду: " + array_command[0]

class IDLE(AbstractState):
    def run(self, _sm):
        while True:
            if not self.stop_state:
                time.sleep(1)
            else:
                _sm.new_state(DELIVERY(_sm))
                break
    def exec_command(self, array_command):
        print "Command: " + array_command[0]
        command = array_command[0]
        if command == '/status':
            say("Отвали человек, мой статус недоступен")
            _tbot.sendMessage(CHAT_ID, "Здесь должен быть статус (заряд батареи)")
        elif command == '/go':
            say("Опять эти кожаные уюлюдки нарезали мне задач")
            _tbot.sendMessage(CHAT_ID, "Доставка пива в дом" + array_command[1])
            global cur_pose
            cur_pose = array_command[1]
            self.stop_state = True
        elif command == '/save':
            say("Я запомнил, где ты живешь")
            _tbot.sendMessage(CHAT_ID, "Запомнить где находится дом " + array_command[1])
            global my_poses
            global temp_pose
            my_poses[array_command[1]] = temp_pose
            global file_points
            with open(file_points, 'w') as outfile:
                json.dump(my_poses, outfile)
        else:
            _tbot.sendMessage(CHAT_ID, 'Ошибка 3! Некорректная команда: ' + command)

class DELIVERY(AbstractState):
    def run(self, _sm):

        # Customize the following values so they are appropriate for your location
        # pos = {'x': 1.22, 'y' : 2.56}
        # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        # my_poses = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4'])
        global my_poses
        global cur_pose
        global navigator
        # print my_poses
        success = navigator.goto(my_poses[cur_pose])

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
            print("Hooray, reached the desired pose")
            _sm.new_state(IDLE(_sm))
        else:
            rospy.loginfo("The base failed to reach the desired pose")
            print("The base failed to reach the desired pose")

            _sm.new_state(IDLE(_sm))
            # TODO: FAILSAFE

        # goto = rospy.Publisher('/move_base/goal', _MoveBaseActionGoal)
        # goto.publish()
                
        # while True:
        #     if not self.stop_state:
        #         time.sleep(1)
        #     else:
        #         _sm.new_state(IDLE(_sm))
        #         break
    def exec_command(self, array_command):
        print "Команда не будет выполнена: " + array_command[0]

class StateMachine(object):
    def __init__(self):
        self.state = INIT(self)

    def new_state(self, _state):
        self.state = _state

    def new_command(self, command):
        array = command.split(' ')
        self.state.exec_command(array)

def say(word):
    espeak.set_voice("ru")
    espeak.set_parameter(espeak.Parameter.Rate, 10, False)
    espeak.rate = 40
    espeak.synth(word)

    time.sleep(3)

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

    def goto(self, _pose):
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

temp_pose = None
my_poses = None
st = None
_tbot = None
CHAT_ID = None
navigator = None

def get_cur_pose(data):
    global temp_pose
    temp_pose = data.pose.pose

def handle(msg):
    """
    хендлер выполняется в отдельном потоке,
    вызывается событием на подобие блокирующей очереди
    """
    content_type, chat_type, chat_id = telepot.glance(msg)
    print(content_type, chat_type, chat_id)
    global CHAT_ID
    global _tbot
    global st

    if chat_id == CHAT_ID:
        if content_type == 'text':
            st.new_command(msg['text'])
        else:
            _tbot.sendMessage(CHAT_ID, "Ошибка 2! Неверный тип: только text и location")
    else:
        _tbot.sendMessage(CHAT_ID, "Ошибка 1! Access error!")

def load_param(param, default=None):
    if rospy.has_param(param):
        return rospy.get_param(param)
    elif not default is None:
        print("Param: " + str(param) + " not set & use default value: " + str(default))
        return rospy.get_param(param, default)
    else:
        print("Error: " + str(param) + " not set & have not default value")
        return None
        # raise SystemExit

def main():
    rospy.init_node('turtle_express')
    rospy.loginfo('Inited node turtle_express')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_cur_pose)
    global st
    global navigator

    global TOKEN
    global CHAT_ID
    global PROXY
    global my_poses
    global cur_pose

    global file_points
    file_points = "./urpylka_points.json"
    # my_poses = dict() if not os.path.isfile(file_points) else dict(json.load(file_points))
    try:
        with open(file_points, 'r') as infile:
            my_poses = json.load(infile)
    except:
        my_poses = dict()

    st = StateMachine()
    navigator = GoToPose()

    TOKEN = load_param('~token')
    CHAT_ID = load_param('~chat_id')
    PROXY = load_param('~proxy')
    if PROXY != None: telepot.api.set_proxy(PROXY)

    global _tbot
    _tbot = telepot.Bot(TOKEN)
    _tbot.sendMessage(CHAT_ID, "I am online")
    MessageLoop(_tbot, handle).run_as_thread()

    rospy.spin()

if __name__ == '__main__':
    main()
