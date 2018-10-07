#! /usr/bin/env python
# -*- coding: utf8 -*-

import time
import threading

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

    def exec_command(self, command):
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
    def exec_command(self, command):
        print "Не могу выполнить команду: " + command

class IDLE(AbstractState):
    def run(self, _sm):
        while True:
            if not self.stop_state:
                time.sleep(1)
            else:
                _sm.new_state(DELIVERY(_sm))
                break
    def exec_command(self, command):
        print "Commend: " + command
        if command == '/status':
            _tbot.sendMessage(CHAT_ID, "Здесь должен быть статус (заряд батареи)")
        elif command == 'beer1':
            _tbot.sendMessage(CHAT_ID, "Доставка пива в дом №1")
            # self.stop_state = True
        elif command == '/remind_beer1':
            _tbot.sendMessage(CHAT_ID, "Запомнить где находится дом №1")
        else:
            _tbot.sendMessage(CHAT_ID, 'Ошибка 3! Некорректная команда: ' + command)

class DELIVERY(AbstractState):
    def run(self, _sm):
        while True:
            if not self.stop_state:
                time.sleep(1)
            else:
                _sm.new_state(IDLE(_sm))
                break
    def exec_command(self, command):
        print command
        self.stop_state = True

class StateMachine(object):
    def __init__(self):
        self.state = INIT(self)

    def new_state(self, _state):
        self.state = _state

    def new_command(self, command):
        self.state.exec_command(command)

    def pr():
        print "weqfqwefqw"

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
st = None
_tbot = None
CHAT_ID = None
navigator = None

def get_cur_pose(data):
    temp_pose = data.pose.pose

 def handle(msg):
        """
        хендлер выполняется в отдельном потоке,
        вызывается событием на подобие блокирующей очереди
        """
        content_type, chat_type, chat_id = telepot.glance(msg)
        print(content_type, chat_type, chat_id)
        if chat_id == CHAT_ID:
            if content_type == 'text':
                print "fadsfasffasgsgasgasg"
                st.pr()
                # st.new_command(msg['text'])
            else:
                _tbot.sendMessage(CHAT_ID, "Ошибка 2! Неверный тип: только text и location")
        else:
            _tbot.sendMessage(CHAT_ID, "Ошибка 1! Access error!")

def main():
    rospy.init_node('turtle_express')
    rospy.loginfo('Inited node turtle_express')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_cur_pose)

    st = StateMachine()

    TOKEN = load_param('~token')
    CHAT_ID = load_param('~chat_id')
    PROXY = load_param('~proxy')
    _tbot = telepot.Bot(TOKEN)
    if PROXY != None: telepot.api.set_proxy(PROXY)
    MessageLoop(_tbot, handle).run_as_thread()

    navigator = GoToPose()

    # Customize the following values so they are appropriate for your location
    # pos = {'x': 1.22, 'y' : 2.56}
    # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
    # my_pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4'])

    my_pose = temp_pose

    time.sleep(10)

    success = navigator.goto(my_pose)

    if success:
        rospy.loginfo("Hooray, reached the desired pose")
    else:
        rospy.loginfo("The base failed to reach the desired pose")

    # goto = rospy.Publisher('/move_base/goal', _MoveBaseActionGoal)
    # goto.publish()

    rospy.spin()

if __name__ == '__main__':
    main()

time.sleep(3)
st.new_command("412423")
time.sleep(3)
st.new_command("412423")
time.sleep(3)
st.new_command("412423")
time.sleep(3)
st.new_command("412423")
time.sleep(3)
st.new_command("412423")
time.sleep(3)
