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
        print command
        self.stop_state = True

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

st = StateMachine()



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