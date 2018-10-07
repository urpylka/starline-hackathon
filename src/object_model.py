#! /usr/bin/env python
# -*- coding: utf8 -*-

import time
import threading

# https://dev.to/karn/building-a-simple-state-machine-in-python

class AbstractState(object):
    stop_state = False

    def __init__(self):
        print "Processing current state: " + str(self)

    def do(self, _sm):
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
        _sm.new_state(IDLE())
    def exec_command(self, command):
        print "Не могу выполнить команду: " + command

class IDLE(AbstractState):
    def run(self, _sm):
        print "IIIIIIDDDLLLEEEE"
        while True:
            if not self.stop_state:
                time.sleep(1)
            else:
                _sm.new_state(IDLE())
    def exec_command(self, command):
        print command
        self.stop_state = True

class GOTO(AbstractState):
    def run(self, _sm):
        print "GOTOTOT"
        self.stop_state = True
        while True:
            if not self.stop_state:
                time.sleep(1)
            else:
                _sm.new_state(IDLE())
                break
    def exec_command(self, command):
        print command
        self.stop_state = True

class StateMachine(object):
    def __init__(self):
        self.state = INIT()
        self.state.do(self)

    def new_state(self, _state):
        self.state = _state

    def new_command(self, command):
        self.state.exec_command(command)

st = StateMachine()
time.sleep(1)
st.new_command("wefaf")
time.sleep(3)
st.new_command("412423")
