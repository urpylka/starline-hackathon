#!/usr/bin/env python
# -*- coding: utf8 -*-

# Title:        The abstarct classes for building the state machine at StarLine competition
# File:         state_machine.py
# Date:         2020-03-25
# Author:       Artem Smirnov @urpylka
# Description:  Abstract classes for building the state machine

import threading

class AbstractState(object):

    stop_state = False

    def __init__(self, _sm):
        print("Processing current state: " + str(self))

        t = threading.Thread(target=self.run, args=(_sm,))
        t.daemon = True
        t.start()

    def run(self, _sm):
        raise NotImplementedError()

    def command(self, array_command):
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


class ObjectStorage():
    pass


class StateMachine(object):

    # User storage for variables
    S = ObjectStorage()

    def __init__(self, _init_state):
        self.state = _init_state(self)

    def new_state(self, _state):
        self.state = _state

    def new_command(self, command):
        array = command.split(' ')
        self.state.command(array)
