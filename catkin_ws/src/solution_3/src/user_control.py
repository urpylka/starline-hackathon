#!/usr/bin/env python
# -*- coding: utf8 -*-

# Title:        The class for manage to state machine at StarLine competition
# File:         user_control.py
# Date:         2020-03-27
# Author:       Artem Smirnov @urpylka
# Description:  Manager of the state machine

import os
import rospy
# import telepot
# from telepot.loop import MessageLoop
from std_srvs.srv import Trigger, TriggerResponse

class ObjectStorage():
    pass

class UserControl(object):
    def __init__(self, _M):
        self.M = _M

        rospy.Service("/fm_goto", Trigger, self.cbFmGoto)
        rospy.Service("/fm_idle", Trigger, self.cbFmIdle)
        rospy.Service("/fm_reset_odometry", Trigger, self.cbFmResetOdometry)

        self.getCredentionals()

        if not self.credentionals.token is None:
            if not self.credentionals.proxy is None:
                telepot.api.set_proxy(self.credentionals.proxy)

            self.tbot = telepot.Bot(self.credentionals.token)
            if not self.credentionals.chat_id == "":
                self.tbot.sendMessage(self.credentionals.chat_id, "Stimulator G is online!")
            MessageLoop(self.tbot, self.cdTg).run_as_thread()


    def getCredentionals(self):
        self.credentionals = ObjectStorage()
        self.credentionals.proxy = os.environ.get('proxy', None)
        self.credentionals.token = os.environ.get('token', None)
        self.credentionals.chat_id = os.environ.get('chat_id', "")
        self.credentionals.secret = os.environ.get('secret', "starline2020")


    def cbTg(self, msg):
        content_type, chat_type, chat_id = telepot.glance(msg)

        # print(content_type, chat_type, chat_id) # debug

        if chat_id == self.credentionals.chat_id:
            if content_type == 'text':
                self.M.command(msg['text'])
                self.tbot.sendMessage(chat_id, "The \"" + msg['text'] + "\" command has sent to the state machine.")

        else:
            if content_type == 'text':
                if msg['text'] == self.credentionals.secret:
                    self.credentionals.chat_id = chat_id
                    os.environ["chat_id"] = self.credentionals.chat_id
                    self.tbot.sendMessage(chat_id, "You have the permission to send commans to the state machine.")
                else:
                    self.tbot.sendMessage(chat_id, "You haven't the permission to send commans to the state machine.")


    def cbFmStart(self, request):
        self.M.command("goto")
        return TriggerResponse(
            success = True,
            message = "The \"goto\" command has sent to the state machine."
            )


    def cbFmStop(self, request):
        self.M.command("idle")
        return TriggerResponse(
            success = True,
            message = "The \"idle\" command has sent to the state machine."
            )


    def cbFmResetOdometry(self, request):
        self.M.command("reset_odometry")
        return TriggerResponse(
            success = True,
            message = "The \"reset_odometry\" command has sent to the state machine."
            )
