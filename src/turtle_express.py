#!/usr/bin/env python
# -*- coding: utf8 -*-

import time, traceback, sys, math, socket, re, os, json
from enum import Enum
import telepot
import http.client

from telegrambot import TelegramBot
# ====================================================================================
# Устранение проблем с кодировкой UTF-8
# http://webhamster.ru/mytetrashare/index/mtb0/13566385393amcr1oegx
reload(sys)
sys.setdefaultencoding('utf8')
# ====================================================================================

class States(Enum):
    """
    States of drone
    """
    INIT = 1
    IDLE = 2
    RTL = 3
    GOTO = 4

def check_internet():
    try:
        socket.gethostbyaddr('ya.ru')
    except socket.gaierror:
        return False
    return True

def wait_internet():
    while not check_internet():
        time.sleep(1)

def get_ip():
    conn = http.client.HTTPConnection("ipv4.icanhazip.com")
    try:
        conn.request("GET", "/")
        http_responce = conn.getresponse().read()
        IP_REGEX = r'^(([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$'
        if re.match(IP_REGEX, http_responce):
            return http_responce
        else:
            return "не удалось вычислить IP адрес"
    except Exception:
        return "не удалось вычислить IP адрес"

class TurtleExpress(object):
    """
    This class consists of logic for states (methods 'do_*' & dependencies) and method 'new_command'
    """
    def __init__(self, token, chat_id, proxy, debug):

        # Params of info-messages
        self.bot = TelegramBot(token, chat_id, proxy, debug, self)
        self.debug_message = self.bot.debug_message

        # Params of states machine
        self.current_state = States.INIT
        self.stop_state = False

    def new_state(self, new_state):
        self.stop_state = True
        self.next_state = new_state

    @property
    def get_status(self):
        buf = "\nGPS: %s" % self._vehicle.gps_0 + \
              "\nBattery: %s" % self._vehicle.battery + \
              "\nLast Heartbeat: %s" % self._vehicle.last_heartbeat + \
              "\nIs Armable?: %s" % self._is_armable + \
              "\nSystem status: %s" % self._vehicle.system_status.state + \
              "\nMode: %s" % self._vehicle.mode.name + \
              "\nGlobal Location: %s" % self._vehicle.location.global_frame + \
              "\nGlobal Relative Location: %s" % self._vehicle.location.global_relative_frame + \
              "\nLocal Location: %s" % self._vehicle.location.local_frame + \
              "\nAttitude: %s" % self._vehicle.attitude + \
              "\nHeading: %s" % self._vehicle.heading + \
              "\nGroundspeed: %s" % self._vehicle.groundspeed + \
              "\nAirspeed: %s" % self._vehicle.airspeed
        return buf

    def do_INIT(self):
        self.current_state = States.INIT

        wait_internet()

        self.debug_message("TurtleExpress is online: %s" % get_ip())
        self.debug_message("STATE = %s" % self.current_state)

        self.bot.start_handler()
        time.sleep(1.5)  # время на ответ сообщений пришедших в выключенный период

        self.next_state = States.IDLE

    def do_IDLE(self):
        self.current_state = States.IDLE
        self.next_state = States.IDLE
        self.debug_message("STATE = %s" % self.current_state)
        self.stop_state = False

        while True:
            if not self.stop_state:
                time.sleep(1)
            else:
                self.debug_message("Прерывание состояния %s" % self.current_state + " переключение в состояние %s" % self.next_state)
                return self.next_state
        self.debug_message("Успешное завершение состояния %s" % self.current_state + " переключение в состояние %s" % self.next_state)
        return self.next_state

    def do_GOTO(self):
        self.current_state = States.GOTO
        self.next_state = States.IDLE
        self.stop_state = False
        self.debug_message("STATE = %s" % self.current_state)

        while self._is_arrived(self._goto_location.lat, self._goto_location.lon, self._goto_location.alt):
            if not self.stop_state:
                self.debug_message("До точки назначения: " + get_distance_metres(self._goto_location, self._vehicle.location.global_relative_frame) + "м")
                time.sleep(1)
            else:
                self.debug_message("Прерывание состояния %s" % self.current_state + " переключение в состояние %s" % self.next_state)
                return self.next_state
        self.debug_message("Успешное завершение состояния %s" % self.current_state + " переключение в состояние %s" % self.next_state)
        self._need_hover = False
        return self.next_state

    def do_RTL(self):
        self.current_state = States.RTL
        self.debug_message("STATE = %s" % self.current_state)
        self.stop_state = False
        self.next_state = States.IDLE

        while not self.onLand:
            if not self.stop_state:
                #self.debug_message('Waiting for ' + self.current_state)
                time.sleep(1)
            else:
                self.debug_message("Прерывание состояния %s" % self.current_state + " переключение в состояние %s" % self.next_state)
                return self.next_state
        self.debug_message("Успешное завершение состояния %s" % self.current_state + " переключение в состояние %s" % self.next_state)
        return self.next_state

def main():
    turtleExpress = None
    try:
        turtleExpress = TurtleExpress(CONNECTION_STR, TOKEN, CHAT_ID, PROXY, DEBUG)
        while True:
            try:
                if turtleExpress.current_state == States.INIT:
                    turtleExpress.do_INIT()
                elif turtleExpress.current_state == States.IDLE:
                    turtleExpress.do_IDLE()
                elif turtleExpress.current_state == States.GOTO:
                    turtleExpress.do_GOTO()
                elif turtleExpress.current_state == States.RTL:
                    turtleExpress.do_RTL()
            except Exception as ex:
                turtleExpress.debug_message("Ошибка в состоянии %s" % turtleExpress.current_state + ":\n" + ex.message + "\n" + traceback.format_exc() + "\n")
    except KeyboardInterrupt:
        pass
    finally:
        pass
        # TODO something same
        # if turtleExpress != None: turtleExpress.disconnect()

if __name__ == "__main__":
    main()
