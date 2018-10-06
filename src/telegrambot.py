#!/usr/bin/env python
# -*- coding: utf8 -*-

import time
import traceback
import telepot
from telepot.loop import MessageLoop

import sys
reload(sys)
sys.setdefaultencoding('utf8')

class TelegramBot(object):
    """
    Здесь находится метод для создания отладочных сообщений,
    а также здесь крутиться в отдельном потоке метод обработки
    пользовательских сообщений
    """

    def __init__(self, _token, _chat_id, _proxy, _debug, _turtle_express):
        self.bot = telepot.Bot(_token)
        self.chat_id = _chat_id
        self.debug = _debug
        self.turtle_express = _turtle_express
        if _proxy != None: telepot.api.set_proxy(_proxy)

    def debug_message(self, msg):
        if self.debug:
            try:
                print msg
                self.bot.sendMessage(self.chat_id, msg)
            except Exception as ex:
                print "Произошла ошибка при отправке сообщения:\n" + ex.message + "\n" + traceback.format_exc()
                time.sleep(2)

    def start_handler(self):
        MessageLoop(self.bot, self.handle).run_as_thread()

    def handle(self, msg):
        """
        хендлер выполняется в отдельном потоке,
        вызывается событием на подобие блокирующей очереди
        """
        content_type, chat_type, chat_id = telepot.glance(msg)
        if self.debug:
            print(content_type, chat_type, chat_id)
        if chat_id == self.chat_id:
            if content_type == 'text':
                self.debug_message(self.new_command(msg['text']))
            else:
                self.debug_message("Ошибка 2! Неверный тип: только text и location")
        else:
            self.debug_message("Ошибка 1! Access error!")

    def new_command(self, command, params = None):
        if self.turtle_express.current_state == States.INIT:
            return "Ошибка 3! Некорректная команда %s" % command + " для состояния %s" % self.turtle_express.current_state

        elif self.turtle_express.current_state == States.IDLE:
            if command == '/status':
                # вывод информации о коптере, ip, заряд батареи
                return "copter ip: %s" % get_ip() + \
                    "\nSTATE: %s" % self.turtle_express.current_state + \
                    "\n%s" % self.turtle_express.get_status
            elif command == 'create_mission':
                self.turtle_express.new_state(States.TAKEOFF)
                return self.turtle_express.create_mission(params['latitude'], params['longitude'])
            elif command == '/takeoff':
                self.turtle_express.new_state(States.TAKEOFF)
                #return "Взлет из состояния: %s" % self.turtle_express.current_state
            elif command == 'get_location':
                lat, lon = self.turtle_express.get_location()
                self.bot.sendLocation(self.chat_id, lat, lon)
            else:
                return 'Ошибка 3! Некорректная команда ' + command + " для состояния %s" % self.turtle_express.current_state

        else:
            return "Ошибка 4! Нет обработчика для состояния: " + self.turtle_express.current_state