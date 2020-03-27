#!/usr/bin/env python
# -*- coding: utf8 -*-

# Title:        The Detect objects stack for state machine at StarLine competition
# File:         Detect_objects.py
# Date:         2020-03-26
# Author:       Artem Smirnov @urpylka, Artur Golubtsov @goldarte
# Description:  Detect objects

import rospy
import numpy as np
import cv2
from enum import Enum
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger

detected_stop = rospy.ServiceProxy('/detected_stop', Trigger)
detected_redlight = rospy.ServiceProxy('/detected_redlight', Trigger)

class DetectObjects(object):

    def detectedStopSign(self):
        result = False
        try:
            result = detected_stop().success
        except rospy.ServiceException:
            result = False
        return result


    def detectedSemaphoreRed(self):
        return False
        # return detected_redlight().success

if __name__ == "__main__":
    d = DetectObjects()
    print("Stop sign detected: {} | Semafore red detected: {}".format(
            d.detectedStopSign(), d.detectedSemaphoreRed()
    ))
