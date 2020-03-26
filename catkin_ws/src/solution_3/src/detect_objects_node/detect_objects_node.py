#!/usr/bin/env python
# -*- coding: utf8 -*-

# Title:        The Detect objects stack for state machine at StarLine competition
# File:         detect_objects_node.py
# Date:         2020-03-26
# Author:       Artur Golubtsov @goldarte
# Description:  Detect objects

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from find_object_2d.msg import ObjectsStamped
from std_srvs.srv import Trigger, TriggerResponse
from dynamic_reconfigure.server import Server
from solution_3.cfg import dyn_detect_Config

class DynParams:

    default_dict = {
        "light_area":0.2,
        "object_area":0.1,
        "red_h_l":0,
        "red_h_h":30,
        "red_s_l":22,
        "red_s_h":203,
        "red_l_l":106,
        "red_l_h":255
    }

    def __init__(self):
        for key, value in self.default_dict.items():
            setattr(self, key, value)

cv_bridge = CvBridge()
dyn_params = DynParams()
last_image = None
last_image_cv2 = None
last_image_area = 0
last_objects = None
detected_objects = None

def image_callback(msg):
    global last_image
    last_image = msg

def objects_callback(msg):
    global last_objects
    last_objects = msg
    rospy.loginfo(last_objects.objects.data)

def dyn_param_callback(config, level):
    for key, value in config.items():
        setattr(dyn_params, key, value)
    return config

def detect_objects(event):
    global cv_bridge, last_objects, last_image, last_image_cv2, last_image_area, detected_objects
    if last_objects is None:
        rospy.loginfo('No objects msg')
        return

    if not last_objects.objects.data:
        rospy.loginfo('No objects')
        return

    if last_image is None:
        rospy.loginfo('No image msg')
        return

    last_image_cv2 = cv_bridge.imgmsg_to_cv2(last_image)
    last_image_area = last_image_cv2.shape[0]*last_image_cv2.shape[1]

    detected_objects = select_objects(last_objects.objects.data)
    debug_image = draw_frames(detected_objects, last_image_cv2)

    image_message = cv_bridge.cv2_to_imgmsg(debug_image, "bgr8")
    image_message.header.stamp = rospy.Time.now()
    debug_pub.publish(image_message)

def select_objects(objects_array, sort_by = 'max_area'):
    objects = {}
    objects['stop'] = []
    objects['traffic_light'] = []
    for i in range(0, len(objects_array), 12):
        if objects_array[0] == 1.0:
            objects['stop'].append(objects_array[i+1:i+12])
        else:
            objects['traffic_light'].append(objects_array[i+1:i+12])
    selected_objects = {}
    selected_objects['stop'] = []
    selected_objects['traffic_light'] = []
    if objects:
        rospy.loginfo(objects)
        try:
            for key, value in objects.items():
                if value:
                    selected_objects[key] = sorted(value, key=lambda data: data[0]*data[1])[-1]
        except ValueError as e:
            rospy.logwarn(e)
    rospy.loginfo("Selected objects: {}".format(selected_objects))
    return selected_objects

def detect_object_service(request):
    global detected_objects, last_image_area
    responce = TriggerResponse()
    if detected_objects['stop']:
        object_area = detected_objects['stop'][0]*detected_objects['stop'][1]
        if object_area > dyn_params.object_area*last_image_area:
            responce.success = True
            responce.message = "Stop sign detected!"
        else:
            responce.success = False
            responce.message = "Stop sign is detected, but small!"
    else:
        responce.success = False
        responce.message = "Stop sign is not detected!"
    return responce

def detect_light_service(request):
    global detected_objects, last_image_area, last_image_cv2
    responce = TriggerResponse()
    if detected_objects['traffic_light']:
        light_area = detected_objects['traffic_light'][0]*detected_objects['traffic_light'][1]
        if light_area > dyn_params.light_area*last_image_area:
            if detect_red(detected_objects, last_image_cv2):
                responce.success = True
                responce.message = "Traffic red light detected!"
            else:
                responce.success = False
                responce.message = "Traffic light detected, but not red!"
        else:
            responce.success = False
            responce.message = "Traffic light detected, but small!"
    else:
        responce.success = False
        responce.message = "Traffic light is not detected!"
    return responce

def get_frame(object_detected):
    if object_detected:
        object_x = int(object_detected[8])
        object_y = int(object_detected[9])
        object_width = int(object_detected[0])*object_detected[2]
        object_height = int(object_detected[1])*object_detected[6]


def draw_frame(object_detected, cv2_image):
    if object_detected:
        color = (0, 0, 255)
        rectangled = cv2.rectangle(cv2_image, (object_x, object_y),
                    (object_x + object_width, object_y + object_height), color, 3)
        return rectangled
    else:
        return cv2_image

def draw_frames(objects, cv2_image):
    result = cv2_image.copy()
    for key, value in objects.items():
        result = draw_frame(value, result)
    return result

def get_red_mask(objects, cv2_image):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_red_l
        Hue_h = self.hue_red_h
        Saturation_l = self.saturation_red_l
        Saturation_h = self.saturation_red_h
        Lightness_l = self.lightness_red_l
        Lightness_h = self.lightness_red_h

        # define range of red color in HSV
        lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_red = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes red light filtered image in compressed type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes red light filtered image in raw type
                self.pub_image_red_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask


def detect_red(objects, cv2_image):

    return False


if __name__ == "__main__":
    rospy.init_node('object_detector')
    srv = Server(dyn_detect_Config, dyn_param_callback)

    image_sub_topic = rospy.get_param('~image_topic', 'test_image')
    rospy.loginfo("image_sub_topic: {}".format(image_sub_topic))

    debug_topic = rospy.get_param('~debug_topic', 'image_debug')
    rospy.loginfo("debug_topic: {}".format(debug_topic))

    objects_topic = rospy.get_param('~objects_topic', 'objectsStamped')
    rospy.loginfo("objects_topic: {}".format(objects_topic))

    image_sub = rospy.Subscriber(image_sub_topic, Image, image_callback)
    objects_sub = rospy.Subscriber(objects_topic, ObjectsStamped, objects_callback)
    debug_pub = rospy.Publisher(debug_topic, Image, queue_size=1)

    rospy.Service('detected_stop', Trigger, detect_object_service)
    rospy.Service('detected_redlight', Trigger, detect_light_service)

    rospy.Timer(rospy.Duration(0.1), detect_objects)

    rospy.spin()
