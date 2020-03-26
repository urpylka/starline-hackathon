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

from enum import Enum
class SemaphoreState(Enum):
    RED = 1
    GREEN = 2
    YELLOW = 3


class DetectObjects(object):
    # https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020/blob/master/turtlebot3_autorace_traffic_light/turtlebot3_autorace_traffic_light_detect/nodes/detect_traffic_light

    def __init__(self):
        # Stop robot when catched Ctrl-C or failure
        # rospy.on_shutdown(self.cancelGoal)

        self.green_count = 0
        self.yellow_count = 0
        self.red_count = 0
        self.stop_count = 0
        rospy.sleep(1)

        self.cvBridge = CvBridge()
        self.cv_image = cv2.imread('dataset/traffic1.png')

        ###############

        self.hue_red_l = rospy.get_param("~detect/lane/red/hue_l", 0)
        self.hue_red_h = rospy.get_param("~detect/lane/red/hue_h", 30)
        self.saturation_red_l = rospy.get_param("~detect/lane/red/saturation_l", 22)
        self.saturation_red_h = rospy.get_param("~detect/lane/red/saturation_h", 203)
        self.lightness_red_l = rospy.get_param("~detect/lane/red/lightness_l", 106)
        self.lightness_red_h = rospy.get_param("~detect/lane/red/lightness_h", 255)

        self.hue_yellow_l = rospy.get_param("~detect/lane/yellow/hue_l", 7)
        self.hue_yellow_h = rospy.get_param("~detect/lane/yellow/hue_h", 26)
        self.saturation_yellow_l = rospy.get_param("~detect/lane/yellow/saturation_l", 7)
        self.saturation_yellow_h = rospy.get_param("~detect/lane/yellow/saturation_h", 199)
        self.lightness_yellow_l = rospy.get_param("~detect/lane/yellow/lightness_l", 220)
        self.lightness_yellow_h = rospy.get_param("~detect/lane/yellow/lightness_h", 255)

        self.hue_green_l = rospy.get_param("~detect/lane/green/hue_l", 39)
        self.hue_green_h = rospy.get_param("~detect/lane/green/hue_h", 81)
        self.saturation_green_l = rospy.get_param("~detect/lane/green/saturation_l", 177)
        self.saturation_green_h = rospy.get_param("~detect/lane/green/saturation_h", 255)
        self.lightness_green_l = rospy.get_param("~detect/lane/green/lightness_l", 106)
        self.lightness_green_h = rospy.get_param("~detect/lane/green/lightness_h", 255)

        self.is_calibration_mode = rospy.get_param("~is_detection_calibration_mode", False)

        ###############

    def detectedStopSign(self):
        return False


    def detectedSemaphore(self):
        return False


    def getStateSemaphore(self):
        return SemaphoreState.GREEN


    def fnFindTrafficLight(self):
        cv_image_mask = self.fnMaskGreenTrafficLight()
        cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

        status1 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'green')
        if status1 == 1 or status1 == 5:
            rospy.loginfo("DetectObjects: Detect GREEN")
            self.stop_count = 0
            self.green_count += 1
        else:
            self.green_count = 0

            cv_image_mask = self.fnMaskYellowTrafficLight()
            cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

            status2 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'yellow')
            if status2 == 2:
                rospy.loginfo("DetectObjects: Detect YELLOW")
                self.yellow_count += 1
            else:
                self.yellow_count = 0

                cv_image_mask = self.fnMaskRedTrafficLight()
                cv_image_mask = cv2.GaussianBlur(cv_image_mask,(5,5),0)

                status3 = self.fnFindCircleOfTrafficLight(cv_image_mask, 'red')
                if status3 == 3:
                    rospy.loginfo("DetectObjects: Detect RED")
                    self.red_count += 1
                elif status3 == 4:
                    self.red_count = 0
                    self.stop_count += 1
                else:
                    self.red_count = 0
                    self.stop_count = 0

        if self.green_count >= 3:
            rospy.loginfo("DetectObjects: GREEN")
            return "GREEN"

        if self.yellow_count >= 3:
            rospy.loginfo("DetectObjects: YELLOW")
            return "YELLOW"

        if self.red_count >= 3:
            rospy.loginfo("DetectObjects: RED")
            return "RED"

    def fnMaskRedTrafficLight(self):
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

    def fnMaskYellowTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes yellow light filtered image in compressed type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes yellow light filtered image in raw type
                self.pub_image_yellow_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask

    def fnMaskGreenTrafficLight(self):
        image = np.copy(self.cv_image)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_green_l
        Hue_h = self.hue_green_h
        Saturation_l = self.saturation_green_l
        Saturation_h = self.saturation_green_h
        Lightness_l = self.lightness_green_l
        Lightness_h = self.lightness_green_h

        # define range of green color in HSV
        lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_green = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        if self.is_calibration_mode == True:
            if self.pub_image_type == "compressed":
                # publishes green light filtered image in compressed type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))

            elif self.pub_image_type == "raw":
                # publishes green light filtered image in raw type
                self.pub_image_green_light.publish(self.cvBridge.cv2_to_imgmsg(mask, "mono8"))

        mask = cv2.bitwise_not(mask)

        return mask

    def fnFindCircleOfTrafficLight(self, mask, find_color):
        status = 0

        params=cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50
        params.maxArea = 600

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.4

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.6

        det = cv2.SimpleBlobDetector_create(params)
        keypts = det.detect(mask)
        frame = cv2.drawKeypoints(self.cv_image,keypts,np.array([]),(0,255,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        col1 = 180
        col2 = 270
        col3 = 305

        low1 = 50
        low2 = 170
        low3 = 170

        # if Detected more than 1 light
        for i in range(len(keypts)):
            self.point_col = int(keypts[i].pt[0])
            self.point_low = int(keypts[i].pt[1])
            if self.point_col > col1 and self.point_col < col2 and self.point_low > low1 and self.point_low < low2:
                if find_color == 'green':
                    status = 1
                elif find_color == 'yellow':
                    status = 2
                elif find_color == 'red':
                    status = 3
            elif self.point_col > col2 and self.point_col < col3 and self.point_low > low1 and self.point_low < low3:
                if find_color == 'red':
                    status = 4
                elif find_color == 'green':
                    status = 5
            else:
                status = 6

        return status

if __name__ == "__main__":
    d = DetectObjects()
    print(d.fnFindTrafficLight())
