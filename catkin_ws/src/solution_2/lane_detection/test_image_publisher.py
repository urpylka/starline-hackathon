import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

test_image = cv2.imread('test_image.jpg')
image_message = bridge.cv2_to_imgmsg(test_image, "bgr8")

def pub_image(event):
    image_message.header.stamp = rospy.Time.now()
    image_pub.publish(image_message)

rospy.init_node('test_image_pub')

image_pub = rospy.Publisher('test_image', Image, queue_size=1)

rospy.Timer(rospy.Duration(0.1), pub_image)

rospy.spin()