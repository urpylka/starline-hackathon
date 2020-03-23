#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from solution_1.cfg import lane_image_prepare_Config
from camera_undistort import get_matrix_and_distortions_ros, get_undistorted_image

class DynParams:

    default_dict = {
        "undistort_image": True,
        "thr_function": 1,
        "thr_max_val": 255,
        "thr_val": 127,
        "thr_type": cv2.THRESH_BINARY,
        "thr_type_flag": 0,
        "a_thr_type": cv2.THRESH_BINARY,
        "a_thr_method": cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        "a_thr_blocksize": 11,
        "a_thr_C": 2,
    }

    def __init__(self):
        for key, value in self.default_dict.items():
            setattr(self, key, value)

cv_bridge = CvBridge()
dyn_params = DynParams()
last_image = None
matrix = None
distortions = None

def image_callback(msg):
    global last_image
    last_image = msg

def dyn_param_callback(config, level):
    for key, value in config.items():
        setattr(dyn_params, key, value)
    return config

def threshold(cv2_image, params):
    try:
        if params.thr_function == 0:
            _, thr_image = cv2.threshold(cv2_image, params.thr_val, params.thr_max_val, params.thr_type+params.thr_type_flag)
        elif params.thr_function == 1:
            thr_image = cv2.adaptiveThreshold(cv2_image, params.thr_max_val, params.a_thr_method, params.a_thr_type, params.a_thr_blocksize, params.a_thr_C)
    except cv2.error as e:
        rospy.logerr("Can't threshold image. Error: {}".format(e))
        return cv2_image
    return thr_image

def prepare_image(cv2_image, params, matrix=None, distortions=None):
    undistorted_image = cv2_image
    if params.undistort_image:
        if matrix is not None and distortions is not None:
            undistorted_image = get_undistorted_image(cv2_image, matrix, distortions)
    gray = cv2.cvtColor(undistorted_image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    thr = threshold(blur, params)
    return thr

def pub_image(cv2_image, publisher, header):
    if publisher.get_num_connections() > 0:
        m = cv_bridge.cv2_to_imgmsg(cv2_image)
        m.header.stamp.secs = header.stamp.secs
        m.header.stamp.nsecs = header.stamp.nsecs
        publisher.publish(m)


if __name__ == "__main__":

    rospy.init_node('lane_detector')
    srv = Server(lane_image_prepare_Config, dyn_param_callback)

    ns = rospy.get_namespace()
    rate_value = rospy.get_param('~rate', 30.0)

    camera_topic = rospy.get_param('~camera_topic', '')

    camera_info_topic_name = rospy.get_param('~camera_info_topic_name', 'camera_info')
    camera_info_topic = camera_topic+'/'+camera_info_topic_name if camera_topic else camera_info_topic_name

    image_sub_topic_name = rospy.get_param('~image_sub_topic_name', 'image_raw')
    image_sub_topic = camera_topic+'/'+image_sub_topic_name if camera_topic else image_sub_topic_name

    image_pub_topic_name = rospy.get_param('~image_pub_topic_name', 'prepared_image/lane')
    image_pub_topic = camera_topic+'/'+image_pub_topic_name if camera_topic else image_pub_topic_name

    image_sub = rospy.Subscriber(image_sub_topic, Image, image_callback)
    image_pub = rospy.Publisher(image_pub_topic, Image, queue_size=1)

    matrix, distortions = get_matrix_and_distortions_ros(camera_info_topic)

    rate = rospy.Rate(rate_value)

    while not rospy.is_shutdown():
        rate.sleep()

        if last_image is None:
            continue

        header = last_image.header

        last_image_cv2 = cv_bridge.imgmsg_to_cv2(last_image)

        prepared_image = prepare_image(last_image_cv2, dyn_params, matrix, distortions)

        pub_image(prepared_image, image_pub, header)