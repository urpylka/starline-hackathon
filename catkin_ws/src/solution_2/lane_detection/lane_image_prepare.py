#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from solution_2.cfg import lane_image_prepare_Config
from camera_undistort import get_matrix_and_distortions_ros, get_undistorted_image

class DynParams:

    default_dict = {
        "undistort_image": False,
        "publish_bin": True,
        "publish_bin_flat":True,
        "draw_perspective":True,
        "thr_function": 1,
        "thr_max_val": 255,
        "thr_val": 127,
        "thr_type": cv2.THRESH_BINARY,
        "thr_type_flag": 0,
        "a_thr_type": cv2.THRESH_BINARY,
        "a_thr_method": cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        "a_thr_blocksize": 11,
        "a_thr_C": 2,
        "expand_lr": 0,
        "t_top_width": 0.5,
        "t_top_v_pos": 0,
        "t_bottom_width": 1,
        "t_bottom_v_pos":0
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

def expand(cv2_image, t, b, l, r):
    result = cv2.copyMakeBorder(cv2_image, t, b, l, r, cv2.BORDER_CONSTANT)
    return result

def expand_lr(cv2_image, params):
    return expand(cv2_image, 0, 0, params.expand_lr, params.expand_lr)

def get_points(cv2_image, params):
    (h, w) = (cv2_image.shape[0], cv2_image.shape[1])
    p1 = (w*(1-params.t_top_width)/2, h*params.t_top_v_pos)
    p2 = (w*(1+params.t_top_width)/2, h*params.t_top_v_pos)
    p3 = (w*(1+params.t_bottom_width)/2, h*params.t_bottom_v_pos)
    p4 = (w*(1-params.t_bottom_width)/2, h*params.t_bottom_v_pos)
    return np.array((p1, p2, p3, p4), dtype = "float32")

def binarize_image(cv2_image, params):
    gray = cv2.cvtColor(cv2_image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)
    thr = threshold(blur, params)
    return thr

def draw_trapezoid(cv2_image, params):
    rect = get_points(cv2_image, params)
    (tl, tr, br, bl) = tuple(map(tuple, rect))
    cv2_image_t = cv2_image.copy()
    cv2_image_t = cv2.line(cv2_image, tl, tr, 127, 2)
    cv2_image_t = cv2.line(cv2_image, tr, br, 127, 2)
    cv2_image_t = cv2.line(cv2_image, br, bl, 127, 2)
    cv2_image_t = cv2.line(cv2_image, bl, tl, 127, 2)
    return cv2_image_t

def flatten_image(cv2_image, params):
    # obtain a consistent order of the points and unpack them
	# individually
    rect = get_points(cv2_image, params)
    (tl, tr, br, bl) = rect
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt((br[0] - bl[0]) ** 2 + (br[1] - bl[1]) ** 2)
    widthB = np.sqrt((tr[0] - tl[0]) ** 2 + (tr[1] - tl[1]) ** 2)
    maxWidth = max(int(widthA), int(widthB))
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt((tr[0] - br[0]) ** 2 + (tr[1] - br[1]) ** 2)
    heightB = np.sqrt((tl[0] - bl[0]) ** 2 + (tl[1] - bl[1]) ** 2)
    maxHeight = max(int(heightA), int(heightB))
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    # compute the perspective transform matrix and then apply it
    warp_matrix = cv2.getPerspectiveTransform(rect, dst)
    unwarp_matrix = cv2.getPerspectiveTransform(dst, rect)
    flat_img = cv2.warpPerspective(cv2_image, warp_matrix, (maxWidth, maxHeight))
    # return the flattened image and unwarp matrix
    return flat_img, unwarp_matrix

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

    rospy.loginfo("Camera topic: {} {}".format(camera_topic, bool(camera_topic)))

    camera_info_topic_name = rospy.get_param('~camera_info_topic_name', 'camera_info')
    camera_info_topic = camera_topic+'/'+camera_info_topic_name if camera_topic else camera_info_topic_name

    image_sub_topic_name = rospy.get_param('~image_sub_topic_name', 'image_raw')
    image_sub_topic = camera_topic+'/'+image_sub_topic_name if camera_topic else image_sub_topic_name
    rospy.loginfo("image_sub_topic: {}".format(image_sub_topic))

    bin_pub_topic_name = rospy.get_param('~bin_pub_topic_name', 'prepared_image/bin')
    bin_pub_topic = camera_topic+'/'+bin_pub_topic_name if camera_topic else bin_pub_topic_name

    bin_flat_pub_topic_name = rospy.get_param('~bin_flat_pub_topic_name', 'prepared_image/bin_flat')
    bin_flat_pub_topic = camera_topic+'/'+bin_flat_pub_topic_name if camera_topic else bin_flat_pub_topic_name

    image_sub = rospy.Subscriber(image_sub_topic, Image, image_callback)
    bin_pub = rospy.Publisher(bin_pub_topic, Image, queue_size=1)
    bin_flat_pub = rospy.Publisher(bin_flat_pub_topic, Image, queue_size=1)

    if dyn_params.undistort_image:
        matrix, distortions = get_matrix_and_distortions_ros(camera_info_topic)

    rate = rospy.Rate(rate_value)

    while not rospy.is_shutdown():
        rate.sleep()

        if last_image is None:
            print('No image')
            continue

        header = last_image.header

        last_image_cv2 = cv_bridge.imgmsg_to_cv2(last_image)

        cv2_image = last_image_cv2

        if dyn_params.undistort_image:
            cv2_image = get_undistorted_image(last_image_cv2, matrix, distortions)

        bin_image_ss = binarize_image(cv2_image, dyn_params)
        bin_image = expand_lr(bin_image_ss, dyn_params)

        if dyn_params.draw_perspective:
            bin_image = draw_trapezoid(bin_image, dyn_params)

        if dyn_params.publish_bin:
            pub_image(bin_image, bin_pub, header)

        bin_flat_image, unwarp_matrix = flatten_image(bin_image, dyn_params)

        if dyn_params.publish_bin_flat:
            pub_image(bin_flat_image, bin_flat_pub, header)