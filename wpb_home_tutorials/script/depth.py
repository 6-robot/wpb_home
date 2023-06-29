#!/home/radiance/miniconda3/envs/ros/bin/python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from wpb_home_behaviors.msg import Coord
import math

_cv_bridge = CvBridge()

depth = None



def show(image_msg):
    global depth
    image = _cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    _, depth = cv2.threshold(image, 4000, 4000, cv2.THRESH_TRUNC)
    depth = cv2.resize(image, (106, 128))
    Xmin = np.min(image)
    Xmax = np.max(image)
    # 将数据映射到[-1,1]区间 即a=-1，b=1
    a = 0
    b = 255
    image = a + (b - a) / (Xmax - Xmin) * (image - Xmin)
    cv2.imshow('hh', image)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('Get_depth', anonymous=True)
    rate = rospy.Rate(500)

    human_point_pub = rospy.Publisher('/Human_pose', Coord, queue_size=1)
    # rospy.Subscriber("/kinect2/sd/image_color_rect", Image, getPositionCallback)
    rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, show)
    rospy.spin()
