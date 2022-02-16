#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

capture_one_frame = True

# 彩色图像回调函数
def cbImage(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # 弹出窗口显示图片
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)
    # 保存图像到文件
    global capture_one_frame
    if capture_one_frame == True:
        cv2.imwrite('/home/robot/1.jpg', cv_image)
        rospy.logwarn("保存图片成功！")
        capture_one_frame = False

# 主函数
if __name__ == "__main__":
    rospy.init_node("image_data")
    # 订阅机器人视觉传感器Kinect2的图像话题
    image_sub = rospy.Subscriber("/kinect2/hd/image_color_rect",Image,cbImage,queue_size=10)
    rospy.spin()