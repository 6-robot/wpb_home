#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("velocity_command")
    # 发布速度控制话题
    vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    # 构建速度消息包并赋值
    msg = Twist()
    msg.linear.x = 0.1
    # msg.angular.z =0.1
    # 构建发送频率对象
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("发送一个速度消息包")
        vel_pub.publish(msg)
        rate.sleep()