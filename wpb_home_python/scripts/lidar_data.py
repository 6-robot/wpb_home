#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import LaserScan

# 激光雷达回调函数
def cbScan(msg):
    rospy.loginfo("雷达数据个数 = %d",len(msg.ranges))
    rospy.logwarn("正前方测距数值 = %.2f",msg.ranges[180])

# 主函数
if __name__ == "__main__":
    rospy.init_node("lidar_data")
    # 订阅激光雷达的数据话题
    lidar_sub = rospy.Subscriber("scan",LaserScan,cbScan,queue_size=10)
    rospy.spin()