#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# 三维点云回调函数
def callbackPointcloud(msg):
    # 从点云中提取三维坐标数值
    pc = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
    point_cnt = 0
    for p in pc:
        rospy.loginfo("第%d点坐标 (x= %.2f, y= %.2f, z= %.2f)",point_cnt, p[0],p[1],p[2] )
        point_cnt += 1

# 主函数
if __name__ == "__main__":
    rospy.init_node("pointcloud_data")
    # 订阅机器人视觉传感器Kinect2的三维点云话题
    pc_sub = rospy.Subscriber("/kinect2/sd/points",PointCloud2, callbackPointcloud,queue_size=10)
    rospy.spin()