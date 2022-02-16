#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# 三维点云回调函数
def callbackPointcloud(msg):
    field_num = len(msg.fields)
    rospy.logwarn("PointField元素个数 = %d",field_num) 
    print("--------------------------------")
    for f in msg.fields:
        rospy.loginfo("name = %s", f.name)
        rospy.loginfo("offset = %d", f.offset)
        rospy.loginfo("datatype = %d", f.datatype)
        rospy.loginfo("count = %d", f.count)
        print("--------------------------------")

# 主函数
if __name__ == "__main__":
    rospy.init_node("pointcloud_field")
    # 订阅机器人视觉传感器Kinect2的三维点云话题
    pc_sub = rospy.Subscriber("/kinect2/sd/points",PointCloud2, callbackPointcloud,queue_size=10)
    rospy.spin()