#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String

# 导航结果回调函数
def resultNavi(msg):
    rospy.loginfo("导航结果 = %s",msg.data)

if __name__ == "__main__":
    rospy.init_node("map_tools_navi")
    # 发布航点名称话题
    waypoint_pub = rospy.Publisher("/waterplus/navi_waypoint",String,queue_size=10)
    # 订阅导航结果话题
    result_sub = rospy.Subscriber("/waterplus/navi_result",String,resultNavi,queue_size=10)
    # 延时1秒钟，让后台的话题发布操作能够完成
    rospy.sleep(1)
    # 构建航点名称消息包
    msg = String()
    # msg.data = '1'
    msg.data = 'kitchen'
    # 发送航点名称消息包
    waypoint_pub.publish(msg)
    # 构建循环让程序别退出，等待导航结果
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()