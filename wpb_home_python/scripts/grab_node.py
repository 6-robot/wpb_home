#!/usr/bin/env python
# coding=utf-8

import rospy
from wpb_home_behaviors.msg import Coord
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# 标记变量，是否处于抓取过程当中
grabbing = False

# 物品检测回调函数
def cbObject(msg):
    global grabbing
    if grabbing == False:
        num = len(msg.name)
        rospy.loginfo("检测物品个数 = %d", num)
        if num > 0:
            rospy.logwarn("抓取目标 %s! (%.2f , %.2f , %.2f)",msg.name[0],msg.x[0],msg.y[0],msg.z[0]);
            grab_msg = Pose()
            grab_msg.position.x = msg.x[0]
            grab_msg.position.y = msg.y[0]
            grab_msg.position.z = msg.z[0]
            global grab_pub
            grab_pub.publish(grab_msg)
            grabbing = True
            # 已经获取物品坐标，停止检测
            behavior_msg = String()
            behavior_msg.data = "object_detect stop"
            behaviors_pub.publish(behavior_msg)

# 抓取执行结果回调函数
def cbGrabResult(msg):
    rospy.logwarn("抓取执行结果 = %s",msg.data)

# 主函数
if __name__ == "__main__":
    rospy.init_node("grab_node")
    # 发布物品检测激活话题
    behaviors_pub = rospy.Publisher("/wpb_home/behaviors", String, queue_size=10)
    # 发布抓取行为激活话题
    grab_pub = rospy.Publisher("/wpb_home/grab_action", Pose, queue_size=10)
    # 订阅物品检测结果的话题
    object_sub = rospy.Subscriber("/wpb_home/objects_3d", Coord, cbObject, queue_size=10)
    # 订阅抓取执行结果的话题
    result_sub = rospy.Subscriber("/wpb_home/grab_result", String, cbGrabResult, queue_size=10)
 
    # 延时三秒，让后台的话题初始化操作能够完成
    rospy.sleep(3)

    # 发送消息，激活物品检测行为
    msg = String()
    msg.data = "object_detect start"
    behaviors_pub.publish(msg)

    rospy.spin()