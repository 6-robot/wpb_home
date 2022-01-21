#!/usr/bin/env python
# coding=utf-8

import rospy
from wpb_home_behaviors.msg import Coord
from std_msgs.msg import String

# 物品检测回调函数
def cbObject(msg):
    print("------------------------------------------")
    rospy.loginfo("检测物品个数 = %d",len(msg.name))
    for i in range(len(msg.name)):
        rospy.logwarn("%d号物品 %s 坐标为(%.2f, %.2f, %.2f) ",
        i, msg.name[i], msg.x[i], msg.y[i], msg.z[i] )

# 主函数
if __name__ == "__main__":
    rospy.init_node("object_detect")
    # 发布物品检测激活话题
    behaviors_pub = rospy.Publisher("/wpb_home/behaviors", String, queue_size=10)
    # 订阅物品检测结果的话题
    object_sub = rospy.Subscriber("/wpb_home/objects_3d", Coord, cbObject, queue_size=10)

    # 延时三秒，让后台的话题初始化操作能够完成
    rospy.sleep(3)

    # 发送消息，激活物品检测行为
    msg = String()
    msg.data = "object_detect start"
    behaviors_pub.publish(msg)
    
    rospy.spin()