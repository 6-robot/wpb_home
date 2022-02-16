#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String

# 语音识别回调函数
def cbKeyword(msg):
    rospy.logwarn("语音识别结果 = %s", msg.data)
    if msg.data == "Hello":
        spk_msg = String()
        spk_msg.data = "Hi"
        spk_pub.publish(spk_msg)

# 主函数
if __name__ == "__main__":
    rospy.init_node("sr_en_node")
    # 订阅语音识别结果的话题
    sr_sub = rospy.Subscriber("/xfyun/iat", String, cbKeyword, queue_size=10)
    # 发布语音说话内容的话题
    spk_pub = rospy.Publisher("/xfyun/tts", String, queue_size=10)

    rospy.spin()