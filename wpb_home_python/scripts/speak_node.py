#!/usr/bin/env python
# coding=utf-8

import rospy
from sound_play.msg import SoundRequest

# 主函数
if __name__ == "__main__":
    rospy.init_node("speak_node")
    # 发布语音内容话题
    speak_pub = rospy.Publisher("/robotsound", SoundRequest, queue_size=20)

    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        # 发送消息，让语音节点发声
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.volume = 1.0
        msg.arg = "hello world"
        speak_pub.publish(msg)
        rate.sleep()