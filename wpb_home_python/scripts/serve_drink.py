#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from wpb_home_behaviors.msg import Coord
from sensor_msgs.msg import JointState
from enum import Enum

class State(Enum):
    ready               = 0
    goto_kitchen        = 1
    drink_detect        = 2
    grab_drink          = 3
    goto_dinning_room   = 4
    put_down            = 5
    backward            = 6
    done                = 7

step = State.ready
deley = 0

# 标记变量，是否处于抓取过程当中
grabbing = False

# 物品检测回调函数
def cbObject(msg):
    global step
    if(step == State.drink_detect):
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

            # 已经获取物品坐标，停止检测
            behavior_msg = String()
            behavior_msg.data = "object_detect stop"
            global behaviors_pub
            behaviors_pub.publish(behavior_msg)

            step = State.grab_drink

# 导航结果回调函数
def resultNavi(msg):
    global step
    if(step == State.goto_kitchen):
        rospy.loginfo("到达目标航点 kitchen")
        behavior_msg = String()
        behavior_msg.data = "object_detect start"
        global behaviors_pub
        behaviors_pub.publish(behavior_msg)
        step = State.drink_detect
    global mani_ctrl_msg
    if(step == State.goto_dinning_room):
        rospy.loginfo("到达目标航点 dinning room")
        mani_ctrl_msg.position[1] = 0.15
        global mani_ctrl_pub
        mani_ctrl_pub.publish(mani_ctrl_msg)
        step = State.put_down

# 抓取执行结果回调函数
def cbGrabResult(msg):
    global step
    if(step == State.grab_drink):
        if(msg.data == "done"):
            rospy.loginfo("抓取行为结束")
            waypoint_msg = String()
            waypoint_msg.data = "dinning room"
            global waypoint_pub
            waypoint_pub.publish(waypoint_msg)
            step = State.goto_dinning_room

# 主函数
if __name__ == "__main__":
    rospy.init_node("serve_drink")
    # 发布物品检测激活话题
    behaviors_pub = rospy.Publisher("/wpb_home/behaviors", String, queue_size=10)
    # 订阅物品检测结果的话题
    object_sub = rospy.Subscriber("/wpb_home/objects_3d", Coord, cbObject, queue_size=10)
    # 发布导航目标名称
    waypoint_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10)
    # 订阅导航执行结果的话题
    navi_result_sub = rospy.Subscriber("/waterplus/navi_result", String, resultNavi, queue_size=10)
    # 发布抓取行为激活话题
    grab_pub = rospy.Publisher("/wpb_home/grab_action", Pose, queue_size=10)
    # 订阅抓取执行结果的话题
    grab_result_sub = rospy.Subscriber("/wpb_home/grab_result", String, cbGrabResult, queue_size=10)
    # 发布手臂控制指令
    mani_ctrl_pub = rospy.Publisher("/wpb_home/mani_ctrl", JointState, queue_size=10)
    # 发布速度控制指令
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    mani_ctrl_msg = JointState()
    mani_ctrl_msg.name = ['lift', 'gripper']
    mani_ctrl_msg.position = [ 0 , 0 ]
    mani_ctrl_msg.velocity = [ 0 , 0 ]

    # 延时一秒，让后台的话题初始化操作能够完成
    rospy.sleep(1.0)

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        if(step == State.ready):
            waypoint_msg = String()
            waypoint_msg.data = "kitchen"
            waypoint_pub.publish(waypoint_msg)
            step = State.goto_kitchen

        if(step == State.put_down):
            deley += 1
            if(deley > 5*10):
                deley = 0
                step = State.backward

        if(step == State.backward):
            vel_cmd = Twist()
            vel_cmd.linear.x = -0.1
            vel_pub.publish(vel_cmd)
            deley += 1
            if(deley > 5*10):
                mani_ctrl_msg.name[0] = "lift"
                mani_ctrl_msg.position[0] = 0
                mani_ctrl_pub.publish(mani_ctrl_msg)
                vel_cmd.linear.x = 0
                vel_pub.publish(vel_cmd)
                step = State.done
        rate.sleep()