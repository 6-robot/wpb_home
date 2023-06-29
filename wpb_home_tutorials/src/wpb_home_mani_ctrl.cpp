/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define WPBH_TEST_MANI_ZERO 0
#define WPBH_TEST_MANI_DOWN 1
#define WPBH_TEST_MANI_UP   2
#define WPBH_TEST_MANI_FOLD 3

#define CMD_WAIT_SEC 10

static int nState = WPBH_TEST_MANI_ZERO;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_mani_ctrl");

    ros::NodeHandle n;
    ros::Publisher mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
    sleep(2);

    sensor_msgs::JointState ctrl_msg;
    ctrl_msg.name.resize(2);
    ctrl_msg.position.resize(2);
    ctrl_msg.velocity.resize(2);
    ctrl_msg.name[0] = "lift";
    ctrl_msg.name[1] = "gripper";
    ctrl_msg.position[0] = 0;
    ctrl_msg.position[1] = 0;

    int nCount = 0;
    ros::Rate r(0.1);
    
    while(ros::ok())
    {
        //ROS_WARN("[wpb_home_mani_ctrl] nCount = %d", nCount);
        switch(nState)
        {
        case WPBH_TEST_MANI_ZERO:   //收起状态,先变成初始状态
            ROS_INFO("[wpb_home_mani_ctrl] ZERO -> DOWN");
            ctrl_msg.position[0] = 0.5;     //升降高度(单位:米)
            ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
            ctrl_msg.position[1] = 0.1;     //手爪指间距(单位:米)
            ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
            nState = WPBH_TEST_MANI_DOWN;
            break;
        case WPBH_TEST_MANI_DOWN:
            ROS_INFO("[wpb_home_mani_ctrl] DOWN -> UP ");
            ctrl_msg.position[0] = 1.0;     //升降高度(单位:米)
            ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
            ctrl_msg.position[1] = 0;       //手爪指间距(单位:米)
            ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
            nState = WPBH_TEST_MANI_UP;
            break;
        case WPBH_TEST_MANI_UP:
            ROS_INFO("[wpb_home_mani_ctrl] UP -> DOWN ");
            ctrl_msg.position[0] = 0.5;     //升降高度(单位:米)
            ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
            ctrl_msg.position[1] = 0.1;     //手爪指间距(单位:米)
            ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
            nState = WPBH_TEST_MANI_FOLD;
            break;
         case WPBH_TEST_MANI_FOLD:
            ROS_INFO("[wpb_home_mani_ctrl] DOWN -> ZERO ");
            ctrl_msg.position[0] = 0;       //升降高度(单位:米)
            ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
            ctrl_msg.position[1] = 0.1;     //手爪指间距(单位:米)
            ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)
            nState = WPBH_TEST_MANI_ZERO;
            break;
        }
        mani_ctrl_pub.publish(ctrl_msg);    //发送指令
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}