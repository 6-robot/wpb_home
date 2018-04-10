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
#include <std_msgs/String.h>

#define STEP_WAIT        0
#define STEP_HAND_UP     1
#define STEP_GRIPPER     2
#define STEP_HAND_DOWN   3
#define STEP_DONE        4

static int nStep = STEP_WAIT;
static ros::Publisher result_pub;
static std_msgs::String result_msg;
static int nDelayCount = 0;

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("pass start");
    if( nFindIndex >= 0 )
    {
        nDelayCount = 0;
        nStep = STEP_HAND_UP;
        ROS_WARN("[pass_start] ");
    }

    nFindIndex = msg->data.find("pass stop");
    if( nFindIndex >= 0 )
    {
        nStep = STEP_WAIT;
        ROS_WARN("[pass_stop] ");
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_pass_server");

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/wpb_home/behaviors", 30, BehaviorCB);
    ros::Publisher mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("wpb_home/mani_ctrl", 30);
    result_pub = n.advertise<std_msgs::String>("/wpb_home/pass_result", 30);

     bool bActive = false;
    ros::NodeHandle nh_param("~");
    nh_param.param<bool>("start", bActive, false);
    if(bActive == true)
    {
        nDelayCount = 0;
        nStep = STEP_HAND_UP;
    }

    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    mani_ctrl_msg.velocity[0] = 0.5;     //升降速度(单位:米/秒)
    mani_ctrl_msg.position[1] = 0.16;
    mani_ctrl_msg.velocity[1] = 5;       //手爪开合角速度(单位:度/秒)

    ros::Rate r(10);
    ros::spinOnce();
    r.sleep();
    
    while(ros::ok())
    {
        switch(nStep)
        {
        case STEP_HAND_UP:
            if(nDelayCount == 0)
            {
                result_msg.data = "hand up";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_HAND_UP");
            }
            mani_ctrl_msg.position[0] = 1.0;
            mani_ctrl_msg.position[1] = -0.1;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nDelayCount ++;
            if(nDelayCount > 80)
            {
                nDelayCount = 0;
                nStep = STEP_GRIPPER;
                result_msg.data = "gripper";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_GRIPPER");
            }
            break;
        case STEP_GRIPPER:
            mani_ctrl_msg.position[1] = 0.16;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nDelayCount ++;
            if(nDelayCount > 50)
            {
                nDelayCount = 0;
                nStep = STEP_HAND_DOWN;
                result_msg.data = "hand down";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_HAND_DOWN");
            }
            break;
        case STEP_HAND_DOWN:
            mani_ctrl_msg.position[0] = 0;
            mani_ctrl_msg.position[1] = 0.16;
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nDelayCount ++;
            if(nDelayCount > 80)
            {
                nDelayCount = 0;
                nStep = STEP_DONE;
                result_msg.data = "done";
                result_pub.publish(result_msg);
                ROS_WARN("[wpb_home_pass_server] STEP_DONE");
            }
            break;
        case STEP_DONE:
            break;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}