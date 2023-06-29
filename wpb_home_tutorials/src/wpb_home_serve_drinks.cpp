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
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <wpb_home_behaviors/Coord.h>
#include <sensor_msgs/JointState.h>

static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;
static ros::Publisher waypoint_pub;
static ros::Publisher grab_drink_pub;
static geometry_msgs::Pose grab_drink_msg;
static ros::Publisher mani_ctrl_pub;
static sensor_msgs::JointState mani_ctrl_msg;

#define STEP_READY                                      0
#define STEP_GOTO_KITCHEN                   1
#define STEP_DRINK_DETECT                    2
#define STEP_GRAB_DRINK                         3
#define STEP_GOTO_DINNING_ROOM    4
#define STEP_PUT_DOWN                            5
#define STEP_BACKWARD                             6
#define STEP_DONE                                         7
int nStep = STEP_READY;
static int nDeley = 0;

void DrinkCoordCB(const wpb_home_behaviors::Coord::ConstPtr &msg)
{
    if(nStep == STEP_DRINK_DETECT)
    {
        // 获取饮料检测结果
        int drink_num = msg->name.size();
        ROS_INFO("[DrinkCoordCB] drink_num = %d",drink_num);
        for(int i = 0;i<drink_num;i++)
        {
            ROS_INFO("[DrinkCoordCB]  %s  (%.2f , %.2f , %.2f)",msg->name[i].c_str(),msg->x[i],msg->y[i],msg->z[i]);
        }
        int grab_drink_index = 0;
        grab_drink_msg.position.x = msg->x[grab_drink_index];
        grab_drink_msg.position.y = msg->y[grab_drink_index];
        grab_drink_msg.position.z = msg->z[grab_drink_index];
        grab_drink_pub.publish(grab_drink_msg);

        std_msgs::String behavior_msg;
        behavior_msg.data = "object_detect stop";
        behaviors_pub.publish(behavior_msg);
        nStep = STEP_GRAB_DRINK;
    }
}

void NaviResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(nStep == STEP_GOTO_KITCHEN)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[NaviResultCB] Waypoint kitchen!");
            behavior_msg.data = "object_detect start";
            behaviors_pub.publish(behavior_msg);
            nStep = STEP_DRINK_DETECT;
        }
    }
     
    if(nStep == STEP_GOTO_DINNING_ROOM)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[NaviResultCB] Waypoint dinning room!");
            mani_ctrl_msg.name[0] = "gripper";
            mani_ctrl_msg.position[0] = 0.15;   //张开手爪
            mani_ctrl_pub.publish(mani_ctrl_msg);
            nDeley = 0;
            nStep = STEP_PUT_DOWN;
        }
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    if(nStep == STEP_GRAB_DRINK)
    {
        if(msg->data == "done")
        {
            ROS_INFO("[GrabResultCB] grab_drink done!");
            std_msgs::String waypoint_msg;
            waypoint_msg.data = "dinning room";
            waypoint_pub.publish(waypoint_msg);
            nStep = STEP_GOTO_DINNING_ROOM;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_drink_serve");

    ros::NodeHandle n;
    behaviors_pub = n.advertise<std_msgs::String>("/wpb_home/behaviors", 10);
    ros::Subscriber drink_result_sub = n.subscribe("/wpb_home/objects_3d", 10 , DrinkCoordCB);
    waypoint_pub = n.advertise<std_msgs::String>("/waterplus/navi_waypoint", 10);
    ros::Subscriber navi_res_sub = n.subscribe("/waterplus/navi_result", 10, NaviResultCB);
    grab_drink_pub = n.advertise<geometry_msgs::Pose>("/wpb_home/grab_action", 1);
    ros::Subscriber grab_res_sub = n.subscribe("/wpb_home/grab_result", 10, GrabResultCB);
    mani_ctrl_pub = n.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 10);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    mani_ctrl_msg.name.resize(1);
    mani_ctrl_msg.position.resize(1);
    mani_ctrl_msg.velocity.resize(1);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.position[0] = 0;
    
    sleep(1);

    ros::Rate r(10);
    while(ros::ok())
    {
        if(nStep == STEP_READY)
        {
            std_msgs::String waypoint_msg;
            waypoint_msg.data = "kitchen";
            waypoint_pub.publish(waypoint_msg);
            nStep = STEP_GOTO_KITCHEN;
        }

        if(nStep == STEP_PUT_DOWN)
        {
            nDeley ++;
            if(nDeley > 5*10)
            {
                nDeley = 0;
                nStep = STEP_BACKWARD;
            }
        }

        if(nStep == STEP_BACKWARD)
        {
            geometry_msgs::Twist vel_cmd;
            vel_cmd.linear.x = -0.1;
            vel_pub.publish(vel_cmd);
            nDeley ++;
            if(nDeley > 5 *10)
            {
                mani_ctrl_msg.name[0] = "lift";
                mani_ctrl_msg.position[0] = 0;            //收回手臂
                mani_ctrl_pub.publish(mani_ctrl_msg);

                vel_cmd.linear.x = 0;                               //停止移动
                vel_pub.publish(vel_cmd);
                nStep = STEP_DONE;
            }
        }

        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
