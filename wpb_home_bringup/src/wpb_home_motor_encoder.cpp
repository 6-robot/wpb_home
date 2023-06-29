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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "driver/WPB_Home_driver.h"

static CWPB_Home_driver m_wpb_home;
static float vel_x = 0.3;
static float vel_y = 0;
static float vel_z = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("[wpb_home_cmd_vel] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    vel_x = msg->linear.x;
    vel_y = msg->linear.y;
    vel_z = msg->angular.z;
}

static int nFirstVal[3];
static bool bFirst = true;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_home_motor_encoder");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",1,&cmdVelCallback);

    m_wpb_home.Open("/dev/ftdi",115200);
    
    ros::Rate r(100.0);
    r.sleep();

    int display_count = 0;
    while(n.ok())
    {
        m_wpb_home.ReadNewData();

        ///////////////////
        display_count ++;
        if(display_count > 100)
        {
            display_count = 0;
            ROS_WARN(" [M1] %d   [M2] %d   [M3] %d", m_wpb_home.arMotorPos[0], m_wpb_home.arMotorPos[1], m_wpb_home.arMotorPos[2]);
            m_wpb_home.Velocity(vel_x,vel_y,vel_z);
        }
        ////////////////////
        if(bFirst == true)
        {
            nFirstVal[0] = m_wpb_home.arMotorPos[0];
            nFirstVal[1] = m_wpb_home.arMotorPos[1];
            nFirstVal[2] = m_wpb_home.arMotorPos[2];
            bFirst = false;
        }
        else
        {
            int nDiff[3];
            nDiff[0] = m_wpb_home.arMotorPos[0] - nFirstVal[0];
            nDiff[1] = m_wpb_home.arMotorPos[1] - nFirstVal[1];
            nDiff[2] = m_wpb_home.arMotorPos[2] - nFirstVal[2];
            // ROS_INFO("Diff [M1]%d [M2]%d [M3]%d", nDiff[0], nDiff[1], nDiff[2]);
        }
        /////////////////////
        
        ros::spinOnce();
        r.sleep();
    }
}