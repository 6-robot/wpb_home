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

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

static bool bFirstFrame = true;
static float fOriginYaw = 0.0f;
static ros::Publisher vel_pub;
static geometry_msgs::Twist vel_cmd;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    float fCurYaw = tf::getYaw(q)*180/3.1415;

    if(bFirstFrame == true)
    {
        fOriginYaw = fCurYaw;
        bFirstFrame = false;
    }
    else
    {
        float dYaw = fCurYaw - fOriginYaw;
        while (dYaw > 360)
        {
            dYaw -= 360;
        }
        while (dYaw < 0)
        {
            dYaw += 360;
        }

        if(fabs(dYaw - 180) < 5)
        {
            vel_cmd.angular.z = 0;
            vel_pub.publish(vel_cmd);
        }
        else
        {
            if(dYaw > 180)
            {
                vel_cmd.angular.z = -0.3;
                vel_pub.publish(vel_cmd);
            }
            else
            {
                vel_cmd.angular.z = 0.3;
                vel_pub.publish(vel_cmd);
            }
        }

        printf("[wpb_home_imu_turn] CurYaw= %.2f  dYaw = %.2f\n",fCurYaw, dYaw);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "wpb_home_imu_turn"); 
    ROS_INFO("[wpb_home_imu_turn]");

    ros::NodeHandle n;
    ros::Subscriber imu_sub = n.subscribe("imu/data_raw", 100, imuCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    vel_pub.publish(vel_cmd);
    ros::spin();

    return 0;
}