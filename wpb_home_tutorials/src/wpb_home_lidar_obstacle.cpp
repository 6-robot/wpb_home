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
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// 速度控制主题发布对象
ros::Publisher vel_pub;

// 用来计算激光雷达检测点(x,y)坐标的变量
static double x_cos[360];
static double y_sin[360];
static float pnt_x[360];
static float pnt_y[360];

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 将所有测距值解算成(x,y)坐标值
    for(int i=0;i<360;i++)
	{
    	pnt_x[i] = scan->ranges[i] * x_cos[i];
		pnt_y[i] = scan->ranges[i] * y_sin[i];
	}

    // 对90度到270度的障碍点取正前方最近的点
    float min_x = 999.99;
    int min_index = 180;
    for(int i=90;i<270;i++)
    {
        // 在机器人中轴线左右0.4米范围内的障碍点才判断其正向距离值
        if(fabs(pnt_y[i])<0.4)  
        {
            if(pnt_x[i] < min_x)
            {
                // 记录正向距离值最小的障碍点角度
                min_x = pnt_x[i];
                min_index = i;
            }
        }
    }
    ROS_WARN("min_x= %.2f min_y = %.2f min_index= %d",min_x,pnt_y[min_index],min_index);

    // 对正向距离值最小的点进行判断，是否需要避障
    geometry_msgs::Twist vel_cmd;
    if(min_x < 1.0)
    {
        // 正向距离小于1.0米，需要避障
        vel_cmd.linear.x = 0;
        if(min_index < 180)
        {
            // 障碍物在右前方，向左平移避开
            vel_cmd.linear.y = 0.05;
        }
        else
        {
            // 障碍物在左前方，向右平移避开
            vel_cmd.linear.y = -0.05;
        }
    }
    else
    {
        // 正向距离大于1.0米，不需要避障
        vel_cmd.linear.x = 0.05;
        vel_cmd.linear.y = 0;
    }
     vel_pub.publish(vel_cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_home_lidar_obstacle");
    
    ROS_INFO("wpb_home_lidar_obstacle start!");

    // 初始化正余弦值，后面用来解算障碍点的（x，y）坐标
    double kStep = (M_PI * 2) / 360;
	for (int i = 0; i < 360; i++)
	{
		x_cos[i] = -1*cos(M_PI*0.0 - kStep*i);
		y_sin[i] = -1*sin(M_PI*0.0 - kStep*i);
	}

    // 订阅激光雷达主题，发布速度控制主题
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    ros::spin();
}
