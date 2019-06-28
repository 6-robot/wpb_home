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
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <wpb_home_behaviors/Coord.h>

static ros::Publisher grab_pub;
static geometry_msgs::Pose grab_msg;
static bool bGrabbing = false;

void ObjCoordCB(const wpb_home_behaviors::Coord::ConstPtr &msg)
{
    if(bGrabbing == false)
    {
        int nNumObj = msg->name.size();
        ROS_WARN("[ObjCoordCB] obj = %d",nNumObj);
        if(nNumObj > 0)
        {
            ROS_WARN("[ObjCoordCB] Grab %s! (%.2f , %.2f , %.2f)",msg->name[0].c_str(),msg->x[0],msg->y[0],msg->z[0]);
            grab_msg.position.x = msg->x[0];
            grab_msg.position.y = msg->y[0];
            grab_msg.position.z = msg->z[0];
            grab_pub.publish(grab_msg);
            bGrabbing = true;
        }
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_grab_object");  //程序初始化
    ROS_WARN("wpb_home_grab_object start!");

    ros::NodeHandle n;
    grab_pub = n.advertise<geometry_msgs::Pose>("/wpb_home/grab_action", 1);
    ros::Subscriber obj_sub = n.subscribe("/wpb_home/objects_3d", 1, ObjCoordCB);
    ros::Subscriber res_sub = n.subscribe("/wpb_home/grab_result", 30, GrabResultCB);

    // grab_msg.position.x = 0.9;
    // grab_msg.position.y = -0.1;
    // grab_msg.position.z = 0.8;
    // grab_pub.publish(grab_msg);

    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}