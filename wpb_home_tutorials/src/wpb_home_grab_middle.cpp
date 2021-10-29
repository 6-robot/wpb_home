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
#include <geometry_msgs/Pose.h>
#include <wpb_home_behaviors/Coord.h>

static ros::Publisher behaviors_pub;
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
            int nMidIndex = 0;
            float fMidDist = fabs(msg->y[0]);
            for(int i=1;i<nNumObj;i++)
            {
                if(fabs(msg->y[i]) < fMidDist)
                {
                    nMidIndex = i;
                    fMidDist = fabs(msg->y[i]);
                }
            }
            ROS_WARN("[ObjCoordCB]nMidIndex= %d",nMidIndex);
            ROS_WARN("[ObjCoordCB] Grab %s (%.2f , %.2f , %.2f)",
            msg->name[nMidIndex].c_str(),
            msg->x[nMidIndex],
            msg->y[nMidIndex],
            msg->z[nMidIndex]);
            grab_msg.position.x = msg->x[nMidIndex]-0.1;
            grab_msg.position.y = msg->y[nMidIndex]-0.06;
            grab_msg.position.z = msg->z[nMidIndex];
            grab_pub.publish(grab_msg);
            bGrabbing = true;

            std_msgs::String behavior_msg;
            behavior_msg.data = "object_detect stop";
            behaviors_pub.publish(behavior_msg);
        }
    }
}

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    //ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_grab_middle");
    ROS_WARN("wpb_home_grab_middle start!");

    ros::NodeHandle n;
    behaviors_pub = n.advertise<std_msgs::String>("/wpb_home/behaviors", 10);
    grab_pub = n.advertise<geometry_msgs::Pose>("/wpb_home/grab_action", 1);
    ros::Subscriber obj_sub = n.subscribe("/wpb_home/objects_3d", 1, ObjCoordCB);
    ros::Subscriber res_sub = n.subscribe("/wpb_home/grab_result", 30, GrabResultCB);

    sleep(2);

    std_msgs::String behavior_msg;
    behavior_msg.data = "object_detect start";
    behaviors_pub.publish(behavior_msg);

    ros::spin();

    return 0;
}
