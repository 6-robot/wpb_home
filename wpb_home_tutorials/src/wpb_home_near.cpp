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
 @author     LinZhanhui
 ********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include "wpb_home_tutorials/Near.h"
// #include <wpb_home_apps/action_manager.h>
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <waterplus_map_tools/GetNumOfWaypoints.h>
#include <waterplus_map_tools/GetWaypointByIndex.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <geometry_msgs/Twist.h>
#include <wpb_home_behaviors/Coord.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
// static CActionManager action_manager;
static ros::ServiceClient cliGetWPIndex;
static ros::ServiceClient cliGetNum;
static waterplus_map_tools::GetWaypointByName srvName;
static waterplus_map_tools::GetWaypointByIndex srvI;
static float human_x = 0;
static float human_y = 0;
static float max_linear_vel = 0.5;
static float max_angular_vel = 1.5;

// #define ROOM_SIZE 5
static int ROOM_SIZE;
static bool bActive = false;
static int naerSuccess = 0;

static int room_index = 0;
static ros::Publisher vel_pub;

static vector<string> arWaypoint;

string strGoto;
bool near_start(wpb_home_tutorials::Near::Request &req, wpb_home_tutorials::Near::Response &res)
{
    naerSuccess = 0;
    float fThredhold = (float)req.thredhold;
    ROS_INFO("Near_start");
    bActive = true;
    while (naerSuccess == 0)
    {
    }
    if (naerSuccess == 0)
        ROS_ERROR("naerSuccess == 0");
    assert(naerSuccess != 0);
    res.result = naerSuccess;
    return true;
}

bool near_stop(wpb_home_tutorials::Near::Request &req, wpb_home_tutorials::Near::Response &res)
{
    ROS_WARN("[Near stop]");
    geometry_msgs::Twist vel_cmd;
    bActive = false;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    vel_pub.publish(vel_cmd);
    return true;
}

void GotoRubbish(const wpb_home_behaviors::Coord::ConstPtr &msg)
{
    if (bActive == true)
    {
        



    }
    naerSuccess = 1;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_near");
    // action_manager.Init();
    ros::NodeHandle n;

    ros::Subscriber pose_sub = n.subscribe<wpb_home_behaviors::Coord>("/Objects_detected", 1, GotoRubbish);

    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    waterplus_map_tools::GetNumOfWaypoints srvNum;

    ros::Rate r(10);
    // ros::Subscriber sub_rubbish = n.subscribe("/rubbish_topic", 1, find_rubbish_callback);

    ros::ServiceServer start_svr = n.advertiseService("wpb_home_near/start", near_start);
    ros::ServiceServer stop_svr = n.advertiseService("wpb_home_near/stop", near_stop);

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();

    return 0;
}