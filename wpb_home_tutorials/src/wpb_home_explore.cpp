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
#include "wpb_home_tutorials/Explore.h"
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

std::vector<int> have_rubbish = {-1, -1, -1, -1, -1};

// #define ROOM_SIZE 5
static int ROOM_SIZE;

static int room_index = 0;
static ros::Publisher vel_pub;

static vector<string> arWaypoint;

string strGoto;
bool explore_start(wpb_home_tutorials::Explore::Request &req, wpb_home_tutorials::Explore::Response &res)
{
    ROS_INFO("Start Exploration");
    while (true)
    {
        // if (have_rubbish[room_index] == 0)
        // {
        //     ROS_WARN("Skip %d room.", room_index);
        //     continue;
        // }
        move_base_msgs::MoveBaseGoal goal;

        MoveBaseClient ac("move_base", true);
        if (!ac.waitForServer(ros::Duration(5.0)))
            ROS_INFO("The move_base action server is no running. action abort...");
        else
        {
            srvI.request.index = room_index;

            if (cliGetWPIndex.call(srvI))
            {
                std::string name = srvI.response.name;
                float x = srvI.response.pose.position.x;
                float y = srvI.response.pose.position.y;
                ROS_INFO("Go to %d room.", room_index);
            }
            else
            {
                ROS_ERROR("Failed to call service get_wp_index");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = srvI.response.pose;
            ac.sendGoal(goal);
            ac.waitForResult();
            if (!(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
            // if (!true)
            {
                ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                continue;
            }
            else
            {
                ROS_INFO("Arrived and Exploration in the %d room.", room_index);
                ros::Time begin = ros::Time::now();
                // after entering, start timer
                while ((ros::Time::now() - begin).toSec() < 3)
                {
                    ros::Duration(1).sleep();
                    ros::param::get("/rubbish_topic", have_rubbish);
                    if (have_rubbish[room_index] > 0)
                    {
                        ROS_INFO("Detecting rubbish in %d room.", room_index);
                        res.result = true;
                        return true;
                    }
                }
                ros::param::set("/rubbish_topic", have_rubbish);
                ROS_INFO("Not Found in the %d room, goint to the next one...", room_index);
                have_rubbish[room_index] = 0;
            }
        }
        room_index = (room_index + 1) % ROOM_SIZE;
    }
    // if (have_rubbish[ROOM_SIZE] == 0)
    //     res.result == false;

    return true;
}

bool explore_stop(wpb_home_tutorials::Explore::Request &req, wpb_home_tutorials::Explore::Response &res)
{
    ROS_WARN("[explore stop]");
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    vel_pub.publish(vel_cmd);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_explore");
    // action_manager.Init();
    ros::NodeHandle n;

    // cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    cliGetNum = n.serviceClient<waterplus_map_tools::GetNumOfWaypoints>("/waterplus/get_num_waypoint");
    cliGetWPIndex = n.serviceClient<waterplus_map_tools::GetWaypointByIndex>("/waterplus/get_waypoint_index");
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    waterplus_map_tools::GetNumOfWaypoints srvNum;
    if (cliGetNum.call(srvNum))
    {
        ROS_INFO("ROOM_SIZE = %d", (int)srvNum.response.num);
        ROOM_SIZE = (int)srvNum.response.num;
    }
    else
    {
        ROS_ERROR("Failed to call service get_num_waypoints");
    }
    n.setParam("rubbish_topic", have_rubbish);

    ros::Rate r(10);
    // ros::Subscriber sub_rubbish = n.subscribe("/rubbish_topic", 1, find_rubbish_callback);

    ros::ServiceServer start_svr = n.advertiseService("wpb_home_explore/start", explore_start);
    ros::ServiceServer stop_svr = n.advertiseService("wpb_home_explore/stop", explore_stop);

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();

    return 0;
}