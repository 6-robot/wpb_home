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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include "kl_outlier.h"
#include "wpb_home_tutorials/Follow.h"

static std::string pc_topic;
static ros::Publisher vel_pub;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static tf::TransformListener *tf_listener; 
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_follow;
static visualization_msgs::Marker text_marker;
static bool bActive = false;
static float ranges[360];
static float keep_dist = 1.0;   //跟随距离
static float flw_x = keep_dist;
static float flw_y = 0;
static float new_flw_x = 0;
static float new_flw_y = 0;
static float max_linear_vel = 0.5;
static float max_angular_vel = 1.5;
static float raw[1920*1080*2];

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //ROS_WARN("ProcCloudCB");
    //tf to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0));  //return value always  false!
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);

    //process
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_follow;
    float minx = flw_x;
    float maxx = flw_x;
    float miny = flw_y;
    float maxy = flw_y;
    float minz = 0;
    float maxz = 1.8;
    int nPointCount = 0;
    for(size_t i=0; i< cloud_src.size(); i++)
    {
        if(cloud_src.points[i].z > minz && cloud_src.points[i].z < maxz)
        {
            double dist = sqrtf((cloud_src.points[i].x - flw_x)*(cloud_src.points[i].x - flw_x) + (cloud_src.points[i].y - flw_y)*(cloud_src.points[i].y - flw_y));
            if(dist < 0.3)
            {
                cloud_follow.points.push_back(cloud_src.points[i]);
                new_flw_x += cloud_src.points[i].x;
                new_flw_y += cloud_src.points[i].y;
                nPointCount ++;
                raw[2*i] = cloud_src.points[i].x; raw[2*i+1] = cloud_src.points[i].y;
                if(cloud_src.points[i].x < minx) { minx = cloud_src.points[i].x; };
                if(cloud_src.points[i].x > maxx) { maxx = cloud_src.points[i].x; };
                if(cloud_src.points[i].y < miny) { miny = cloud_src.points[i].y; };
                if(cloud_src.points[i].y > maxy) { maxy = cloud_src.points[i].y; };
            }
        }
    }
    if(nPointCount < 100) return;
    cloud_follow.width = nPointCount;
    cloud_follow.height = 1;
    new_flw_x /= nPointCount;
    new_flw_y /= nPointCount;
    
    //output
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_follow, output);
    output.header.frame_id = pc_footprint.header.frame_id;
    pc_pub.publish(output);

    RemoveBoxes();
    float box_height = 1.8;
    DrawBox(minx,maxx,miny,maxy,0,box_height,0,1,0);
    DrawPath(flw_x,flw_y,0);
    DrawText("Follow_Target",0.1,flw_x,flw_y,box_height+0.05,1,1,0);
}

void ScanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<360;i++)
    {
        ranges[i] = scan->ranges[i];
    }

    filter(flw_x,flw_y,ranges,raw,new_flw_x,new_flw_y);
    flw_x = new_flw_x;
    flw_y = new_flw_y;

    geometry_msgs::Twist vel_cmd;
    float flw_dist = sqrt(flw_x*flw_x + flw_y*flw_y);
    float diff_dist = flw_dist - keep_dist;
    float flw_linear = diff_dist * 0.3;
    if(fabs(flw_linear) > 0.05)
    {
        vel_cmd.linear.x = flw_linear;
        if( vel_cmd.linear.x > max_linear_vel ) vel_cmd.linear.x = max_linear_vel;
        if( vel_cmd.linear.x < -max_linear_vel ) vel_cmd.linear.x = -max_linear_vel;
        if( vel_cmd.linear.x < 0 ) vel_cmd.linear.x *= 0.3;
    }
    else
    {
        vel_cmd.linear.x = 0;
    }
    float d_angle = 0;
    float abs_x = fabs(new_flw_x);
    if(abs_x != 0) d_angle = atan(flw_y/abs_x);
    float flw_turn = d_angle * 1.5;
    if(fabs(flw_turn) > 0.1)
    {
        vel_cmd.angular.z = flw_turn;
        if( vel_cmd.angular.z > max_angular_vel ) vel_cmd.angular.z = max_angular_vel;
        if( vel_cmd.angular.z < -max_angular_vel ) vel_cmd.angular.z = -max_angular_vel;
    }
    else
    {
        vel_cmd.angular.z = 0;
    }
    if(bActive == true)
    {
        vel_pub.publish(vel_cmd);
    }
}

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 0;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);
    marker_pub.publish(line_box);
}

void DrawPath(float inX, float inY, float inZ)
{
    line_follow.header.frame_id = "base_footprint";
    line_follow.ns = "line_follow";
    line_follow.action = visualization_msgs::Marker::ADD;
    line_follow.id = 1;
    line_follow.type = visualization_msgs::Marker::LINE_LIST;
    line_follow.scale.x = 0.01;
    line_follow.color.r = 1.0;
    line_follow.color.g = 0;
    line_follow.color.b = 1.0;
    line_follow.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = 0; p.y = 0; p.z = 0; line_follow.points.push_back(p);
    p.x = inX; p.y = inY; p.z = inZ; line_follow.points.push_back(p);
    marker_pub.publish(line_follow);
}

void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_follow";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = 2;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void RemoveBoxes()
{
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
    line_follow.action = 3;
    line_follow.points.clear();
    marker_pub.publish(line_follow);
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}

bool follow_start(wpb_home_tutorials::Follow::Request  &req, wpb_home_tutorials::Follow::Response &res)
{
    float fThredhold = (float) req.thredhold;
    flw_x = fThredhold;
    flw_y = 0;
    keep_dist = flw_x;
    thredhold(flw_x);
    bActive = true;
    ROS_WARN("[follow_start] thredhold = %.2f", fThredhold);
    return true;
}

bool follow_stop(wpb_home_tutorials::Follow::Request  &req, wpb_home_tutorials::Follow::Response &res)
{
    float fThredhold = (float) req.thredhold;
    ROS_WARN("[follow_stop] thredhold = %.2f", fThredhold);
    bActive = false;
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

bool follow_resume(wpb_home_tutorials::Follow::Request  &req, wpb_home_tutorials::Follow::Response &res)
{
    float fThredhold = (float) req.thredhold;
    ROS_WARN("[follow_resume] thredhold = %.2f", fThredhold);
    bActive = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_follow");
    ROS_WARN("[wpb_home_follow]");
    flw_x = keep_dist;
    flw_y = 0;
    thredhold(flw_x);
    tf_listener = new tf::TransformListener(); 

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/sd/points");
    nh_param.param<bool>("start", bActive, false);

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 1 , ProcCloudCB);
    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1,ScanCB);

    ros::ServiceServer start_svr = nh.advertiseService("wpb_home_follow/start", follow_start);
    ros::ServiceServer stop_svr = nh.advertiseService("wpb_home_follow/stop", follow_stop);
    ros::ServiceServer resume_svr = nh.advertiseService("wpb_home_follow/resume", follow_resume);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("follow_pointcloud",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("follow_marker", 10);

    ros::spin();

    delete tf_listener; 

    return 0;
}