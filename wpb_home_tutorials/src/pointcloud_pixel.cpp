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
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>



void PointcloudCB(const sensor_msgs::PointCloud2 &input)
{
    // // 将点云数值从相机坐标系转换到机器人坐标系
    // if(result == false)
    // {
    //     return;
    // }
    // sensor_msgs::PointCloud2 pc_footprint;
    // pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    // // 将点云数据从ROS格式转换到PCL格式
    // pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    // pcl::fromROSMsg(pc_footprint , cloud_src);
   
}

void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
    pcl::fromROSMsg(*msg , pointCloudIn);

    int cloudSize = pointCloudIn.points.size();
    for(int i=0;i<cloudSize;i++)
    {
        ROS_INFO("[i= %d] ( %.2f , %.2f , %.2f)", 
            i , 
            pointCloudIn.points[i].x, 
            pointCloudIn.points[i].y, 
            pointCloudIn.points[i].z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_pixel");
    ROS_WARN("pointcloud_pixel start");

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe("/kinect2/sd/points", 1 , callbackPC);

    ros::spin();

    return 0;
}