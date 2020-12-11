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
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

static tf::TransformListener *tf_listener; 

void PointcloudCB(const sensor_msgs::PointCloud2 &input)
{
    // 将点云数值从相机坐标系转换到机器人坐标系
    bool result = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(1.0)); 
    if(result == false)
    {
        return;
    }
    sensor_msgs::PointCloud2 pc_footprint;
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //将点云数据从ROS格式转换到PCL格式
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
   
    // 截取z轴方向，高度0.3米到1.5米内的点云
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_src.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 1.5);
    pass.filter (cloud_src);

    // 定义模型分类器
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud_src.makeShared());
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.05);
    segmentation.setOptimizeCoefficients(true);

    // 使用模型分类器进行检测
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);

    // 统计平面点集的平均高度
    int point_num =  planeIndices->indices.size();
    float points_z_sum = 0;
    for(int i=0;i<point_num;i++)
    {
        int point_index = planeIndices->indices[i];
        points_z_sum += cloud_src.points[point_index].z;
    }
    float plane_height = points_z_sum/point_num;
    ROS_INFO("plane_height = %.2f" , plane_height);
 } 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_plane_detect");
    tf_listener = new tf::TransformListener(); 
    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe("/kinect2/sd/points", 10 , PointcloudCB);

    ros::spin();

    delete tf_listener; 

    return 0;

}