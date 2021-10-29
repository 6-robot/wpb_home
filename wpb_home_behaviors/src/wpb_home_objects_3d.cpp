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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <wpb_home_behaviors/Coord.h>
#include <opencv2/highgui/highgui_c.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>


// 工作模式
#define MODE_IDLE                   0
#define MODE_OBJECT_DETECT 1
static int nMode = MODE_IDLE;

using namespace std;

static std::string pc_topic;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static ros::Publisher coord_pub;
static tf::TransformListener *tf_listener; 
void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
void SortObjects();
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_follow;
static visualization_msgs::Marker text_marker;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static ros::Publisher segmented_objects;
static ros::Publisher segmented_plane;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

static stBoxMarker boxMarker;

typedef struct stObjectDetected
{
    string name;
    float x;
    float y;
    float z;
    float probability;
}stObjectDetected;

static stObjectDetected tmpObj;
static vector<stObjectDetected> arObj;

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    //MODE_IDLE 不处理数据
    if(nMode == MODE_IDLE)
        return;

    //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    //ROS_INFO("cloud_src size = %d",cloud_src.size()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
    cloud_source_ptr = cloud_src.makeShared(); 

    pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象
    pass.setInputCloud (cloud_source_ptr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 1.5);
    pass.filter (*cloud_source_ptr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud_source_ptr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-1.5, 1.5);
    pass.filter (*cloud_source_ptr);

    //process
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Get the plane model, if present.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud_source_ptr);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.005);
    segmentation.setOptimizeCoefficients(true);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    segmentation.setAxis(axis);
    segmentation.setEpsAngle(  10.0f * (M_PI/180.0f) );
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);
    ROS_INFO_STREAM("Planes: " << planeIndices->indices.size());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    ////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    int i = 0, nr_points = (int) cloud_source_ptr->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_source_ptr->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        segmentation.setInputCloud (cloud_source_ptr);
        segmentation.segment (*planeIndices, *coefficients);
        if (planeIndices->indices.size () == 0)
        {
            ROS_WARN("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the planeIndices
        extract.setInputCloud (cloud_source_ptr);
        extract.setIndices (planeIndices);
        extract.setNegative (false);
        extract.filter (*plane);
        float plane_height = plane->points[0].z;
        ROS_INFO("%d - plana: %d points. height =%.2f" ,i, plane->width * plane->height,plane_height);
        if(plane_height > 0.6 && plane_height < 0.85) 
        {
            ROS_INFO("Final plane: %d points. height =%.2f" , plane->width * plane->height,plane_height);
            break;
        }

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_source_ptr.swap (cloud_f);
        i++;
    }
    

    if (planeIndices->indices.size() == 0)
        std::cout << "Could not find a plane in the scene." << std::endl;
    else
    {
        // Copy the points of the plane to a new cloud.
        extract.setInputCloud(cloud_source_ptr);
        extract.setIndices(planeIndices);
        extract.filter(*plane);

        // Retrieve the convex hull.
        pcl::ConvexHull<pcl::PointXYZRGB> hull;
        hull.setInputCloud(plane);
        // Make sure that the resulting hull is bidimensional.
        hull.setDimension(2);
        hull.reconstruct(*convexHull);

        // Redundant check.
        if (hull.getDimension() == 2)
        {
            // Prism object.
            pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
            prism.setInputCloud(cloud_source_ptr);
            prism.setInputPlanarHull(convexHull);
            prism.setHeightLimits(-0.20, -0.03); //height limit objects lying on the plane
            pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

            // Get and show all points retrieved by the hull.
            prism.segment(*objectIndices);
            extract.setIndices(objectIndices);
            extract.filter(*objects);
            segmented_objects.publish(objects);
            segmented_plane.publish(plane);

            // run clustering extraction on "objects" to get several pointclouds
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            std::vector<pcl::PointIndices> cluster_indices;
            ec.setClusterTolerance (0.05);
            ec.setMinClusterSize (500);
            ec.setMaxClusterSize (10000000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (objects);
            ec.extract (cluster_indices);

            pcl::ExtractIndices<pcl::PointXYZRGB> extract_object_indices;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB> > objectf;
            cv::Mat element;

            RemoveBoxes();
            arObj.clear();
            int nObjCnt = 0;
            for(int i = 0; i<cluster_indices.size(); ++i)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                extract_object_indices.setInputCloud(objects);
                extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[i]));
                extract_object_indices.filter(*object_cloud);
                objectf.push_back(*object_cloud);

                bool bFirstPoint = true;
                for (int j = 0; j < object_cloud->points.size(); j++) 
                {
                    pcl::PointXYZRGB p = object_cloud->points[j];
                    if(bFirstPoint == true)
                    {
                        boxMarker.xMax = boxMarker.xMin = p.x;
                        boxMarker.yMax = boxMarker.yMin = p.y;
                        boxMarker.zMax = boxMarker.zMin = p.z;
                        bFirstPoint = false;
                    }

                    if(p.x < boxMarker.xMin) { boxMarker.xMin = p.x;}
                    if(p.x > boxMarker.xMax) { boxMarker.xMax = p.x;}
                    if(p.y < boxMarker.yMin) { boxMarker.yMin = p.y;}
                    if(p.y > boxMarker.yMax) { boxMarker.yMax = p.y;}
                    if(p.z < boxMarker.zMin) { boxMarker.zMin = p.z;}
                    if(p.z > boxMarker.zMax) { boxMarker.zMax = p.z;}

                }
                if(boxMarker.xMin < 1.5 && boxMarker.yMin > -0.5 && boxMarker.yMax < 0.5)
                {
                    DrawBox(boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 0, 1, 0);

                    std::ostringstream stringStream;
                    stringStream << "obj_" << nObjCnt;
                    std::string obj_id = stringStream.str();
                    float object_x = boxMarker.xMax;
                    float object_y = (boxMarker.yMin+boxMarker.yMax)/2;
                    float object_z = boxMarker.zMin;
                    DrawText(obj_id,0.06, object_x,object_y,object_z, 1,0,1);
                    tmpObj.name = obj_id;
                    tmpObj.x = object_x;
                    tmpObj.y = object_y;
                    tmpObj.z = object_z;
                    tmpObj.probability = 1.0f;
                    arObj.push_back(tmpObj);

                    // coord.name.push_back(obj_id);
                    // coord.x.push_back(object_x);
                    // coord.y.push_back(object_y);
                    // coord.z.push_back(object_z);
                    // coord.probability.push_back(1.0f);
                    nObjCnt++;
                    ROS_WARN("[obj_%d] xMin= %.2f yMin = %.2f yMax = %.2f",i,boxMarker.xMin, boxMarker.yMin, boxMarker.yMax);
                } 
            }
            SortObjects();
            // coord_pub.publish(coord);
        }
        else std::cout << "The chosen hull is not planar." << std::endl;
    }
}

float CalObjDist(stObjectDetected* inObj)
{
    float x = inObj->x;
    float y = inObj->y;
    float z = inObj->z - 0.8f;
    float dist = sqrt(x*x + y*y + z*z);
    return dist;
}

void SortObjects()
{
    int nNum = arObj.size();
    if (nNum == 0)
        return;
    // 冒泡排序
    stObjectDetected tObj;
    for(int n = 0; n<nNum; n++)
    {
        float minObjDist = CalObjDist(&arObj[n]);
        for(int i=n+1;i<nNum; i++)
        {
            float curDist = CalObjDist(&arObj[i]);
            if(curDist < minObjDist)
            {
                // 交换位置
                tObj = arObj[n];
                arObj[n] = arObj[i];
                arObj[i] = tObj;
                minObjDist = curDist;
            }
        }
    }
    // 排序完毕，发送消息
    wpb_home_behaviors::Coord coord;
    for(int i=0;i<nNum; i++)
    {
        coord.name.push_back(arObj[i].name);
        coord.x.push_back(arObj[i].x);
        coord.y.push_back(arObj[i].y);
        coord.z.push_back(arObj[i].z);
        coord.probability.push_back(arObj[i].probability);
    }
    coord_pub.publish(coord);
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

static int nTextNum = 2;
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = nTextNum;
    nTextNum ++;
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

void BehaviorCB(const std_msgs::String::ConstPtr &msg)
{
    int nFindIndex = 0;
    nFindIndex = msg->data.find("object_detect start");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[object_detect start] ");
        nMode = MODE_OBJECT_DETECT;
    }

    nFindIndex = msg->data.find("object_detect stop");
    if( nFindIndex >= 0 )
    {
        ROS_WARN("[object_detect stop] ");
        RemoveBoxes();
        nMode = MODE_IDLE;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_objects_3d");
    ROS_INFO("wpb_home_objects_3d start!");
    tf_listener = new tf::TransformListener(); 

    ros::NodeHandle nh_param("~");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/qhd/points");
    bool start_flag = false;
    nh_param.param<bool>("start", start_flag, false);
    if(start_flag == true)
    {
        nMode = MODE_OBJECT_DETECT;
    }

    ros::NodeHandle nh;
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 10 , ProcCloudCB);
    ros::Subscriber beh_sub = nh.subscribe("/wpb_home/behaviors", 30, BehaviorCB);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("obj_pointcloud",1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("obj_marker", 10);
    coord_pub = nh.advertise<wpb_home_behaviors::Coord>("/wpb_home/objects_3d", 10);

    segmented_objects = nh.advertise<PointCloud> ("segmented_objects",1);
    segmented_plane = nh.advertise<PointCloud> ("segmented_plane",1);

    ros::spin();

    delete tf_listener; 

    return 0;

}