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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace cv;

static std::string rgb_topic;
static std::string pc_topic;
static std::string face_cascade_name;
static CascadeClassifier face_cascade;

static const std::string WIN_TITLE = "face";
static Mat frame_gray;
static ros::Publisher image_pub;
static std::vector<Rect> faces;
static std::vector<cv::Rect>::const_iterator face_iter;

static ros::Publisher pc_pub;
static tf::TransformListener *tf_listener; 
static ros::Publisher marker_pub;
static visualization_msgs::Marker line_face;
static visualization_msgs::Marker pos_face;
static visualization_msgs::Marker text_marker;

cv::Mat drawFacesRGB(cv::Mat inImage) 
{
    std::vector<cv::Rect>::const_iterator i;
    for (face_iter = faces.begin(); face_iter != faces.end(); ++face_iter) 
    {
        cv::rectangle(
            inImage,
            cv::Point(face_iter->x , face_iter->y),
            cv::Point(face_iter->x + face_iter->width, face_iter->y + face_iter->height),
            CV_RGB(255, 0 , 255),
            10);
    }
    return inImage;
}

void DrawTextMarker(std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inID;
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

void callbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("callbackRGB");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // change contrast: 0.5 = half  ; 2.0 = double
    cv_ptr->image.convertTo(frame_gray, -1, 1.5, 0);

    // create B&W image
    cvtColor( frame_gray, frame_gray, CV_BGR2GRAY );

	equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 9, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

    //ROS_INFO("face = %d",faces.size());
    if(faces.size() > 0)
    {
        cv_ptr->image = drawFacesRGB(cv_ptr->image);
    }

    image_pub.publish(cv_ptr->toImageMsg());
}

void callbackPointCloud(const sensor_msgs::PointCloud2 &input)
{
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
    //ROS_WARN("cloud_src size = %d  width = %d",cloud_src.size(),input.width); 

    ////////////////////////////////
    // Draw Face Boxes
    line_face.points.clear();
    line_face.header.frame_id = "base_footprint";
    line_face.ns = "line_face";
    line_face.action = visualization_msgs::Marker::ADD;
    line_face.id = 1;
    line_face.type = visualization_msgs::Marker::LINE_LIST;
    line_face.scale.x = 0.01;
    line_face.color.r = 1.0;
    line_face.color.g = 0;
    line_face.color.b = 1.0;
    line_face.color.a = 1.0;

    pos_face.points.clear();
    pos_face.header.frame_id = "base_footprint";
    pos_face.ns = "pos_face";
    pos_face.action = visualization_msgs::Marker::ADD;
    pos_face.id = 1;
    pos_face.type = visualization_msgs::Marker::CUBE_LIST;
    pos_face.scale.x = 0.5;
    pos_face.scale.y = 0.5;
    pos_face.scale.z = 0.001;
    pos_face.color.r = 1.0;
    pos_face.color.g = 0;
    pos_face.color.b = 1.0;
    pos_face.color.a = 1.0;

    geometry_msgs::Point p;
    int nFaceIndex = 1;
    std::vector<cv::Rect>::const_iterator i;
    for (face_iter = faces.begin(); face_iter != faces.end(); ++face_iter) 
    {
        int rgb_face_x = face_iter->x  + face_iter->width/2;
        int rgb_face_y = face_iter->y + face_iter->height/2;
        int index_pc = rgb_face_y*input.width + rgb_face_x;

        if(index_pc >= cloud_src.points.size())
            continue;

        float face_x = cloud_src.points[index_pc].x;
        float face_y = cloud_src.points[index_pc].y;
        float face_z = cloud_src.points[index_pc].z;

       if(isnanf(face_x) || isnanf(face_y) || isnanf(face_z))
            continue;
            
        p.x = 0.2; p.y = 0; p.z = 1.37; line_face.points.push_back(p);
        //p.x = -0.1; p.y = 0; p.z = 1.25; line_face.points.push_back(p);
        p.x = face_x; p.y = face_y; p.z = face_z; line_face.points.push_back(p);
        p.z = 0;pos_face.points.push_back(p);

        std::ostringstream stringStream;
        stringStream << "Face_" << nFaceIndex;
        std::string face_id = stringStream.str();
        DrawTextMarker(face_id,nFaceIndex,0.1,face_x,face_y,face_z+0.2,0,1.0,0);
        nFaceIndex ++;

        ROS_WARN("face (%d,%d) - (%.2f %.2f %.2f)",rgb_face_x,rgb_face_y,face_x,face_y,face_z); 
    }
    marker_pub.publish(line_face);
    marker_pub.publish(pos_face);

    for(int y=0; y< 300; y++)
    {
        for(int x=0; x< 200; x++)
        {
            int index_pc = y*input.width + x;
            cloud_src.points[index_pc].r = 1.0f;
            cloud_src.points[index_pc].g = 0.0f;
            cloud_src.points[index_pc].b = 1.0f;
        }
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_src, output);

    output.header.frame_id = pc_footprint.header.frame_id;
    pc_pub.publish(output);
}

int main(int argc, char **argv)
{
    cv::namedWindow(WIN_TITLE);
    ros::init(argc, argv, "wpb_home_face_detect");
    ros::NodeHandle nh_param("~");
    //nh_param.param<std::string>("rgb_topic", rgb_topic, "/camera/image_raw");
    nh_param.param<std::string>("rgb_topic", rgb_topic, "/kinect2/hd/image_color");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/hd/points");
    nh_param.param<std::string>("face_cascade_name", face_cascade_name, "haarcascade_frontalface_alt.xml");

    ROS_INFO("wpb_home_face_detect");

    bool res = face_cascade.load(face_cascade_name);
	if (res == false)
	{
		ROS_ERROR("fail to load haarcascade_frontalface_alt.xml");
        return 0;
	}

    tf_listener = new tf::TransformListener(); 
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe(rgb_topic, 1 , callbackRGB);
    ros::Subscriber pc_sub = nh.subscribe(pc_topic, 1 , callbackPointCloud);

    image_pub = nh.advertise<sensor_msgs::Image>("/face/image", 2);
    marker_pub = nh.advertise<visualization_msgs::Marker>("face_marker", 2);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("face_pointcloud",1);

    ros::Rate loop_rate(30);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyWindow(WIN_TITLE);
    delete tf_listener; 

    return 0;
}