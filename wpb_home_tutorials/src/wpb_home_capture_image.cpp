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
#include <unistd.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

static std::string rgb_topic;

static const std::string WIN_TITLE = "image";
static ros::Publisher image_pub;
static char filename[128];
static int nImageIndex = 1;
static bool bCaptrueOneFrame = false;
static char ImageFlag[] = "Drink";

void callbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("callbackRGB");
    if(bCaptrueOneFrame == true)
    {
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

        char *home_path = getenv("HOME");
        sprintf(filename,"%s/Capture_Images/%s_%04d.jpg",home_path,ImageFlag,nImageIndex);
        while (access(filename,R_OK) == 0)
        {
            nImageIndex++;
            sprintf(filename,"%s/Capture_Images/%s_%04d.jpg",home_path,ImageFlag,nImageIndex);
        }

        imwrite(filename,cv_ptr->image);
        ROS_WARN("captrue %s",filename);
        bCaptrueOneFrame = false;
    }
}

void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
    int btn_x = joy->buttons[2];
    //ROS_WARN("btn x = %d",btn_x);
    if(btn_x > 0)
    {
        bCaptrueOneFrame = true;
    }
}

int main(int argc, char **argv)
{
    cv::namedWindow(WIN_TITLE);
    ros::init(argc, argv, "wpb_home_capture_image");
    ros::NodeHandle nh_param("~");
    //nh_param.param<std::string>("rgb_topic", rgb_topic, "/camera/image_raw");
    nh_param.param<std::string>("rgb_topic", rgb_topic, "/kinect2/hd/image_color");

    ROS_WARN("wpb_home_capture_image");
    
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe(rgb_topic, 1 , callbackRGB);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10 , callbackJoy);

    ros::Rate loop_rate(1);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyWindow(WIN_TITLE);

    return 0;
}