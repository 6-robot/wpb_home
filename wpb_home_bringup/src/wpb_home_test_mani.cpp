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
#include "driver/WPB_Home_driver.h"

#define WPBH_TEST_MANI_ZERO 0
#define WPBH_TEST_MANI_DOWN 1
#define WPBH_TEST_MANI_UP   2
#define WPBH_TEST_MANI_FOLD 3

#define CMD_WAIT_SEC 10

static int nState = WPBH_TEST_MANI_ZERO;
static int nCount = 0;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_home_test_mani");
    ros::NodeHandle n;

    ROS_WARN("[TEST] Manipulator...");
    CWPB_Home_driver m_wpb_home;
    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ttyUSB0");
    m_wpb_home.Open(strSerialPort.c_str(),115200);
    
    ros::Rate r(1);

    while(ros::ok())
    {
        switch(nState)
        { 
        case WPBH_TEST_MANI_ZERO:   //收起状态,先变成初始状态
            ROS_INFO("[Mani] ZERO -> DOWN... %d",nCount);
            if (nCount == 0)
            {
                m_wpb_home.ManiPos( 11200,  1000, 50000, 80);
            }
            nCount ++;
            if(nCount >= CMD_WAIT_SEC)
            {
                nState = WPBH_TEST_MANI_DOWN;
                nCount = 0;
            }
            break;
        case WPBH_TEST_MANI_DOWN:
            ROS_INFO("[Mani] DOWN -> UP ... %d",nCount);
            if (nCount == 0)
            {
                m_wpb_home.ManiPos( 55000,  1000, 50000, 80);
            }
            nCount ++;
            if(nCount >= CMD_WAIT_SEC)
            {
                nState = WPBH_TEST_MANI_UP;
                nCount = 0;
            }
            break;
        case WPBH_TEST_MANI_UP:
             ROS_INFO("[Mani] UP -> DOWN ... %d",nCount);
            if (nCount == 0)
            {
                m_wpb_home.ManiPos( 11200,  1000, 0, 80);
            }
            nCount ++;
            if(nCount >= CMD_WAIT_SEC)
            {
                nState = WPBH_TEST_MANI_FOLD;
                nCount = 0;
            }
            break;
         case WPBH_TEST_MANI_FOLD:
             ROS_INFO("[Mani] DOWN -> ZERO ... %d",nCount);
            if (nCount == 0)
            {
                m_wpb_home.ManiPos( 0,  1000, 0, 80);
            }
            nCount ++;
            if(nCount >= CMD_WAIT_SEC)
            {
                nState = WPBH_TEST_MANI_ZERO;
                nCount = 0;
            }
            break;
        }
       
        r.sleep();
    }
}