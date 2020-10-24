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

#include <wpbh_local_planner/wpbh_local_planner.h>
#include <wpbh_local_planner/wl_helper.h>
#include <tf_conversions/tf_eigen.h>
#include <pluginlib/class_list_macros.h>
#include <wpbh_local_planner/CLidarAC.h>

// register this planner as a WpbhLocalPlanner plugin
//PLUGINLIB_DECLARE_CLASS(wpbh_local_planner, WpbhLocalPlanner, wpbh_local_planner::WpbhLocalPlanner, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS( wpbh_local_planner::WpbhLocalPlanner, nav_core::BaseLocalPlanner)

static CLidarAC lidar_ac;
static float ranges[360];

namespace wpbh_local_planner
{
    WpbhLocalPlanner::WpbhLocalPlanner()
    {
        //ROS_WARN("[WPBH]WpbhLocalPlanner() ");
        InitHelper();
        m_costmap_ros = NULL;
        m_tf_listener = NULL; 
        m_goal_reached = false;
        m_bInitialized = false;
        m_bAC = false;
        m_bFirstStep = true;
    }

    WpbhLocalPlanner::~WpbhLocalPlanner()
    {
        
    }

    void WpbhLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("WpbhLocalPlanner::initialize() ");
        if(!m_bInitialized)
        {	
            m_tf_listener = tf;
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            
            m_global_frame_id = m_costmap_ros->getGlobalFrameID();      //"odom"
            m_robot_base_frame_id = m_costmap_ros->getBaseFrameID();    //"base_footprint"
            
            m_footprint_spec = m_costmap_ros->getRobotFootprint();
            costmap_2d::calculateMinAndMaxDistances(m_footprint_spec, m_robot_inscribed_radius, m_robot_circumscribed_radius); 

            ros::NodeHandle nh_planner("~/" + name);
            nh_planner.param("max_vel_trans", m_max_vel_trans, 0.3);
            nh_planner.param("max_vel_rot", m_max_vel_rot, 0.9);
            nh_planner.param("acc_scale_trans", m_acc_scale_trans, 1.5);
            nh_planner.param("acc_scale_rot", m_acc_scale_rot, 0.5);
            nh_planner.param("goal_dist_tolerance", m_goal_dist_tolerance, 0.1);
            nh_planner.param("goal_yaw_tolerance", m_goal_yaw_tolerance, 0.1);
            nh_planner.param("scan_topic", m_scan_topic, std::string("/scan"));

            m_pub_target = nh_planner.advertise<geometry_msgs::PoseStamped>("local_planner_target", 10);
            m_scan_sub = nh_planner.subscribe<sensor_msgs::LaserScan>(m_scan_topic,1,&WpbhLocalPlanner::lidarCallback,this);
            
            m_bInitialized = true;

            ROS_DEBUG("wpbh_local_planner plugin initialized.");
        }
        else
        {
            ROS_WARN("wpbh_local_planner has already been initialized, doing nothing.");
        }
    }

    void WpbhLocalPlanner::initialize(std::string name,tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("WpbhLocalPlanner::initialize() ");
        if(!m_bInitialized)
        {	
            //m_tf_listener = tf;
            m_tf_listener = new tf::TransformListener;
            m_costmap_ros = costmap_ros;
            m_costmap = m_costmap_ros->getCostmap();
            
            m_global_frame_id = m_costmap_ros->getGlobalFrameID();      //"odom"
            m_robot_base_frame_id = m_costmap_ros->getBaseFrameID();    //"base_footprint"
            
            m_footprint_spec = m_costmap_ros->getRobotFootprint();
            costmap_2d::calculateMinAndMaxDistances(m_footprint_spec, m_robot_inscribed_radius, m_robot_circumscribed_radius); 

            ros::NodeHandle nh_planner("~/" + name);
            nh_planner.param("max_vel_trans", m_max_vel_trans, 0.3);
            nh_planner.param("max_vel_rot", m_max_vel_rot, 0.9);
            nh_planner.param("acc_scale_trans", m_acc_scale_trans, 1.5);
            nh_planner.param("acc_scale_rot", m_acc_scale_rot, 0.5);
            nh_planner.param("goal_dist_tolerance", m_goal_dist_tolerance, 0.1);
            nh_planner.param("goal_yaw_tolerance", m_goal_yaw_tolerance, 0.1);
            nh_planner.param("scan_topic", m_scan_topic, std::string("/scan"));

            m_pub_target = nh_planner.advertise<geometry_msgs::PoseStamped>("local_planner_target", 10);
            m_scan_sub = nh_planner.subscribe<sensor_msgs::LaserScan>(m_scan_topic,1,&WpbhLocalPlanner::lidarCallback,this);
            
            m_bInitialized = true;

            ROS_DEBUG("wpbh_local_planner plugin initialized.");
        }
        else
        {
            ROS_WARN("wpbh_local_planner has already been initialized, doing nothing.");
        }
    }

    static double fScaleD2R = 3.14159 / 180;
    void WpbhLocalPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        for(int i=0;i<360;i++)
        {
            ranges[i] = scan->ranges[i];
        }
    }

    bool WpbhLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if(!m_bInitialized)
        {
            ROS_ERROR("wpbh_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        for(int i=0;i<360;i++)
        {
            ranges[i] = 0;
        }

        m_global_plan.clear();
        m_global_plan = plan;
        m_nPathIndex = 0;

        m_goal_reached = false;
        m_nStep = WPBH_STEP_GOTO;
        m_bFirstStep = true;
        
        return true;
    }

    static double CalDirectAngle(double inFromX, double inFromY, double inToX, double inToY)
    {
        double res = 0;
        double dx = inFromX - inToX;
        double dy = -(inFromY - inToY);
        if (dx == 0)
        {
            if (dy > 0)
            {
                res = 180 - 90;
            }
            else
            {
                res = 0 - 90;
            }
        }
        else
        {
            double fTan = dy / dx;
            res = atan(fTan) * 180 / 3.1415926;

            if (dx < 0)
            {
                res = res - 180;
            }
        }
        res = 180 - res;
        if (res < 0)
        {
            res += 360;
        }
        if (res > 360)
        {
            res -= 360;
        }
        res = res*3.1415926/180;
        return res;
    }

    static double AngleFix(double inAngle, double inMin, double inMax)
    {
        if (inMax - inMin > 6.28)
        {
            return inAngle;
        }
        
        double retAngle = inAngle;
        while (retAngle < inMin)
        {
            retAngle += 6.28;
        }
        while (retAngle > inMax)
        {
            retAngle -= 6.28;
        }
        return retAngle;
    }

    bool WpbhLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!m_bInitialized)
        {
            ROS_ERROR("wpbh_local_planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        int path_num = m_global_plan.size();
        if(path_num == 0)
        {
            return false;
        }

        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        bool res = true;

        if(m_bFirstStep == true)
        {
            double target_x, target_y, target_th;
            while(m_nPathIndex < path_num-1)
            {
                getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, target_x, target_y, target_th);
                if(sqrt(target_x*target_x + target_y*target_y) < m_goal_dist_tolerance)
                {
                    m_nPathIndex ++;
                }
                else
                {
                    break;  //target is far enough
                }
            }

            double face_target = CalDirectAngle(0, 0, target_x, target_y);
            face_target = AngleFix(face_target,-2.1,2.1);
            if(fabs(face_target)> 0.09)
            {
                //turn in place
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = face_target * m_acc_scale_rot;
                if(cmd_vel.angular.z > 0) cmd_vel.angular.z +=0.2;
                if(cmd_vel.angular.z < 0) cmd_vel.angular.z -=0.2;
            }
            else
            {
                m_bFirstStep = false;
            }
        }
        
        if(m_nStep == WPBH_STEP_ARRIVED)
        {
            ROS_WARN("[WPBH_ARRIVED](%.2f %.2f):%.2f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
            return true;
        }

        double goal_x,goal_y,goal_th;
        getTransformedPosition(m_global_plan.back(), m_robot_base_frame_id, goal_x, goal_y, goal_th);
        
        if(m_nStep == WPBH_STEP_GOTO)
        {
            lidar_ac.SetRanges(ranges);
            // check if global goal is near
            double goal_dist = sqrt(goal_x*goal_x + goal_y*goal_y);
            if(goal_dist < m_goal_dist_tolerance)
            {
                m_nStep = WPBH_STEP_NEAR;
            }
            else
            {
                ClearObst();
                SetRanges(ranges);
                //check if target is near
                double minDist = 999;
                int nMinDistIndex = m_nPathIndex;
                double target_x, target_y, target_th;
                //while(m_nPathIndex < path_num-1)
                for(int i=m_nPathIndex;i<path_num;i++)
                {
                    getTransformedPosition(m_global_plan[i], m_robot_base_frame_id, target_x, target_y, target_th);
                    if( ChkTarget(target_y/0.05+50,target_x/0.05+50) == true)
                    {
                        double tmpDist = sqrt(target_x*target_x + target_y*target_y) < m_goal_dist_tolerance;
                        if(tmpDist < minDist)
                        {
                            nMinDistIndex = i;
                            minDist = tmpDist;
                        }
                    }
                }
                m_nPathIndex = nMinDistIndex;
                
                double gpath_x, gpath_y, gpath_th;
                ClearTarget();
                for(int i=m_nPathIndex;i<path_num;i++)
                {
                    getTransformedPosition(m_global_plan[i], m_robot_base_frame_id, gpath_x, gpath_y, gpath_th);
                    SetTarget(gpath_y/0.05+50,gpath_x/0.05+50);
                }
                res = OutLine();
                if(res == false)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = 0;
                    return true;
                }
                if(GetHelperNum() > 5 && (path_num - m_nPathIndex) > 1)
                {
                    target_x = GetFixX();
                    target_y = GetFixY();
                    gpath_x = GetFaceX();
                    gpath_y = GetFaceY();
                }
                else
                {
                    getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, target_x, target_y, target_th);
                    getTransformedPosition(m_global_plan[m_nPathIndex], m_robot_base_frame_id, gpath_x, gpath_y, gpath_th);
                }
                // 确定机器人朝向
                double face_target = CalDirectAngle(0, 0, gpath_x, gpath_y);
                face_target = AngleFix(face_target,-2.1,2.1);
                if(fabs(face_target)> 0.8)
                {
                    // turn in place
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = face_target * m_acc_scale_rot;
                    if(cmd_vel.angular.z > 0) cmd_vel.angular.z +=0.2;
                    if(cmd_vel.angular.z < 0) cmd_vel.angular.z -=0.2;
                }
                else
                {
                    // start to move
                    cmd_vel.linear.x = target_x * m_acc_scale_trans;
                    cmd_vel.linear.y = target_y * m_acc_scale_trans;
                    cmd_vel.angular.z = face_target * m_acc_scale_rot;
                }

                if(cmd_vel.linear.x > 0) cmd_vel.linear.x+=0.05;
                if(cmd_vel.linear.x < 0) cmd_vel.linear.x-=0.05;
                if(cmd_vel.linear.y > 0) cmd_vel.linear.y+=0.02;
                if(cmd_vel.linear.y < 0) cmd_vel.linear.y-=0.02;

                lidar_ac.OutLine();
                m_pub_target.publish(m_global_plan[m_nPathIndex]);
            }
        }

        if(m_nStep == WPBH_STEP_NEAR)
        {
            
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = goal_th;

            if(fabs(goal_th) < m_goal_yaw_tolerance)
            {
                m_goal_reached = true;
                m_nStep = WPBH_STEP_ARRIVED;
                cmd_vel.angular.z = 0;
                //ROS_WARN("[WPBH-ARRIVED] goal (%.2f,%.2f) %.2f",goal_x, goal_y, goal_th);
            }
            m_pub_target.publish(m_global_plan.back());
        }

        if(cmd_vel.linear.x > m_max_vel_trans) cmd_vel.linear.x = m_max_vel_trans;
        if(cmd_vel.linear.x < -m_max_vel_trans) cmd_vel.linear.x = -m_max_vel_trans;
        if(cmd_vel.linear.y > m_max_vel_trans) cmd_vel.linear.y = m_max_vel_trans;
        if(cmd_vel.linear.y < -m_max_vel_trans) cmd_vel.linear.y = -m_max_vel_trans;
        if(cmd_vel.angular.z > m_max_vel_rot) cmd_vel.angular.z = m_max_vel_rot;
        if(cmd_vel.angular.z < -m_max_vel_rot) cmd_vel.angular.z = -m_max_vel_rot;

        m_last_cmd = cmd_vel;
        
        return res;
    }


    bool WpbhLocalPlanner::isGoalReached()
    {
        if (m_goal_reached)
        {
            ROS_INFO("GOAL Reached!");
            return true;
        }
        return false;
    }

    void WpbhLocalPlanner::getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta)
    {
        geometry_msgs::PoseStamped tf_pose;
        pose.header.stamp = ros::Time(0);
        m_tf_listener->transformPose(frame_id, pose, tf_pose);
        x = tf_pose.pose.position.x;
        y = tf_pose.pose.position.y,
        theta = tf::getYaw(tf_pose.pose.orientation);
    }

}