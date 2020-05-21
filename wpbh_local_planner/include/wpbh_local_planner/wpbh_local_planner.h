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

#ifndef WPBH_LOCAL_PLANNER_H_
#define WPBH_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>

#define WPBH_STEP_GOTO     1
#define WPBH_STEP_NEAR     2
#define WPBH_STEP_ARRIVED  3

namespace wpbh_local_planner
{
  class WpbhLocalPlanner : public nav_core::BaseLocalPlanner
  {
  public:
    WpbhLocalPlanner();
    ~WpbhLocalPlanner();

    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();

  protected:
    void getTransformedPosition(geometry_msgs::PoseStamped& pose, std::string& frame_id, double& x, double& y, double& theta);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    
  private:
    bool m_bInitialized;
    int m_nStep;
    int m_nPathIndex;
    bool m_bAC;
    bool m_bFirstStep;

    costmap_2d::Costmap2DROS* m_costmap_ros;
    costmap_2d::Costmap2D* m_costmap;
    tf::TransformListener* m_tf_listener;
    std::vector<geometry_msgs::PoseStamped> m_global_plan;
    std::string m_global_frame_id; 
    std::string m_robot_base_frame_id;
    bool m_goal_reached;
    
    geometry_msgs::Twist m_last_cmd;
    
    std::vector<geometry_msgs::Point> m_footprint_spec;
    double m_robot_inscribed_radius;
    double m_robot_circumscribed_radius;

    ros::Subscriber m_scan_sub;
    ros::Publisher m_pub_target;
    double m_max_vel_trans;
    double m_max_vel_rot;
    double m_acc_scale_trans;
    double m_acc_scale_rot;
    double m_goal_dist_tolerance;
    double m_goal_yaw_tolerance;
    std::string m_scan_topic;
  };
}; // end namespace wpbh_local_planner

#endif // WPBH_LOCAL_PLANNER_H_