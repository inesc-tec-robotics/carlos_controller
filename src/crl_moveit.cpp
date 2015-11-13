/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Sachin Chitta
*********************************************************************/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "crl_move_group");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  std::vector<double> points(6);
  points[0] = 0.0;
  points[1] = 0.0;
  points[2] = 0.0;
  points[3] = 0.0;
  points[4] = 0.0;
  points[5] = 0.0;
  
  move_group_interface::MoveGroup group("arm");
  robot_state::RobotStatePtr robot_sate = group.getCurrentState();
  std::vector<double> current_points = group.getCurrentJointValues();
  for(int i=0; i<current_points.size(); i++)
    ROS_INFO("Current position: %f", current_points[i]);
  //current_points[2] = 0.0;
  group.setJointValueTarget(points);
  group.move();
  
  geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
  ROS_INFO("Position x: %f", current_pose.pose.position.x);
  ROS_INFO("Position y: %f", current_pose.pose.position.y);
  ROS_INFO("Position z: %f", current_pose.pose.position.z);
  ROS_INFO("Orientation x: %f", current_pose.pose.orientation.x);
  ROS_INFO("Orientation y: %f", current_pose.pose.orientation.y);
  ROS_INFO("Orientation z: %f", current_pose.pose.orientation.z);
  ROS_INFO("Orientation w: %f", current_pose.pose.orientation.w);

  current_pose.pose.position.z -= 0.1;
  group.setPoseTarget(current_pose);
  group.move();
  current_pose = group.getCurrentPose();
  current_pose.pose.position.x += 0.1;
  group.setPoseTarget(current_pose);
  group.move();

  ros::waitForShutdown();
  
  return 0;
}

