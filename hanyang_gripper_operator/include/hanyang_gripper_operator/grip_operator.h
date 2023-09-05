/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef GRIP_OPERATOR_H
#define GRIP_OPERATOR_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_srvs/Trigger.h>
#include "hanyang_gripper_operator/dish_size_msg.h"
#include "hanyang_gripper_operator/dish_size_msg_points.h"

class GripOperator
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_trajectory_pub_;
  ros::Publisher rel_pos_pub_;

  // ROS Topic Subscriber

  // ROS Service Server
  ros::ServiceServer move_command_server_;
  ros::ServiceServer rel_pos_command_server_;

  // ROS Service Client

  trajectory_msgs::JointTrajectory *jnt_tra_msg_;
  hanyang_gripper_operator::dish_size_msg *rel_pos_msg_;
  bool is_loop_;

 public:
  GripOperator();
  GripOperator(std::string tasks);
  ~GripOperator();

  bool isLoop(void){ return is_loop_;}

  bool getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg);
  bool get_rel_PosInfo(const std::string yaml_file, hanyang_gripper_operator::dish_size_msg *dish_size_msg);
  bool moveCommandMsgCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);
  bool rel_moveCommandMsgCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);
};

#endif // GRIP_OPERATOR_H
