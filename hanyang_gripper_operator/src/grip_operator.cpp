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

#include "hanyang_gripper_operator/grip_operator.h"

GripOperator::GripOperator()
  :node_handle_(""),
   priv_node_handle_("~"),
   is_loop_(false)
{
  std::string yaml_file = node_handle_.param<std::string>("trajectory_info", "");
  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
  rel_pos_msg_ = new hanyang_gripper_operator::dish_size_msg;

  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);
  bool result2 = get_rel_PosInfo(yaml_file, rel_pos_msg_);
  if (result||result2 == false)
  {
    ROS_ERROR("Please check YAML file");
    exit(0);
  }

  joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);
  rel_pos_pub_ = node_handle_.advertise<hanyang_gripper_operator::dish_size_msg>("reletive_pos", 100);
  
  move_command_server_ = node_handle_.advertiseService("execution", &GripOperator::moveCommandMsgCallback, this);
  rel_pos_command_server_ = node_handle_.advertiseService("rel_execution", &GripOperator::rel_moveCommandMsgCallback, this);

  is_loop_ = priv_node_handle_.param<bool>("is_loop", "false");
}

GripOperator::GripOperator(std::string tasks)
  :node_handle_(""),
   priv_node_handle_("~"),
   is_loop_(false)
{
  std::string yaml_file = node_handle_.param<std::string>(tasks, "");
  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
  rel_pos_msg_ = new hanyang_gripper_operator::dish_size_msg;

  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg_);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    exit(0);
  }

  joint_trajectory_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);
  // rel_pos_pub_ = node_handle_.advertise<hanyang_gripper_operator::dish_size_msg>("reletive_pos", 100);
  move_command_server_ = node_handle_.advertiseService(tasks, &GripOperator::moveCommandMsgCallback, this);
  // rel_pos_command_server_ = node_handle_.advertiseService("rel_execution", &GripOperator::rel_moveCommandMsgCallback, this);

  is_loop_ = priv_node_handle_.param<bool>("is_loop", "false");
}

GripOperator::~GripOperator()
{
}

bool GripOperator::getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
  YAML::Node file;
  file = YAML::LoadFile(yaml_file.c_str());

  if (file == NULL)
    return false;

  YAML::Node joint = file["joint"];
  uint8_t joint_size = joint["names"].size();

  for (uint8_t index = 0; index < joint_size; index++)
  {
    std::string joint_name = joint["names"][index].as<std::string>();
    jnt_tra_msg->joint_names.push_back(joint["names"][index].as<std::string>());
  }

  YAML::Node motion = file["motion"];
  uint8_t motion_size = motion["names"].size();

  for (uint8_t index = 0; index < motion_size; index++)
  {
    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

    std::string name = motion["names"][index].as<std::string>();
    YAML::Node motion_name = motion[name];
    for (uint8_t size = 0; size < joint_size; size++)
    {
      if (joint_size != motion_name["step"].size())
      {
        ROS_ERROR("Please check motion step size. It must be equal to joint size");
        return 0;
      }

      jnt_tra_point.positions.push_back(motion_name["step"][size].as<double>());

      ROS_INFO("motion_name : %s, step : %f", name.c_str(), motion_name["step"][size].as<double>());
    }

    if (motion_name["time_from_start"] == NULL)
    {
      ROS_ERROR("Please check time_from_start. It must be set time_from_start each step");
      return 0;
    }

    jnt_tra_point.time_from_start.fromSec(motion_name["time_from_start"].as<double>());

    ROS_INFO("time_from_start : %f", motion_name["time_from_start"].as<double>());

    jnt_tra_msg->points.push_back(jnt_tra_point);
  }

  return true;
}

bool GripOperator::get_rel_PosInfo(const std::string yaml_file, hanyang_gripper_operator::dish_size_msg *dish_size_msg)
{
   YAML::Node file;
  file = YAML::LoadFile(yaml_file.c_str());

  if (file == NULL)
    return false;

  YAML::Node joint = file["joint"];
  uint8_t joint_size = joint["names"].size();

  for (uint8_t index = 0; index < joint_size; index++)
  {
    std::string joint_name = joint["names"][index].as<std::string>();
    dish_size_msg->joint_names.push_back(joint["names"][index].as<std::string>());
  }

  YAML::Node motion = file["motion"];
  uint8_t motion_size = motion["names"].size();

  for (uint8_t index = 0; index < motion_size; index++)
  {
    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;
    hanyang_gripper_operator::dish_size_msg_points dish_size_rel;

    std::string name = motion["names"][index].as<std::string>();
    YAML::Node motion_name = motion[name];
    for (uint8_t size = 0; size < joint_size; size++)
    {
      if (joint_size != motion_name["step"].size())
      {
        ROS_ERROR("Please check motion step size. It must be equal to joint size");
        return 0;
      }

      jnt_tra_point.positions.push_back(motion_name["step"][size].as<double>());
      dish_size_rel.Finger_pos.push_back(motion_name["step"][size].as<double>());

      ROS_INFO("motion_name : %s, step : %f", name.c_str(), motion_name["step"][size].as<double>());
    }

    if (motion_name["time_from_start"] == NULL)
    {
      ROS_ERROR("Please check time_from_start. It must be set time_from_start each step");
      return 0;
    }

    jnt_tra_point.time_from_start.fromSec(motion_name["time_from_start"].as<double>());
    dish_size_rel.time_from_start.fromSec(motion_name["time_from_start"].as<double>());

    ROS_INFO("time_from_start : %f", motion_name["time_from_start"].as<double>());

    jnt_tra_msg_->points.push_back(jnt_tra_point);
    dish_size_msg->points.push_back(dish_size_rel);
  }

  return true;
}

bool GripOperator::moveCommandMsgCallback(std_srvs::Trigger::Request &req,
                                           std_srvs::Trigger::Response &res)
{
  joint_trajectory_pub_.publish(*jnt_tra_msg_);

  res.success = true;
  res.message = "Success to publish joint trajectory";

  return true;
}

bool GripOperator::rel_moveCommandMsgCallback(std_srvs::Trigger::Request  &req,
                                              std_srvs::Trigger::Response &res)
{
  rel_pos_pub_.publish(*rel_pos_msg_);

  res.success = true;
  res.message = "Success to publish joint trajectory";

  return true;
}                                           

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "grip_operator");
  GripOperator grip_operator;
  GripOperator fold_operator(std::string("finger_fold"));
  GripOperator unfold_operator(std::string("finger_unfold"));
  // GripOperator grip_operator(std::string("grip_to_size"));
  GripOperator initialize_operator(std::string("initialize_to_max"));

  ROS_INFO("For now, you can use publish joint trajectory msgs by triggering service(/execution)");

  if (grip_operator.isLoop())
  {
    while(1)
    {
      std_srvs::Trigger trig;
      grip_operator.moveCommandMsgCallback(trig.request, trig.response);
      fold_operator.moveCommandMsgCallback(trig.request, trig.response);
      unfold_operator.moveCommandMsgCallback(trig.request, trig.response);
      grip_operator.moveCommandMsgCallback(trig.request, trig.response);
      initialize_operator.moveCommandMsgCallback(trig.request, trig.response);
    }
  }

  ros::spin();

  return 0;
}
