#include <ros/ros.h>
#include <hanyang_gripper_operator/finger_vel_msg.h>
#include <std_msgs/Bool.h>
#include "hanyang_gripper_operator/grip_operator.h"
#include <dynamixel_workbench_msgs/DynamixelStateList.h>


void joint_trajectory_callback(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
    std::string asdf = msg->joint_names[1];
    ROS_INFO("Recived : %s", asdf.c_str());
}

void dynamixel_state_callback(const dynamixel_workbench_msgs::DynamixelStateListConstPtr &msg)
{
    int32_t pos = msg->dynamixel_state[1].present_position;
    ROS_INFO("Recived : %d", pos);
}

void push_state_callback(const std_msgs::BoolConstPtr &msg)
{
    bool state = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_initializer");
    ros::NodeHandle nh;
    ros::Subscriber dynamixel_info_sub_ = nh.subscribe<dynamixel_workbench_msgs::DynamixelStateList>("hanyang_gripper/dynamixel_state",1,&dynamixel_state_callback);
    ros::Subscriber orig_trajectory_sub_ = nh.subscribe<trajectory_msgs::JointTrajectory>("hanyang_gripper/joint_trajectory",1,&joint_trajectory_callback);
    ros::Subscriber push_state_sub_ = nh.subscribe<std_msgs::Bool>("pushed",1,&push_state_callback)

    ros::spin();
}