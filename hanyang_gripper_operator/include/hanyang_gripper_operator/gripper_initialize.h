#ifndef GRIPPER_INITIALIZE_H
#define GRIPPER_INITIALIZE_H


#include <ros/ros.h>
#include <hanyang_gripper_operator/finger_vel_msg.h>
#include <std_msgs/Bool.h>
#include "hanyang_gripper_operator/grip_operator.h"
#include <dynamixel_workbench_msgs/DynamixelStateList.h>

class GripperInit
{
    private:
        ros::NodeHandle node_handle_;
        ros::NodeHandle priv_node_handle_;
        bool init_state;
        bool first_pos_flag;
        int32_t curr_position1;
        int32_t curr_position2;
        int32_t initialized_pos;

        ros::Subscriber dynamixel_info_sub_;
        ros::Subscriber orig_trajectory_sub_;
        ros::Subscriber push_state_sub_;

        ros::Publisher mod_trajectory_pub_;
        ros::Publisher init_move_pub_;

    public:
        GripperInit();
        ~GripperInit();

        void initPublisher(void);
        void initSubscriber(void);
        bool push_state;
        

        void joint_trajectory_callback(const trajectory_msgs::JointTrajectoryConstPtr &msg);
        void dynamixel_state_callback(const dynamixel_workbench_msgs::DynamixelStateListConstPtr &msg);
        void push_state_callback(const std_msgs::BoolConstPtr &msg);

        bool move_finger_to_init(void);




};

#endif