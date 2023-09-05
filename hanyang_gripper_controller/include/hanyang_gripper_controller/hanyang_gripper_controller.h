#ifndef HANYANG_GRIPPER_CONTROLLER_H
#define HANYANG_GRIPPER_CONTROLLER_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include "hanyang_gripper_controller/trajectory_generator.h"

#include "hanyang_gripper_operator/finger_vel_msg.h"
#include "hanyang_gripper_controller/FingerCommand.h"
#include "hanyang_gripper_controller/reletive_pos.h"

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0


typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class FingerController
{
    protected:
        // ROS NodeHandle
        ros::NodeHandle node_handle_;
        ros::NodeHandle priv_node_handle_;

        // ROS Parameters

        // ROS Topic Publisher
        ros::Publisher dynamixel_state_list_pub_;
        ros::Publisher joint_states_pub_;
        ros::Publisher reletive_position_pub_;

        // ROS Topic Subscriber
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber trajectory_sub_;
        ros::Subscriber contact_sub_;

        // ROS Service Server
        ros::ServiceServer dynamixel_command_server_;
        ros::ServiceServer finger_service_server_;

        // ROS Service Client

        // Dynamixel Workbench Parameters
        DynamixelWorkbench *dxl_wb_;

        std::map<std::string, uint32_t> dynamixel_;
        std::map<std::string, const ControlItem*> control_items_;
        std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
        dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
        sensor_msgs::JointState joint_state_msg_;
        std::vector<WayPoint> pre_goal_;
        std_msgs::Bool pushed_msg;

        bool is_joint_state_topic_;
        bool is_cmd_vel_topic_;
        bool use_moveit_;

        double wheel_separation_;
        double wheel_radius_;

        JointTrajectory *jnt_tra_;
        trajectory_msgs::JointTrajectory *jnt_tra_msg_;

        double read_period_;
        double write_period_;
        double pub_period_;

        long position1_;
        long position2_;

        bool is_moving_;

    public:
        FingerController();
        ~FingerController();
        bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
        bool getDynamixelsInfo(const std::string yaml_file);
        bool loadDynamixels(void);
        bool initDynamixels(void);
        bool initControlItems(void);
        bool initSDKHandlers(void);
        bool getPresentPosition(std::vector<std::string> dxl_name);

        double getReadPeriod(){return read_period_;}
        double getWritePeriod(){return write_period_;}
        double getPublishPeriod(){return pub_period_;}

        void initPublisher(void);
        void initSubscriber(void);

        void initServer();

        void readCallback(const ros::TimerEvent&);
        void writeCallback(const ros::TimerEvent&);
        void publishCallback(const ros::TimerEvent&);

        void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
        bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                        dynamixel_workbench_msgs::DynamixelCommand::Response &res);

        void FingerVelocityCallback(const hanyang_gripper_operator::finger_vel_msgConstPtr &msg);
        bool FingerCommandMsgCallback(hanyang_gripper_controller::FingerCommand::Request &req,
                                     hanyang_gripper_controller::FingerCommand::Response &res);
        void buttonPushedCallback(const std_msgs::Bool &msg);
        // void getPushState(const std_msgs::Bool &msg);
};

#endif