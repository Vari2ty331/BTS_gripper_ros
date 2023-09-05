#include "hanyang_gripper_operator/gripper_initialize.h"

GripperInit::GripperInit()
    :node_handle_(""),
     priv_node_handle_("~"),
     push_state(false),
     init_state(false),
     first_pos_flag(true),
     curr_position1(0),
     curr_position2(0)
{
}


GripperInit::~GripperInit(){}


void GripperInit::initPublisher(void)
{
    init_move_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("hanyang_gripper/joint_trajectory", 100);
}
void GripperInit::initSubscriber(void)
{
    dynamixel_info_sub_ = node_handle_.subscribe("hanyang_gripper/dynamixel_state",1, &GripperInit::dynamixel_state_callback, this);
    // orig_trajectory_sub_ = node_handle_.subscribe("hanyang_gripper/joint_trajectory",1,&GripperInit::joint_trajectory_callback, this);
    push_state_sub_ = node_handle_.subscribe("pushed",1,&GripperInit::push_state_callback, this);
}

void GripperInit::joint_trajectory_callback(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
    std::string asdf = msg->joint_names[1];
}

void GripperInit::dynamixel_state_callback(const dynamixel_workbench_msgs::DynamixelStateListConstPtr &msg)
{
    int32_t pos1 = msg->dynamixel_state[1].present_position;
    int32_t pos2 = msg->dynamixel_state[2].present_position;
    curr_position1 = pos1;
    curr_position2 = pos2;
    if(!push_state)
    {
        move_finger_to_init();
    }
    else
    {
        init_state = true;
    }

}

void GripperInit::push_state_callback(const std_msgs::BoolConstPtr &msg)
{
    bool state = msg->data;
    push_state = state;

}

bool GripperInit::move_finger_to_init(void)
{
    trajectory_msgs::JointTrajectory jnt_tra_msg;
    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

    double pos_subtractor = 20;
    double time_step = 1;
    
    jnt_tra_msg.joint_names.push_back(std::string("singleFinger"));
    jnt_tra_msg.joint_names.push_back(std::string("doubleFinger"));
    if(!init_state)
    {
        jnt_tra_point.positions.push_back(curr_position1+pos_subtractor);
        jnt_tra_point.positions.push_back(curr_position2+pos_subtractor);
        jnt_tra_point.time_from_start.fromSec(time_step);
        jnt_tra_msg.points.push_back(jnt_tra_point);
        init_move_pub_.publish(jnt_tra_msg);
    }
    ROS_INFO("%d,%d",curr_position1,curr_position2);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_initializer");
    GripperInit gripper_init;

    gripper_init.initSubscriber();
    gripper_init.initPublisher();
    // while(!gripper_init.push_state)
    // bool result = gripper_init.move_finger_to_init();


    ros::spin();
}