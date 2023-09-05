// #include "hanyang_gripper_controller/hanyang_gripper_controller.h"
#include "hanyang_gripper_controller/hanyang_gripper_controller.h"


FingerController::FingerController()
  :node_handle_(""),
   priv_node_handle_("~"),
   is_joint_state_topic_(false),
   is_cmd_vel_topic_(false),
   use_moveit_(false),
   wheel_separation_(0.0f),
   wheel_radius_(0.0f),
   is_moving_(false),
   position1_(0),
   position2_(0)
{
  is_joint_state_topic_ = priv_node_handle_.param<bool>("use_joint_states_topic", true);
  is_cmd_vel_topic_ = priv_node_handle_.param<bool>("use_cmd_vel_topic", false);
  use_moveit_ = priv_node_handle_.param<bool>("use_moveit", false);

  read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.010f);
  write_period_ = priv_node_handle_.param<double>("dxl_write_period", 0.010f);
  pub_period_ = priv_node_handle_.param<double>("publish_period", 0.010f);

  if (is_cmd_vel_topic_ == true)
  {
    wheel_separation_ = priv_node_handle_.param<double>("mobile_robot_config/seperation_between_wheels", 0.0);
    wheel_radius_ = priv_node_handle_.param<double>("mobile_robot_config/radius_of_wheel", 0.0);
  }

  dxl_wb_ = new DynamixelWorkbench;
  jnt_tra_ = new JointTrajectory;
  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
}

FingerController::~FingerController(){}

bool FingerController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool FingerController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

bool FingerController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {      
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool FingerController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool FingerController::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)  return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}

bool FingerController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {  
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */    
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}

bool FingerController::getPresentPosition(std::vector<std::string> dxl_name)
{
  bool result = false;
  const char* log = NULL;

  int32_t get_position[dxl_name.size()];

  uint8_t id_array[dxl_name.size()];
  uint8_t id_cnt = 0;

  for (auto const& name:dxl_name)
  {
    id_array[id_cnt++] = dynamixel_[name];
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                               id_array,
                               dxl_name.size(),
                               &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    WayPoint wp;

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                      id_array,
                                      id_cnt,
                                      control_items_["Present_Position"]->address,
                                      control_items_["Present_Position"]->data_length,
                                      get_position,
                                      &log);

    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    else
    {
      for(uint8_t index = 0; index < id_cnt; index++)
      {
        // wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
        wp.position = get_position[index];
        wp.velocity = 0.0f;
        wp.acceleration = 0.0f;
        pre_goal_.push_back(wp);
      }
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    WayPoint wp;
    uint32_t read_position;
    for (auto const& dxl:dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                     control_items_["Present_Position"]->address,
                                     control_items_["Present_Position"]->data_length,
                                     &read_position,
                                     &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      wp.position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
      wp.velocity = 0.0f;
      wp.acceleration = 0.0f;
      pre_goal_.push_back(wp);
    }
  }

  return result;
}

void FingerController::initPublisher()
{
  dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  if (is_joint_state_topic_) joint_states_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
  reletive_position_pub_ = priv_node_handle_.advertise<hanyang_gripper_controller::reletive_pos>("reletive_pos",100);
}

void FingerController::initSubscriber()
{
  trajectory_sub_ = priv_node_handle_.subscribe("joint_trajectory", 100, &FingerController::trajectoryMsgCallback, this);
  // if (is_cmd_vel_topic_) cmd_vel_sub_ = priv_node_handle_.subscribe("cmd_vel", 10, &FingerController::commandVelocityCallback, this);
  cmd_vel_sub_ = priv_node_handle_.subscribe("finger_vel", 10, &FingerController::FingerVelocityCallback, this);
  // contact_sub_ = node_handle_.subscribe("pushed", 10, &FingerController::buttonPushedCallback, this);
}

void FingerController::initServer()
{
  dynamixel_command_server_ = priv_node_handle_.advertiseService("dynamixel_command", &FingerController::dynamixelCommandMsgCallback, this);
  finger_service_server_    = priv_node_handle_.advertiseService("finger_command",    &FingerController::FingerCommandMsgCallback,    this);
}

void FingerController::readCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_read_secs =ros::Time::now().toSec();
#endif
  bool result = false;
  const char* log = NULL;

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                id_array,
                                dynamixel_.size(),
                                &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Current"]->address,
                                                  control_items_["Present_Current"]->data_length,
                                                  get_current,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Velocity"]->address,
                                                  control_items_["Present_Velocity"]->data_length,
                                                  get_velocity,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Position"]->address,
                                                  control_items_["Present_Position"]->data_length,
                                                  get_position,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    for(uint8_t index = 0; index < id_cnt; index++)
    {
      dynamixel_state[index].present_current = get_current[index];
      dynamixel_state[index].present_velocity = get_velocity[index];
      dynamixel_state[index].present_position = get_position[index];

      dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
    }
  }
  else if(dxl_wb_->getProtocolVersion() == 1.0f)
  {
    uint16_t length_of_data = control_items_["Present_Position"]->data_length +
                              control_items_["Present_Velocity"]->data_length +
                              control_items_["Present_Current"]->data_length;
    uint32_t get_all_data[length_of_data];
    uint8_t dxl_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                     control_items_["Present_Position"]->address,
                                     length_of_data,
                                     get_all_data,
                                     &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
      dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
      dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

      dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
      dxl_cnt++;
    }
  }

#ifdef DEBUG
  ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
  priv_read_secs = ros::Time::now().toSec();
#endif
}

void FingerController::publishCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif
  dynamixel_state_list_pub_.publish(dynamixel_state_list_);

  if (is_joint_state_topic_)
  {
    joint_state_msg_.header.stamp = ros::Time::now();

    joint_state_msg_.name.clear();
    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();

    uint8_t id_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      double position = 0.0;
      double velocity = 0.0;
      double effort = 0.0;

      joint_state_msg_.name.push_back(dxl.first);

      if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        if (strcmp(dxl_wb_->getModelName((uint8_t)dxl.second), "XL-320") == 0) effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
        else  effort = dxl_wb_->convertValue2Current((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
      }
      else if (dxl_wb_->getProtocolVersion() == 1.0f) effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);

      velocity = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity);
      position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_position);

      joint_state_msg_.effort.push_back(effort);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);

      id_cnt++;
    }

    joint_states_pub_.publish(joint_state_msg_);
  }

#ifdef DEBUG
  ROS_WARN("[publishCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}

void FingerController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  double wheel_velocity[dynamixel_.size()];
  int32_t dynamixel_velocity[dynamixel_.size()];

  const uint8_t LEFT = 0;
  const uint8_t RIGHT = 1;

  double robot_lin_vel = msg->linear.x;
  double robot_ang_vel = msg->angular.z;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  float rpm = 0.0;
  for (auto const& dxl:dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

//  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
//       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

  double velocity_constant_value = 1 / (wheel_radius_ * rpm * 0.10472);

  wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * wheel_separation_ / 2);
  wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * wheel_separation_ / 2);

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    if (strcmp(dxl_wb_->getModelName(id_array[0]), "XL-320") == 0)
    {
      if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
      else if (wheel_velocity[LEFT] < 0.0f) dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
      else if (wheel_velocity[LEFT] > 0.0f) dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

      if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
      else if (wheel_velocity[RIGHT] < 0.0f)  dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
      else if (wheel_velocity[RIGHT] > 0.0f)  dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
    }
    else
    {
      dynamixel_velocity[LEFT]  = wheel_velocity[LEFT] * velocity_constant_value;
      dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT] * velocity_constant_value;
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
    else if (wheel_velocity[LEFT] < 0.0f) dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
    else if (wheel_velocity[LEFT] > 0.0f) dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

    if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
    else if (wheel_velocity[RIGHT] < 0.0f)  dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
    else if (wheel_velocity[RIGHT] > 0.0f)  dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
  }

  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void FingerController::writeCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t dynamixel_position[dynamixel_.size()];

  static uint32_t point_cnt = 0;
  //static uint32_t position_cnt = 0;

  for (auto const& joint:jnt_tra_msg_->joint_names)
  {
    id_array[id_cnt] = (uint8_t)dynamixel_[joint];
    id_cnt++;
  }

  if (is_moving_ == true)
  {
    for (uint8_t index = 0; index < id_cnt; index++)
      dynamixel_position[index] = jnt_tra_msg_->points[point_cnt].positions.at(index);

    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, id_array, id_cnt, dynamixel_position, 1, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    // position_cnt++;
    // if (position_cnt >= jnt_tra_msg_->points[point_cnt].positions.size())
    point_cnt++;
    if (point_cnt >= jnt_tra_msg_->points.size())
    {
      // point_cnt++;
      // position_cnt = 0;
      // if (point_cnt >= jnt_tra_msg_->points.size())
      // {
      //   is_moving_ = false;
      //   point_cnt = 0;
      //   position_cnt = 0;

      //   ROS_INFO("Complete Execution");
      // }
      is_moving_ = false;
      point_cnt = 0;
      ROS_INFO("Complete Execution");
    }
  }

#ifdef DEBUG
  ROS_WARN("[writeCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}

void FingerController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  uint8_t id_cnt = 0;
  bool result = false;
  WayPoint wp;

  if (is_moving_ == false)
  {
    jnt_tra_msg_->joint_names.clear();
    jnt_tra_msg_->points.clear();
    pre_goal_.clear();

    result = getPresentPosition(msg->joint_names);
    if (result == false)
      ROS_ERROR("Failed to get Present Position");

    for (auto const& joint:msg->joint_names)
    {
      ROS_INFO("'%s' is ready to move", joint.c_str());

      jnt_tra_msg_->joint_names.push_back(joint);
      id_cnt++;
    }

    if (id_cnt != 0)
    {
      uint8_t cnt = 0;
      while(cnt < msg->points.size())
      {
        std::vector<WayPoint> goal;
        for (std::vector<int>::size_type id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
        {
          wp.position = msg->points[cnt].positions.at(id_num);

          if (msg->points[cnt].velocities.size() != 0)  wp.velocity = msg->points[cnt].velocities.at(id_num);
          else wp.velocity = 0.0f;

          if (msg->points[cnt].accelerations.size() != 0)  wp.acceleration = msg->points[cnt].accelerations.at(id_num);
          else wp.acceleration = 0.0f;

          goal.push_back(wp);
        }

        if (use_moveit_ == true)
        {
          trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

          for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
          {
            jnt_tra_point_msg.positions.push_back(goal[id_num].position);
            jnt_tra_point_msg.velocities.push_back(goal[id_num].velocity);
            jnt_tra_point_msg.accelerations.push_back(goal[id_num].acceleration);
          }

          jnt_tra_msg_->points.push_back(jnt_tra_point_msg);

          cnt++;
        }
        else
        {
          jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());

          double move_time = 0.0f;
          if (cnt == 0) move_time = msg->points[cnt].time_from_start.toSec();
          else move_time = msg->points[cnt].time_from_start.toSec() - msg->points[cnt-1].time_from_start.toSec();

          jnt_tra_->init(move_time,
                         write_period_,
                         pre_goal_,
                         goal);

          std::vector<WayPoint> way_point;
          trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

          for (double index = 0.0; index < move_time; index = index + write_period_)
          {
            way_point = jnt_tra_->getJointWayPoint(index);

            for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
            {
              jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
              jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
              jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
            }

            jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
            jnt_tra_point_msg.positions.clear();
            jnt_tra_point_msg.velocities.clear();
            jnt_tra_point_msg.accelerations.clear();
          }

          pre_goal_ = goal;
          cnt++;
        }
      }
      ROS_INFO("Succeeded to get joint trajectory!");
      is_moving_ = true;
    }
    else
    {
      ROS_WARN("Please check joint_name");
    }
  }
  else
  {
    ROS_INFO_THROTTLE(1, "Dynamixel is moving");
  }
}

bool FingerController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                      dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  bool result = false;
  const char* log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}


bool FingerController::FingerCommandMsgCallback(hanyang_gripper_controller::FingerCommand::Request &req, hanyang_gripper_controller::FingerCommand::Response &res)
{

  bool result = false;
  const char* log = NULL;

  dynamixel_ = this->dynamixel_;

  int32_t dynamixel_velocity[dynamixel_.size()];

  const uint8_t LEFT = 0;
  const uint8_t RIGHT = 1;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t init_vel = -16;
  bool contact_bool = false;
  
  float rpm = 0.0;
  for (auto const& dxl:dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  
  if(req.inputCommand == "initialize")
  {
      dynamixel_velocity[LEFT] = init_vel;
      dynamixel_velocity[RIGHT] = init_vel;
      res.state = true;
      result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
      // contact_sub_ = priv_node_handle_.subscribe("pushed", 10, &FingerController::getPushState, this);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }
      std_msgs::Bool::ConstPtr contact = ros::topic::waitForMessage<std_msgs::Bool>("/pushed");
      contact_bool = contact->data;
  }
  
  if(contact_bool)
  {
    dynamixel_velocity[LEFT] = 0;
    dynamixel_velocity[RIGHT] = 0;
    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
    // reletive_position_pub_ = priv_node_handle_.advertise<hanyang_gripper_controller::reletive_pos>("reletive_pos",10);
    uint8_t dxl_cnt = 0;
    for(auto const& dxl:dynamixel_){
    dynamixel_workbench_msgs::DynamixelStateList info = dynamixel_state_list_;
    dynamixel_workbench_msgs::DynamixelState dynamixel_temp = info.dynamixel_state[dxl_cnt];
    long present_position = dynamixel_temp.present_position;
    if(dxl_cnt < 1)
    {
      position1_ = present_position;
    }
    else
    {
      position2_ = present_position;
    }
    dxl_cnt++;
  }
  }
  ROS_INFO("%ld,%ld",position1_,position2_);
  return true;
}


void FingerController::FingerVelocityCallback(const hanyang_gripper_operator::finger_vel_msgConstPtr &msg)
{
    bool result = false;
  const char* log = NULL;

  dynamixel_ = this->dynamixel_;

  double wheel_velocity[dynamixel_.size()];
  int32_t dynamixel_velocity[dynamixel_.size()];

  const uint8_t LEFT = 0;
  const uint8_t RIGHT = 1;

  double robot_lin_vel = msg->allFinger;
  double robot_ang_vel = msg->singleFinger;

  double allFinger_vel = msg->allFinger;
  double singleFinger_vel = msg->singleFinger;
  double doubleFinger_vel = msg->doubleFinger;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  float rpm = 0.0;
  for (auto const& dxl:dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  dynamixel_velocity[LEFT]  = allFinger_vel + singleFinger_vel;
  dynamixel_velocity[RIGHT] = allFinger_vel + doubleFinger_vel;


  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void FingerController::buttonPushedCallback(const std_msgs::Bool &msg)
{
  bool result = false;
  const char* log = NULL;

  dynamixel_ = this->dynamixel_;

  double wheel_velocity[dynamixel_.size()];
  int32_t dynamixel_velocity[dynamixel_.size()];

  const uint8_t LEFT = 0;
  const uint8_t RIGHT = 1;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  float rpm = 0.0;
  for (auto const& dxl:dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
  
  bool button_state = msg.data;
  if (button_state){
    dynamixel_velocity[LEFT]  = int32_t(0);
    dynamixel_velocity[RIGHT] = int32_t(0);
    result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanyang_gripper_controller");
  ros::NodeHandle node_handle("");

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate = 57600;

  if (argc < 2)
  {
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
  }

  FingerController finger_controller;

  bool result = false;

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  result = finger_controller.initWorkbench(port_name, baud_rate);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return 0;
  }

  result = finger_controller.getDynamixelsInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  result = finger_controller.loadDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return 0;
  }

  result = finger_controller.initDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return 0;
  }

  result = finger_controller.initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return 0;
  }

  result = finger_controller.initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return 0;
  }

  finger_controller.initPublisher();
  finger_controller.initSubscriber();
  finger_controller.initServer();

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(finger_controller.getReadPeriod()), &FingerController::readCallback, &finger_controller);
  ros::Timer write_timer = node_handle.createTimer(ros::Duration(finger_controller.getWritePeriod()), &FingerController::writeCallback, &finger_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(finger_controller.getPublishPeriod()), &FingerController::publishCallback, &finger_controller);

  ros::spin();

  return 0;
}