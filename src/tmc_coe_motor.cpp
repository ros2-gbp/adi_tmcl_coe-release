/**
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmc_coe_motor.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmcCoeMotor::TmcCoeMotor(ros::NodeHandle *p_nh, TmcCoeInterpreter* p_tmc_coe_interpreter,
                         uint8_t slave_number, uint8_t motor_number) :
p_nh_ (p_nh),
p_tmc_coe_interpreter_ (p_tmc_coe_interpreter),
slave_number_ (slave_number),
motor_number_ (motor_number)
{
  ROS_DEBUG_STREAM("[TmcCoeMotor::" << __func__ << "] called");
  
  this->initParams();
}

/* Destructor */
TmcCoeMotor::~TmcCoeMotor()
{
  ROS_DEBUG_STREAM("[TmcCoeMotor::" << __func__ << "] called");
  p_tmc_coe_interpreter_ = nullptr;
  p_nh_ = nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TmcCoeMotor::init()
{
  ROS_INFO_STREAM("[TmcCoeMotor::" << __func__ << "] called");

  this->initPublisher();
  this->initSubscriber();
  ROS_INFO_STREAM("[" << __func__ << "] Velocity unit: rpm");
  ROS_INFO_STREAM("[" << __func__ << "] Position unit: pulses");
  ROS_INFO_STREAM("[" << __func__ << "] Torque unit: mA");

  ROS_INFO_STREAM("[" << __func__ << "] Motor" << static_cast<int>(motor_number_) << " Initialized!\n");
}

/* Initialize Parameters */
void TmcCoeMotor::initParams()
{
  s_node_name_ = ros::this_node::getName();

  ROS_INFO_STREAM("[TmcCoeMotor::" << __func__ << "] called");

  const std::string s_pub_rate_tmc_coe_info = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                              std::to_string(motor_number_) + "/pub_rate_tmc_coe_info";
  if(!p_nh_->getParam(s_pub_rate_tmc_coe_info, param_pub_rate_tmc_coe_info_))
  {
    param_pub_rate_tmc_coe_info_ = DEFAULT_RATE;
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_rate_tmc_coe_info_ for slave" << 
                    static_cast<int>(slave_number_) << " motor" << static_cast<int>(motor_number_) << 
                    ", setting to default value: " << param_pub_rate_tmc_coe_info_);
    p_nh_->setParam(s_pub_rate_tmc_coe_info, param_pub_rate_tmc_coe_info_);
  }
  else
  {
    if(param_pub_rate_tmc_coe_info_ < DEFAULT_RATE || param_pub_rate_tmc_coe_info_ > MAX_RATE)
    {
      param_pub_rate_tmc_coe_info_ = DEFAULT_RATE;
      ROS_WARN_STREAM("[" << __func__ << "] Set value to pub_rate_tmc_coe_info_ for slave" << 
                      static_cast<int>(slave_number_) << " motor" << static_cast<int>(motor_number_) << 
                      " is out of range, setting to default value: " << param_pub_rate_tmc_coe_info_);
      p_nh_->setParam(s_pub_rate_tmc_coe_info, param_pub_rate_tmc_coe_info_);
    }
  }

  const std::string s_en_pub_tmc_coe_info = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                            std::to_string(motor_number_) + "/en_pub_tmc_coe_info";
  if(!p_nh_->getParam(s_en_pub_tmc_coe_info, param_en_pub_tmc_coe_info_))
  {
    param_en_pub_tmc_coe_info_ = false;
    p_nh_->setParam(s_en_pub_tmc_coe_info, param_en_pub_tmc_coe_info_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get en_pub_tmc_coe_info for slave" << 
                    static_cast<int>(slave_number_) << " motor" << static_cast<int>(motor_number_) << 
                    ", setting to default value: " << std::boolalpha << param_en_pub_tmc_coe_info_);
  }

  const std::string s_pub_actual_vel = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                       std::to_string(motor_number_) + "/pub_actual_vel";
  if(!p_nh_->getParam(s_pub_actual_vel, param_pub_actual_vel_))
  {
    param_pub_actual_vel_ = false;
    p_nh_->setParam(s_pub_actual_vel, param_pub_actual_vel_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_vel for slave" << static_cast<int>(slave_number_) <<
                    " motor" << static_cast<int>(motor_number_) << ", setting to default value: " << std::boolalpha << 
                    param_pub_actual_vel_);
  }

  const std::string s_pub_actual_pos = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                       std::to_string(motor_number_) + "/pub_actual_pos";
  if(!p_nh_->getParam(s_pub_actual_pos, param_pub_actual_pos_))
  {
    param_pub_actual_pos_ = false;
    p_nh_->setParam(s_pub_actual_pos, param_pub_actual_pos_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_pos for slave" << static_cast<int>(slave_number_) << 
                    " motor" << static_cast<int>(motor_number_) << ", setting to default value: " << std::boolalpha << 
                    param_pub_actual_pos_);
  }

  const std::string s_pub_actual_trq = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                       std::to_string(motor_number_) + "/pub_actual_trq";
  if(!p_nh_->getParam(s_pub_actual_trq, param_pub_actual_trq_))
  {
    param_pub_actual_trq_ = false;
    p_nh_->setParam(s_pub_actual_trq, param_pub_actual_trq_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get pub_actual_trq for slave" << static_cast<int>(slave_number_) << 
    " motor" << static_cast<int>(motor_number_) << ", setting to default value: " << std::boolalpha << 
    param_pub_actual_trq_);
  }

  const std::string s_tmc_coe_info_topic = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                           std::to_string(motor_number_) + "/tmc_coe_info_topic";
  if(!p_nh_->getParam(s_tmc_coe_info_topic, param_tmc_coe_info_topic_))
  {
    param_tmc_coe_info_topic_ = "/tmc_coe_info_" + std::to_string(slave_number_) + "_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_coe_info_topic, param_tmc_coe_info_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_info_topic for slave" << static_cast<int>(slave_number_) << 
                    " motor" << static_cast<int>(motor_number_) << ", setting to default value: " << 
                    param_tmc_coe_info_topic_);
  }

  const std::string s_tmc_cmd_vel_topic = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                          std::to_string(motor_number_) + "/tmc_cmd_vel_topic";
  if(!p_nh_->getParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic_))
  {
    param_tmc_cmd_vel_topic_ = "/cmd_vel_" + std::to_string(slave_number_) + "_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_vel_topic, param_tmc_cmd_vel_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_vel_topic for slave" << static_cast<int>(slave_number_) 
                    << " motor" << static_cast<int>(motor_number_) << ", setting to default value: " << 
                    param_tmc_cmd_vel_topic_);
  }

  const std::string s_tmc_cmd_abspos_topic = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                             std::to_string(motor_number_) + "/tmc_cmd_abspos_topic";
  if(!p_nh_->getParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic_))
  {
    param_tmc_cmd_abspos_topic_ = "/cmd_abspos_" + std::to_string(slave_number_) + "_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_abspos_topic, param_tmc_cmd_abspos_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_abspos_topic for slave" << 
                    static_cast<int>(slave_number_) << " motor" << static_cast<int>(motor_number_) << 
                    ", setting to default value: " << param_tmc_cmd_abspos_topic_);
  }

  const std::string s_tmc_cmd_relpos_topic = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                             std::to_string(motor_number_) + "/tmc_cmd_relpos_topic";
  if(!p_nh_->getParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic_))
  {
    param_tmc_cmd_relpos_topic_ = "/cmd_relpos_" + std::to_string(slave_number_) + "_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_relpos_topic, param_tmc_cmd_relpos_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_relpos_topic for slave" << 
                    static_cast<int>(slave_number_) << " motor"<< static_cast<int>(motor_number_) << 
                    ", setting to default value: " << param_tmc_cmd_relpos_topic_);
  }

  const std::string s_tmc_cmd_trq_topic = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" +
                                          std::to_string(motor_number_) + "/tmc_cmd_trq_topic";
  if(!p_nh_->getParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic_))
  {
    param_tmc_cmd_trq_topic_ = "/cmd_trq_" + std::to_string(slave_number_) + "_" + std::to_string(motor_number_);
    p_nh_->setParam(s_tmc_cmd_trq_topic, param_tmc_cmd_trq_topic_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get tmc_cmd_trq_topic for slave" << static_cast<int>(slave_number_) 
                    << " motor" << static_cast<int>(motor_number_) << ", setting to default value: " << 
                    param_tmc_cmd_trq_topic_);
  }

  const std::string s_wheel_diameter = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                       std::to_string(motor_number_) + "/wheel_diameter";
  if(!p_nh_->getParam(s_wheel_diameter, param_wheel_diameter_))
  {
    param_wheel_diameter_ = 0;
    p_nh_->setParam(s_wheel_diameter, param_wheel_diameter_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get wheel_diameter for slave" << static_cast<int>(slave_number_) <<
                    " motor" << static_cast<int>(motor_number_) << ", setting to default value: " << 
                    param_wheel_diameter_);
  }
  else
  {
    if(param_wheel_diameter_ < 0)
    {

      param_wheel_diameter_ = 0;
      p_nh_->setParam(s_wheel_diameter, param_wheel_diameter_);
      ROS_WARN_STREAM("[" << __func__ << "] Set value to wheel_diameter for slave" << static_cast<int>(slave_number_) 
                      << " motor" << static_cast<int>(motor_number_) << " is out of range, setting to default value: "
                      << param_wheel_diameter_);
    }
  }

  const std::string s_additional_ratio_vel = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                             std::to_string(motor_number_) + "/additional_ratio_vel";
  if(!p_nh_->getParam(s_additional_ratio_vel, param_add_ratio_vel_))
  {
    param_add_ratio_vel_ = 1;
    p_nh_->setParam(s_additional_ratio_vel, param_add_ratio_vel_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_vel for slave" << 
                    static_cast<int>(slave_number_) << " motor" << static_cast<int>(motor_number_) << 
                    ", setting to default value: " << param_add_ratio_vel_);
  }
  const std::string s_additional_ratio_pos = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" +
                                             std::to_string(motor_number_) + "/additional_ratio_pos";
  if(!p_nh_->getParam(s_additional_ratio_pos, param_add_ratio_pos_))
  {
    param_add_ratio_pos_ = 1;
    p_nh_->setParam(s_additional_ratio_pos, param_add_ratio_pos_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_pos for slave" << 
                    static_cast<int>(slave_number_) << " motor" << static_cast<int>(motor_number_) << 
                    ", setting to default value: " << param_add_ratio_pos_);
  }

  const std::string s_additional_ratio_trq = s_node_name_ + "/slv" + std::to_string(slave_number_) + "/motor" + 
                                             std::to_string(motor_number_) + "/additional_ratio_trq";
  if(!p_nh_->getParam(s_additional_ratio_trq, param_add_ratio_trq_))
  {
    param_add_ratio_trq_ = 1;
    p_nh_->setParam(s_additional_ratio_trq, param_add_ratio_trq_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get additional_ratio_trq for slave" << 
                    static_cast<int>(slave_number_) << " motor" << static_cast<int>(motor_number_) << 
                    ", setting to default value: " << param_add_ratio_trq_);
  }

  /* Parameters that are already validated in tmc_coe_ros.cpp. Only need to get the param value */
  const std::string s_interface_name = s_node_name_ + "/interface_name";
  (void)p_nh_->getParam(s_interface_name, param_interface_name_);
  const std::string s_SDO_PDO_retries = s_node_name_ + "/SDO_PDO_retries";
  (void)p_nh_->getParam(s_SDO_PDO_retries, param_SDO_PDO_retries_);
}

/* Initialize Publisher */
void TmcCoeMotor::initPublisher()
{
  ROS_INFO_STREAM("[TmcCoeMotor::" << __func__ << "] called");
  
  // Set frame IDs
  s_namespace_ = ros::this_node::getNamespace();

  /* Checks if namespace is empty if not, remove "/" */
  if(s_namespace_.compare("/") == 0)
  {
    frame_id_ = "tmcm" + p_tmc_coe_interpreter_->getSlaveName(slave_number_).substr(5) +  "_mtr" + 
                 std::to_string(motor_number_) + "_frame";
  }
  else
  {
    s_namespace_.erase(s_namespace_.begin());
    frame_id_ = s_namespace_ + "/tmcm"  + p_tmc_coe_interpreter_->getSlaveName(slave_number_).substr(5) + "_mtr" +
                std::to_string(motor_number_) + "_frame";
  }

  if(param_en_pub_tmc_coe_info_)
  {
    tmc_coe_info_pub_ = p_nh_->advertise<adi_tmc_coe::TmcCoeInfo>(param_tmc_coe_info_topic_, 100);
    seq_ctr_ = 0;
    ros::Rate period(param_pub_rate_tmc_coe_info_);
    timer_callback_ = p_nh_->createTimer(ros::Duration(period), &TmcCoeMotor::rosPublishTmcCoeInfo, this);
    
    if(timer_callback_.isValid())
    {
      ROS_INFO_STREAM("[" << __func__  << "] Publisher TimerCallback is created successfully!");
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Publisher TimerCallback not created!");
    }
  }
}

/* Publisher Callback */
void TmcCoeMotor::rosPublishTmcCoeInfo(const ros::TimerEvent& event)
{
  std::string mode_of_operation = "";

  tmc_coe_info_msg_.header.stamp = ros::Time::now();
  tmc_coe_info_msg_.header.seq = seq_ctr_;
  tmc_coe_info_msg_.header.frame_id = frame_id_;
  tmc_coe_info_msg_.interface_name = param_interface_name_;
  tmc_coe_info_msg_.slave_number = slave_number_;
  tmc_coe_info_msg_.motor_number = motor_number_;

  /* Initialize messages to 0 first */
  tmc_coe_info_msg_.velocity = 0;
  tmc_coe_info_msg_.position = 0;
  tmc_coe_info_msg_.torque = 0;
  
  switch (p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display)
  {
    case NONE: mode_of_operation = "None"; break;
    case PROFILE_POSITION: mode_of_operation = "Profile Position"; break;
    case PROFILE_VELOCITY: mode_of_operation = "Profile Velocity"; break;
    case HOMING_MODE: mode_of_operation = "Homing Mode"; break;
    case CYCLIC_SYNC_POS: mode_of_operation = "Cyclic Synchronous Position Mode"; break;
    case CYCLIC_SYNC_VEL: mode_of_operation = "Cyclic Synchronous Velocity Mode"; break;
    case CYCLIC_SYNC_TRQ: mode_of_operation = "Cyclic Synchronous Torque Mode"; break;
    default : mode_of_operation = "NONE"; break;
  }
  tmc_coe_info_msg_.mode_of_operation = mode_of_operation;
  tmc_coe_info_msg_.status_word = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->status_word;
  
  if(param_pub_actual_vel_)
  {
    tmc_coe_info_msg_.velocity = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->velocity_actual_value * 
                                 param_add_ratio_vel_;
  }

  if(param_pub_actual_pos_)
  {
    tmc_coe_info_msg_.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
                                 param_add_ratio_pos_;
  }

  if(param_pub_actual_trq_)
  {
    tmc_coe_info_msg_.torque = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->torque_actual_value *
                               param_add_ratio_trq_;
  }

  tmc_coe_info_pub_.publish(tmc_coe_info_msg_);
  seq_ctr_++;
}

/* Initialize Subscriber */
void TmcCoeMotor::initSubscriber()
{
  ROS_INFO_STREAM("[TmcCoeMotor::" << __func__ << "] called");

  tmc_cmd_vel_sub_ = p_nh_->subscribe(param_tmc_cmd_vel_topic_, 1000, &TmcCoeMotor::cmdVelCallback, this);
  tmc_cmd_abspos_sub_ = p_nh_->subscribe(param_tmc_cmd_abspos_topic_, 1000, &TmcCoeMotor::cmdAbsPosCallback, this);
  tmc_cmd_relpos_sub_ = p_nh_->subscribe(param_tmc_cmd_relpos_topic_, 1000, &TmcCoeMotor::cmdRelPosCallback, this);
}

/* Subscriber Callback */
void TmcCoeMotor::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  float val = msg.linear.x;
  uint8_t prev_cycle_count = 0;
  uint8_t retries = 0;

  val = val / param_add_ratio_vel_;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.linear.x << " board value: "
                    << val);
  p_tmc_coe_interpreter_->startCycleCounter();

  while(retries <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      if(p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display != PROFILE_VELOCITY)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_VELOCITY;
      }
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity = val;

      while((p_tmc_coe_interpreter_->getCycleCounter() - prev_cycle_count) < 1)
      {
        // Wait until it reaches next cycle
      }

      if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity == val)
      {
        ROS_DEBUG_STREAM("["<< __func__ << "] Subscriber callback exited successfully");
        break;
      }
      prev_cycle_count = p_tmc_coe_interpreter_->getCycleCounter();
      retries++;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity != val)
  {
    ROS_ERROR_STREAM("["<< __func__ << "] Failed to set Velocity");
  }
}

void TmcCoeMotor::cmdAbsPosCallback(const std_msgs::Int32 msg)
{
  int32_t val = msg.data;
  
  val /= param_add_ratio_pos_;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: "
                    << val);
  p_tmc_coe_interpreter_->startCycleCounter();

  while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      if(p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display != PROFILE_POSITION)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = val;
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = ABSOLUTE_POSITION;
    }

    if(p_tmc_coe_interpreter_->statusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();
  p_tmc_coe_interpreter_->startCycleCounter();

  while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = ENABLE_OPERATION;
    }
    if(!p_tmc_coe_interpreter_->statusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position == val)
  {
    ROS_DEBUG_STREAM("["<< __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("["<< __func__ << "] Failed to set Absolute Position");    
  }
}

void TmcCoeMotor::cmdRelPosCallback(const std_msgs::Int32 msg)
{
  int32_t val = msg.data;
  
  val /= param_add_ratio_pos_;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: "
                    << val);
  p_tmc_coe_interpreter_->startCycleCounter();

  while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      if(p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display != PROFILE_POSITION)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = val;
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = RELATIVE_POSITION;
    }

    if(p_tmc_coe_interpreter_->statusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();
  p_tmc_coe_interpreter_->startCycleCounter();

  while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->control_word = ENABLE_OPERATION;
    }
    if(!p_tmc_coe_interpreter_->statusWordState(slave_number_, SET_POINT_ACK_IN_PROCESS))
    {
      break;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position == val)
  {
    ROS_DEBUG_STREAM("["<< __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("["<< __func__ << "] Failed to set Relative Position");    
  }
}
