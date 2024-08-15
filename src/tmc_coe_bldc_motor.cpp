/**
 * Copyright (c) 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmc_coe_bldc_motor.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmcCoeBLDCMotor::TmcCoeBLDCMotor(ros::NodeHandle *p_nh, TmcCoeInterpreter* p_tmc_coe_interpreter,
                         uint8_t slave_number, uint8_t motor_number) :
TmcCoeMotor(p_nh, p_tmc_coe_interpreter, slave_number, motor_number)
{
  ROS_DEBUG_STREAM("[TmcCoeBLDCMotor::" << __func__ << "] called");
}

/* Destructor */
TmcCoeBLDCMotor::~TmcCoeBLDCMotor()
{
  ROS_DEBUG_STREAM("[TmcCoeBLDCMotor::" << __func__ << "] called");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TmcCoeBLDCMotor::init()
{
  const std::string s_commutation_mode = "Drive settings - Commutation mode";
  const std::string s_position_scaler = "Drive settings - Position Scaler";
  const std::string s_encoder_steps = "ABN encoder settings - Steps";
  std::string val = "";
  position_scaler_ = 0;
  encoder_steps_ = 0;
  
  ROS_INFO_STREAM("[TmcCoeBLDCMotor::" << __func__ << "] called");

  if(p_tmc_coe_interpreter_->readSDO(slave_number_, s_commutation_mode, &val))
  {
    commutation_mode_ = (bldc_comm_mode_t) std::stoi(val);
    val = "";
  }
  else
  {
    commutation_mode_ = BLDC_DISABLED_MOTOR;
    ROS_ERROR_STREAM("[" << __func__ << "] Object Name for Commutation Mode is not Available");
  }
  
  if(commutation_mode_ > BLDC_DISABLED_MOTOR)
  {
    if(p_tmc_coe_interpreter_->readSDO(slave_number_, s_position_scaler, &val))
    {
      position_scaler_ = std::stoi(val);
      val = "";
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Object Name for Position Scaler not Available");
    }
  }

  if(position_scaler_ == 0 && commutation_mode_ > BLDC_OPENLOOP_MOTOR)
  {
    ROS_WARN_STREAM("[" << __func__ << "] Checking Encoder Steps");

    if(p_tmc_coe_interpreter_->readSDO(slave_number_, s_encoder_steps, &val))
    {
      encoder_steps_ = std::stoi(val);
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Object Name for Encoder Steps not Available");
    }
  }

  if(param_wheel_diameter_ == 0)
  {
    ROS_INFO_STREAM("[" << __func__ << "] Velocity unit: rpm");
  }
  else
  {
    ROS_INFO_STREAM("[" << __func__ << "] Velocity unit: m/s");
  }

  if(position_scaler_ == 0 && encoder_steps_ == 0)
  {
    ROS_INFO_STREAM("[" << __func__ << "] Position unit: pulses");
  }
  else
  {
    ROS_INFO_STREAM("[" << __func__ << "] Position unit: angular degrees");
  }
  
  ROS_INFO_STREAM("[" << __func__ << "] Torque unit: mA");

  initPublisher();
  this->initSubscriber();

  ROS_INFO_STREAM("[" << __func__ << "] Motor" << static_cast<int>(motor_number_) << " Initialized!\n");
}

/* Publisher Callback */
void TmcCoeBLDCMotor::rosPublishTmcCoeInfo(const ros::TimerEvent& event)
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
    tmc_coe_info_msg_.velocity = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->velocity_actual_value;
    
    // Convert rpm to linear velocity
    if(param_wheel_diameter_ == 0)
    {
      tmc_coe_info_msg_.velocity = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->velocity_actual_value * 
                                   param_add_ratio_vel_;
    }
    else
    {
      tmc_coe_info_msg_.velocity = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->velocity_actual_value * 
                                   ((PI * param_wheel_diameter_) / SECS_TO_MIN) * param_add_ratio_vel_;
    }
  }

  if(param_pub_actual_pos_)
  {
    tmc_coe_info_msg_.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value;
    
    // Convert steps to degrees
    if(position_scaler_ > 0)
    {
      tmc_coe_info_msg_.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
                                   (ANGULAR_FULL_ROTATION / (float)position_scaler_) * param_add_ratio_pos_;
    }
    else if(encoder_steps_ > 0)
    {
      tmc_coe_info_msg_.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
                                   (ANGULAR_FULL_ROTATION / (float)encoder_steps_) * param_add_ratio_pos_;
    }
    else
    {
      tmc_coe_info_msg_.position = p_tmc_coe_interpreter_->input_pdo_[slave_number_]->position_actual_value *
                                   param_add_ratio_pos_;
    }
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
void TmcCoeBLDCMotor::initSubscriber()
{
  ROS_INFO_STREAM("[TmcCoeBLDCMotor::" << __func__ << "] called");

  switch (commutation_mode_)
  {
    case BLDC_DISABLED_MOTOR:
      ROS_INFO_STREAM("[" << __func__ << "] Commutation Mode : DISABLED");
      break;

    case BLDC_OPENLOOP_MOTOR:
      ROS_INFO_STREAM("[" << __func__ << "] Commutation Mode : OPEN LOOP");
      TmcCoeMotor::initSubscriber();
      break;
  
    default:
      ROS_INFO_STREAM("[" << __func__ << "] Commutation Mode : CLOSED LOOP");
      TmcCoeMotor::initSubscriber();
      break;
  }
}

/* Subscriber Callback */
void TmcCoeBLDCMotor::cmdVelCallback(const geometry_msgs::Twist& msg)
{
  float val = msg.linear.x;
  int32_t board_val = 0;
  uint8_t prev_cycle_count = 0;
  uint8_t retries = 0;

  //If wheel diameter is set to 0 (or no wheels connected), the input value for linearX is equal to motors rpm 
  if(param_wheel_diameter_ == 0)
  {
    board_val = val / param_add_ratio_vel_;
  }
  else
  {
    //Formula to convert linear value to rpm (unit that the board accepts)
    board_val = val * (SECS_TO_MIN / (PI * param_wheel_diameter_)) * (1 / param_add_ratio_vel_);
  }

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << val << " board value: "
                    << board_val);
  p_tmc_coe_interpreter_->startCycleCounter();

  while(retries <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      if(p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display != PROFILE_VELOCITY)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_VELOCITY;
      }
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity = board_val;

      while((p_tmc_coe_interpreter_->getCycleCounter() - prev_cycle_count) < 1)
      {
        // Wait until it reaches next cycle
      }

      if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity == board_val)
      {
        ROS_DEBUG_STREAM("["<< __func__ << "] Subscriber callback exited successfully");
        break;
      }
      prev_cycle_count = p_tmc_coe_interpreter_->getCycleCounter();
      retries++;
    }
  }
  p_tmc_coe_interpreter_->stopCycleCounter();

  if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_velocity != board_val)
  {
    ROS_ERROR_STREAM("["<< __func__ << "] Failed to set Velocity");
  }
}

void TmcCoeBLDCMotor::cmdAbsPosCallback(const std_msgs::Int32 msg)
{
  float convert_const_deg = 0.00;
  int32_t unit_val = 0;
  int32_t val = msg.data;

  //convert input(degrees) to unit
  if(position_scaler_ > 0)
  {
    convert_const_deg = ((float)position_scaler_ / ANGULAR_FULL_ROTATION) * (1 / param_add_ratio_pos_);
  }
  else if(encoder_steps_ > 0)
  {
    convert_const_deg = ((float)encoder_steps_ / ANGULAR_FULL_ROTATION) * (1 / param_add_ratio_pos_);
  }
  else
  {
    //inverting position additional ratio 
    convert_const_deg = 1 / param_add_ratio_pos_;
  }

  unit_val = val * convert_const_deg;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: "
                    << unit_val);
  p_tmc_coe_interpreter_->startCycleCounter();

  while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      if(p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display != PROFILE_POSITION)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = unit_val;
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

  if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position == unit_val)
  {
    ROS_DEBUG_STREAM("["<< __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("["<< __func__ << "] Failed to set Absolute Position");    
  }
}

void TmcCoeBLDCMotor::cmdRelPosCallback(const std_msgs::Int32 msg)
{
  float convert_const_deg = 0.00;
  int32_t unit_val = 0;
  int32_t val = msg.data;

  //convert input(degrees) to unit
  if(position_scaler_ > 0)
  {
    convert_const_deg = ((float)position_scaler_ / ANGULAR_FULL_ROTATION) * (1 / param_add_ratio_pos_);
  }
  else if(encoder_steps_ > 0)
  {
    convert_const_deg = ((float)encoder_steps_ / ANGULAR_FULL_ROTATION) * (1 / param_add_ratio_pos_);
  }
  else
  {
    //inverting position additional ratio 
    convert_const_deg = 1 / param_add_ratio_pos_;
  }

  unit_val = val * convert_const_deg;

  ROS_DEBUG_STREAM("[" << __func__ << "] Subscriber callback entered, received: " << msg.data << " board value: "
                    << unit_val);
  p_tmc_coe_interpreter_->startCycleCounter();

  while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
  {
    if(p_tmc_coe_interpreter_->isCycleFinished())
    {
      if(p_tmc_coe_interpreter_->input_pdo_[slave_number_]->modes_of_operation_display != PROFILE_POSITION)
      {
        p_tmc_coe_interpreter_->output_pdo_[slave_number_]->modes_of_operation = PROFILE_POSITION;
      }
      p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position = unit_val;
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

  if(p_tmc_coe_interpreter_->output_pdo_[slave_number_]->target_position == unit_val)
  {
    ROS_DEBUG_STREAM("["<< __func__ << "] Subscriber callback exited successfully");
  }
  else
  {
    ROS_ERROR_STREAM("["<< __func__ << "] Failed to set Relative Position");    
  }
}
