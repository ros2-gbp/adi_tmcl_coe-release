/**
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMC_COE_MOTOR_H
#define TMC_COE_MOTOR_H

#include "tmc_coe_interpreter.h"
#include "adi_tmc_coe/TmcCoeInfo.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

/* Available Modes of Operation */
typedef enum
{
  NONE = 0,
  PROFILE_POSITION,
  PROFILE_VELOCITY = 3,
  HOMING_MODE = 6,
  CYCLIC_SYNC_POS = 8,
  CYCLIC_SYNC_VEL,
  CYCLIC_SYNC_TRQ
} mode_of_operation_t;

/* Controlword command for different Position Control */
typedef enum
{
  ABSOLUTE_POSITION = 31,
  RELATIVE_POSITION = 95,
} pos_control_word_t;

const uint8_t DEFAULT_RATE = 10;      //the Minimum rate is also the default rate
const uint8_t MAX_RATE = 100;
const uint8_t BLDC_MOTOR = 6;

/* Conversion constants */

/* Derived from converting linear velocity (ROS velocity unit) to rpm (TMC board velocity unit) */
const double PI = 3.1415926535;
const uint8_t SECS_TO_MIN = 60;

/* Used for converting degrees (general position/angular unit) to steps (TMC board position/angular unit) */
const uint16_t ANGULAR_FULL_ROTATION = 360;

class TmcCoeMotor
{
private:
  void initParams();

protected:
  /* Publisher */
  void initPublisher();
  virtual void rosPublishTmcCoeInfo(const ros::TimerEvent& event);
  ros::Timer timer_callback_;
  ros::Publisher tmc_coe_info_pub_;
  adi_tmc_coe::TmcCoeInfo tmc_coe_info_msg_;
  std::string frame_id_;  
  uint32_t seq_ctr_;

  /* Subscriber */
  virtual void initSubscriber();
  virtual void cmdVelCallback(const geometry_msgs::Twist& msg);
  virtual void cmdAbsPosCallback(const std_msgs::Int32 msg);
  virtual void cmdRelPosCallback(const std_msgs::Int32 msg);
  ros::Subscriber tmc_cmd_vel_sub_;
  ros::Subscriber tmc_cmd_abspos_sub_;
  ros::Subscriber tmc_cmd_relpos_sub_;

  std::string s_node_name_;
  std::string s_namespace_;
  uint8_t motor_number_;
  uint8_t slave_number_;
  int param_SDO_PDO_retries_;
  bool param_en_pub_tmc_coe_info_;
  bool param_pub_actual_vel_;
  bool param_pub_actual_pos_;
  bool param_pub_actual_trq_;
  std::string param_tmc_coe_info_topic_;
  std::string param_tmc_cmd_vel_topic_;
  std::string param_tmc_cmd_abspos_topic_;
  std::string param_tmc_cmd_relpos_topic_;
  std::string param_tmc_cmd_trq_topic_;
  float param_wheel_diameter_;
  float param_add_ratio_vel_;
  float param_add_ratio_pos_;
  float param_add_ratio_trq_;
  std::string param_interface_name_;
  float param_pub_rate_tmc_coe_info_;
  
  ros::NodeHandle *p_nh_;
  TmcCoeInterpreter *p_tmc_coe_interpreter_;

public:
  TmcCoeMotor(ros::NodeHandle *p_nh, TmcCoeInterpreter* p_tmc_coe_interpreter,
              uint8_t slave_number, uint8_t motor_number);
  virtual ~TmcCoeMotor();
  virtual void init();
};

#endif /* TMC_COE_MOTOR_H */
