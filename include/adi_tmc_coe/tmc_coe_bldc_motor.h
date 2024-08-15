/**
 * Copyright (c) 2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMC_COE_BLDC_MOTOR_H
#define TMC_COE_BLDC_MOTOR_H

#include "tmc_coe_motor.h"

/* Commutation Modes available for BLDC Motors */
typedef enum
{
  BLDC_DISABLED_MOTOR = 0,
  BLDC_OPENLOOP_MOTOR,
  BLDC_CLOSEDLOOP_MOTOR
} bldc_comm_mode_t;

class TmcCoeBLDCMotor : public TmcCoeMotor
{
private:

  /* Publisher */
  void rosPublishTmcCoeInfo(const ros::TimerEvent& event) override;

  /* Subscriber */
  void initSubscriber() override;
  void cmdVelCallback(const geometry_msgs::Twist& msg) override;
  void cmdAbsPosCallback(const std_msgs::Int32 msg) override;
  void cmdRelPosCallback(const std_msgs::Int32 msg) override;

  uint32_t encoder_steps_;
  int32_t position_scaler_;
  bldc_comm_mode_t commutation_mode_;

public:
  TmcCoeBLDCMotor(ros::NodeHandle *p_nh, TmcCoeInterpreter* p_tmc_coe_interpreter,
                  uint8_t slave_number, uint8_t motor_number);
  ~TmcCoeBLDCMotor();
  void init() override;
};

#endif /* TMC_COE_BLDC_MOTOR_H */
