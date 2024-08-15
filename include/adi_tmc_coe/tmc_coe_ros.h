/**
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMC_COE_ROS_H
#define TMC_COE_ROS_H

#include "tmc_coe_bldc_motor.h"
#include "tmc_coe_stepper_motor.h"
#include "adi_tmc_coe/CSx_mode.h"
#include "adi_tmc_coe/read_write_SDO.h"
#include "adi_tmc_coe/read_write_PDO.h"
#include "adi_tmc_coe/state_change.h"

const uint8_t MOTOR_TYPE_BLDC = 6;
const uint8_t MOTOR_TYPE_STEPPER_MIN = 0;
const uint8_t MOTOR_TYPE_STEPPER_MAX = 9;
const uint8_t SDO_PDO_RETRIES_DEFAULT = 3;
const uint8_t SDO_PDO_RETRIES_MIN = 1;
const uint8_t SDO_PDO_RETRIES_MAX = 10;
const uint8_t TIMEOUT_DEFAULT = 3;
const uint8_t TIMEOUT_MIN = 3;
const uint8_t TIMEOUT_MAX = 5;
const int8_t INTERPOLATION_TIME_INDEX_MIN = -3;
const uint8_t INTERPOLATION_TIME_INDEX_MAX = 3;

class TmcCoeROS
{
private:
  bool validateInterfaceParams();
  bool validateAutogenParams();
  void createMotor();
  uint8_t enFlagsVectorParamCheck(std::string param_name, std::vector<int> &param_var, uint32_t max_val);
  
  /* Services */
  void initService();
  bool readSDOCallBack(adi_tmc_coe::read_write_SDO::Request& req, adi_tmc_coe::read_write_SDO::Response& res);
  ros::ServiceServer read_sdo_server_;
  bool writeSDOCallBack(adi_tmc_coe::read_write_SDO::Request& req, adi_tmc_coe::read_write_SDO::Response& res);
  ros::ServiceServer write_sdo_server_;
  bool readPDOCallBack(adi_tmc_coe::read_write_PDO::Request& req, adi_tmc_coe::read_write_PDO::Response& res);
  ros::ServiceServer read_pdo_server_;
  bool writePDOCallBack(adi_tmc_coe::read_write_PDO::Request& req, adi_tmc_coe::read_write_PDO::Response& res);
  ros::ServiceServer write_pdo_server_;
  bool stateChangeCallback(adi_tmc_coe::state_change::Request& req, adi_tmc_coe::state_change::Response& res);
  ros::ServiceServer state_change_server_;
  bool cyclicSyncModeCallback(adi_tmc_coe::CSx_mode::Request& req, adi_tmc_coe::CSx_mode::Response& res);
  ros::ServiceServer cyclic_sync_server_;

  uint8_t total_slaves_;
  std::string s_node_name_;
  std::string s_namespace_;
  std::string param_interface_name_;
  int param_SDO_PDO_retries_;
  double param_interface_timeout_;
  std::vector<int> param_en_slave_;
  std::vector<int> param_adhoc_mode_;
  std::vector<int> param_en_motor_;
  std::vector<uint8_t> total_motor_vector_;
  std::vector<std::string> slave_name_str_;

  ros::NodeHandle *p_nh_;
  TmcCoeInterpreter *p_tmc_coe_interpreter_;
  std::vector<std::vector<TmcCoeMotor*>> p_motor_;

public:
  TmcCoeROS(ros::NodeHandle *p_nh);
  ~TmcCoeROS();
  bool initialize();
  void deInit();
  bool isInterfaceUnresponsive();
};

#endif /* TMC_COE_ROS_H */
