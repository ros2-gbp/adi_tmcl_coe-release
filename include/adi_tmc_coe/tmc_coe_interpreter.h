/**
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#ifndef TMC_COE_INTERPRETER_H
#define TMC_COE_INTERPRETER_H

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <boost/thread/thread.hpp>
#include "osal.h"

/* Network Management States */
typedef enum
{
  INIT = 1,
  PRE_OP = 2,
  SAFE_OP = 4,
  OPERATIONAL = 8,
  SAFE_OP_ERROR = 20,
  SAFE_OP_ERROR_ACK = 20
}nmt_state_t;

/* Control Word State Commands */
typedef enum
{
  SHUTDOWN = 6,      
  SWITCH_ON = 7,          
  ENABLE_OPERATION = 15,           
  FAULT_RESET = 128,   
} control_word_state_t;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Statusword State Coding for bit masking */

//Not ready to switch on,    0b0000_0000_0100_1111, 0b0000_0000_0000_0000
//Switch on disable,         0b0000_0000_0100_1111, 0b0000_0000_0100_0000
//Ready to switch on,        0b0000_0000_0110_1111, 0b0000_0000_0010_0001
//Switched on,               0b0000_0000_0110_1111, 0b0000_0000_0010_0011
//Operation enabled,         0b0000_0000_0110_1111, 0b0000_0000_0010_0111
//Quick stop active,         0b0000_0000_0110_1111, 0b0000_0000_0000_0111
//Fault reaction active,     0b0000_0000_0100_1111, 0b0000_0000_0000_1111
//Fault,                     0b0000_0000_0100_1111, 0b0000_0000_0000_1000
//Set Point Ack,             0b0001_0000_0000_0000, 0b0000_0000_0000_0000

/* Finite State Automation States */
typedef enum
{
  NOT_READY_TO_SWITCH_ON = 0,      
  SWITCH_ON_DISABLE,          
  READY_TO_SWITCH_ON,           
  SWITCHED_ON,           
  OPERATION_ENABLED,        
  QUICK_STOP_ACTIVE,          
  FAULT_REACTION_ACTIVE,
  FAULT,
  SET_POINT_ACK_IN_PROCESS,
} fsa_state_t;

/* Status Word Coding Values */
typedef enum
{
  NOT_READY_TO_SWITCH_ON_VAL = 0,
  SWITCH_ON_DISABLE_VAL = 64,
  READY_TO_SWITCH_ON_VAL = 33,
  SWITCHED_ON_VAL = 35,
  OPERATION_ENABLED_VAL = 39,
  QUICK_STOP_ACTIVE_VAL = 7,
  FAULT_REACTION_ACTIVE_VAL = 15,
  FAULT_VAL = 8,
  SET_POINT_ACK_IN_PROCESS_VAL = 4096,
} status_word_val_t;

std::vector<int> state_coding_val_ = {NOT_READY_TO_SWITCH_ON_VAL, 
                                      SWITCH_ON_DISABLE_VAL,
                                      READY_TO_SWITCH_ON_VAL,
                                      SWITCHED_ON_VAL,
                                      OPERATION_ENABLED_VAL,
                                      QUICK_STOP_ACTIVE_VAL,
                                      FAULT_REACTION_ACTIVE_VAL,
                                      FAULT_VAL,
                                      SET_POINT_ACK_IN_PROCESS_VAL};

/* Statusword Bit position */
typedef enum
{
  RTSO_BIT = 0,
  SO_BIT,
  OE_BIT,
  F_BIT,
  VE_BIT,
  QS_BIT,
  SOD_BIT,
  SPA_BIT = 12,
} statusword_pos_t;

/* Statusword State bitmask */
std::vector<int> status_state_mask_ = {
  (1<<SOD_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SOD_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SOD_BIT)|(1<<QS_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SOD_BIT)|(1<<QS_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SOD_BIT)|(1<<QS_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SOD_BIT)|(1<<QS_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SOD_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SOD_BIT)|(1<<F_BIT)|(1<<OE_BIT)|(1<<SO_BIT)|(1<<RTSO_BIT),
  (1<<SPA_BIT)
  };

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Output PDOs (Sync Manager 2) - Read/Write access */
typedef struct PACKED
{
  int8_t modes_of_operation;
  uint16_t control_word;
  int32_t target_position;
  int32_t target_velocity;
  int16_t target_torque;
} output_pdo_t;

/* Output PDOs (Sync Manager 3) - Read only access*/
typedef struct PACKED
{
  int8_t modes_of_operation_display;
  uint16_t status_word;
  int32_t position_demand_value;
  int32_t position_actual_value;
  int32_t velocity_demand_value;
  int32_t velocity_actual_value;
  int16_t torque_demand_value;
  int16_t torque_actual_value;
} input_pdo_t;

/* Constants */
const uint32_t IOMAP_SIZE = 4096;
const int8_t CONTROL_WORD_FAIL = 0;
const int8_t CONTROL_WORD_SUCCESS = 1;
const int8_t CONTROL_WORD_TIMEOUT = -1;
const uint16_t PROCESS_DATA_DELAY = 1000;  // Standard PLC cycle time
const uint16_t ERROR_CHECK_DELAY = 10000;  // Based on SOEM's simple_test script 
const uint8_t TOTAL_WORK_COUNTER_PER_SLAVE = 3; // 2counts of outputPDO workcount + 1count of inputPDO workcount
const uint8_t DEBUG_PERIOD = 1;            // Period of debug logs in seconds

class TmcCoeInterpreter
{
private:
  void processData();
  boost::thread processdata_thread_;
  void errorCheck();
  boost::thread error_check_thread_;
  int8_t setControlWord(uint8_t slave_number, fsa_state_t response_SW, control_word_state_t requested_CW);
  template <typename T>  
  std::string readSDO(uint8_t slave_number, uint16_t index_number, uint16_t subindex_number, T value);
  template <typename T> 
  std::string writeSDO(uint8_t slave_number, uint16_t index_number, uint16_t subindex_number, T value);

  char IOmap[IOMAP_SIZE];
  std::vector<int> slave_;
  std::vector<std::vector<std::string>> all_obj_name_;
  std::vector<std::vector<std::string>> all_index_;
  std::vector<std::vector<std::string>> all_sub_index_;
  std::vector<std::vector<std::string>> all_datatype_;

  uint8_t SDO_PDO_retries_;
  double interface_timeout_;
  int work_count_;
  bool b_exit_threads_;
  bool b_cycle_finished_;
  bool b_start_cycle_count_;
  uint8_t cycle_counter_;
  bool b_interface_unresponsive_;

public:
  TmcCoeInterpreter(uint8_t SDO_PDO_retries, double interface_timeout);
  ~TmcCoeInterpreter();
  void paramTransfer(std::vector<std::vector<std::string>> all_obj_name, 
                     std::vector<std::vector<std::string>> all_index,
                     std::vector<std::vector<std::string>> all_sub_index, 
                     std::vector<std::vector<std::string>> all_datatype);
  uint8_t initInterface(std::string ifname);
  bool safeOPstate(std::vector<int> en_slave);
  bool OPstate();
  void stopInterface();
  bool statusWordState(uint8_t slave_number, fsa_state_t state);
  uint8_t deviceStateChange(uint8_t slave_number, nmt_state_t state);
  bool readSDO(uint8_t slave_number, std::string object_name, std::string *value);
  bool writeSDO(uint8_t slave_number, std::string object_name, std::string *value);
  bool commandCodingTransition(uint8_t slave_number);
  std::string getSlaveName(uint8_t slave_number);
  bool isCycleFinished();
  bool isInterfaceUnresponsive();
  uint8_t getCycleCounter();
  void startCycleCounter();
  void stopCycleCounter();

  std::vector<input_pdo_t*> input_pdo_;
  std::vector<output_pdo_t*> output_pdo_;
  nmt_state_t nmt_state_;
};

#endif /* TMC_COE_INTERPRETER_H */
