/**
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmc_coe_interpreter.h"
#include "ethercat.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmcCoeInterpreter::TmcCoeInterpreter(uint8_t SDO_PDO_retries, double interface_timeout) : 
SDO_PDO_retries_(SDO_PDO_retries),
interface_timeout_(interface_timeout),
input_pdo_(1), 
output_pdo_(1),
all_obj_name_(1),
all_index_(1),
all_sub_index_(1),
all_datatype_(1)
{
  b_exit_threads_ = false;
  b_cycle_finished_ = false;
  b_start_cycle_count_ = false;
  b_interface_unresponsive_ = false;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" <<  __func__ << "] called");
}

/* Destructor */
TmcCoeInterpreter::~TmcCoeInterpreter()
{
  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" <<  __func__ << "] called");

  input_pdo_.clear();
  output_pdo_.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Initialize Ethernet Interface
 * @param 2D vector of all obj_name, index, sub-index, and datatype
 */
void TmcCoeInterpreter::paramTransfer(std::vector<std::vector<std::string>> all_obj_name, 
                                      std::vector<std::vector<std::string>> all_index,
                                      std::vector<std::vector<std::string>> all_sub_index, 
                                      std::vector<std::vector<std::string>> all_datatype)
{
  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");

  all_obj_name_ = all_obj_name; 
  all_index_ = all_index;
  all_sub_index_ = all_sub_index;
  all_datatype_  = all_datatype; 
}

/** Initialize Ethernet Interface
 * @param ifname = Device name, ex. "eth0"
 * @return total slave count, if 0 = fail
 */
uint8_t TmcCoeInterpreter::initInterface(std::string ifname)
{
  ROS_INFO_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");

  if(ec_init(ifname.c_str()))
  {
    ROS_INFO_STREAM("[" << __func__ << "] Init on " << ifname << " succeeded");
      
    if(ec_config_init(FALSE) > 0)
    {
      ROS_INFO_STREAM("[" << __func__ << "] " << ec_slavecount << " slaves found and configured");
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] No slaves found! Exiting");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] No socket connection on " << ifname);
  }

  return ec_slavecount;
}

/** Set Device to Safe_Operational State, also creates Cyclic processdata
 * @param en_slave = vector of slaves enabled
 * @return true if success, false if not
 */
bool TmcCoeInterpreter::safeOPstate(std::vector<int> en_slave)
{
  uint8_t slave_index = 1;
  bool b_result = true;
  slave_ = en_slave;

  ROS_INFO_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");

  input_pdo_.resize(ec_slavecount + 1, nullptr);
  output_pdo_.resize(ec_slavecount + 1, nullptr);

  ec_config_map(&IOmap);
  ec_configdc();

  while(b_result && (slave_index <= ec_slavecount))
  {
    if(slave_[slave_index])
    {
      output_pdo_[slave_index] = (output_pdo_t*)ec_slave[slave_index].outputs;
      input_pdo_[slave_index] = (input_pdo_t*)ec_slave[slave_index].inputs;
      ROS_DEBUG_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << 
                       " mapped. Set slave to SAFE_OP");

      if(this->deviceStateChange(slave_index, SAFE_OP) == EC_STATE_SAFE_OP)
      {
        ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << " is on SAFE_OP state");
      }
      else
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << 
                         " does not reached SAFE_OP state");
        b_result = false;
      }
    }
    slave_index++;
  }

  if(b_result)
  {
    /* Create and run thread for processdata */
    ROS_INFO_STREAM("[" << __func__ << "] Creating Process Data thread...");
    processdata_thread_ = boost::thread(&TmcCoeInterpreter::processData, this);
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Will not create Process Data thread.");
  }
  
  return b_result;
}

/** Set Device and StatusWord to Operational State
 * @param en_slave = vector of slaves enabled
 * @return true if success, false if not
 */
bool TmcCoeInterpreter::OPstate()
{
  uint8_t slave_index = 1;
  bool b_result = true;

  ROS_INFO_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");

  while(b_result && (slave_index <= ec_slavecount))
  {
    if(slave_[slave_index])
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] Set slave" << static_cast<int>(slave_index) << " state to OP");

      if(this->deviceStateChange(slave_index, OPERATIONAL) == EC_STATE_OPERATIONAL)
      {
        ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << " is on OP state");

        // set statusword to Operational too 
        if(!commandCodingTransition(slave_index))
        {
          b_result = false; 
        }
      }
      else
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << " does not reach OP state");
        ec_readstate();
        ROS_ERROR_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << " State=" << 
        ec_slave[slave_index].state << " - " << ec_ALstatuscode2string(ec_slave[slave_index].ALstatuscode));
        b_result = false;        
      }
    }
    slave_index++;
  }

  if(b_result)
  {
    /* Create and run thread for errorCheck */
    ROS_INFO_STREAM("[" << __func__ << "] Creating Error Check thread...");
    error_check_thread_ = boost::thread(&TmcCoeInterpreter::errorCheck, this);
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Will not create Error Check thread.");
  }

  return b_result;
}

/** Command Coding Transition
 * @param slave_number = slave_number to which command to set
 * @return true if status word is on OPERATION_ENABLED state
 */
bool TmcCoeInterpreter::commandCodingTransition(uint8_t slave_number)
{
  bool b_result = false;
  bool b_timeout = false;
  uint8_t index = 0;
  uint8_t sequence_retries = SDO_PDO_retries_;
  std::vector<fsa_state_t> status_word_state = {READY_TO_SWITCH_ON, SWITCHED_ON, OPERATION_ENABLED};
  std::vector<control_word_state_t> control_word_state = {SHUTDOWN, SWITCH_ON, ENABLE_OPERATION};
  int8_t CW_return_val = 0;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");

  while(!b_timeout && (0 < sequence_retries))
  {
    while(index < control_word_state.size())
    {
      CW_return_val = setControlWord(slave_number, status_word_state[index], control_word_state[index]);
      if(CW_return_val <= CONTROL_WORD_FAIL)
      {
        break;
      }
      index++;
    }

    if(CW_return_val == CONTROL_WORD_SUCCESS)
    {
      b_result = true;
      break;
    }
    else if (CW_return_val == CONTROL_WORD_FAIL)
    {
      sequence_retries--;
    }
    else
    {
      b_timeout = true;
    }
  }

  if(!b_result && !b_timeout)
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave" <<  static_cast<int>(slave_number) << 
                     " StatusWord did not recover from FAULT state.");
  }
  else if(b_result)
  {
    ROS_INFO_STREAM("[" << __func__ << "] Slave" <<  static_cast<int>(slave_number) << 
                     " StatusWord state is on Operation Enabled");
  }

  return b_result;
}


/** Proper stopping of Interface */
void TmcCoeInterpreter::stopInterface()
{
  ROS_INFO_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");
  
  /* Set b_exit_thread even with no thread created. To make sure that no threads are stucked in while loop */
  b_exit_threads_ = true;

  /* Closes the thread after it finish cleanly */
  if(processdata_thread_.joinable())
  { 
    processdata_thread_.join();
    ROS_DEBUG_STREAM("[" << __func__ << "] Process Data Thread closed");
  }
  if(error_check_thread_.joinable())
  { 
    error_check_thread_.join();
    ROS_DEBUG_STREAM("[" << __func__ << "] Error Check Thread closed");
  } 

  if(!b_interface_unresponsive_ && (0 < ec_slavecount))
  {
    ROS_INFO_STREAM("[" << __func__ << "] Set all slave state to INIT");
    (void) this->deviceStateChange(0, INIT);
  }

  /* stop SOEM, close socket */
  ROS_INFO_STREAM("[" << __func__ << "] Closing socket");
  ec_close();
}

/** Set Control World
 * @param slave_number = slave_number to which command to set
 * @param response_SW = required Status Word response
 * @param requested_CW = requested Control Word
 * @return 0 if Status Word does not recover from FAULT, -1 if timeout is activated, 1 if successful
 */
int8_t TmcCoeInterpreter::setControlWord(uint8_t slave_number, fsa_state_t response_SW, 
                                         control_word_state_t requested_CW)
{
  int8_t result = 0;
  double start_time = ros::Time::now().toSec();
  double end_time = start_time;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");
  
  startCycleCounter();
  while((this->getCycleCounter() <= SDO_PDO_retries_) && statusWordState(slave_number, FAULT))
  {
    if(isCycleFinished())
    {
      ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[" << __func__ << "] Slave" <<  static_cast<int>(slave_number) 
                                << " is on FAULT, resetting...");
      output_pdo_[slave_number]->control_word = FAULT_RESET;
    }
    if(interface_timeout_ < (end_time - start_time))
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Slave" <<  static_cast<int>(slave_number) << 
                       " did not respond while changing Control Word. Timeout!");
      result = CONTROL_WORD_TIMEOUT;
      break;
    }
    end_time = ros::Time::now().toSec();
  }
  start_time = ros::Time::now().toSec();
  stopCycleCounter();
  
  startCycleCounter();
  while((result >= CONTROL_WORD_FAIL) && (this->getCycleCounter() <= SDO_PDO_retries_) && 
        !statusWordState(slave_number, response_SW))
  {
    if(isCycleFinished())
    {
      ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[" << __func__ << "] Setting Slave" <<  static_cast<int>(slave_number) 
                                << " Controlword to " << requested_CW);
      output_pdo_[slave_number]->control_word = requested_CW;
    }
    if(interface_timeout_ < (end_time - start_time))
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Slave" <<  static_cast<int>(slave_number) << 
                       " did not respond while changing CW. Timeout!");
      result = CONTROL_WORD_TIMEOUT;
      break;
    }
    end_time = ros::Time::now().toSec();
  }
  stopCycleCounter();

  if(statusWordState(slave_number, response_SW))
  {
    ROS_DEBUG_STREAM("[" << __func__ << "] Slave" <<  static_cast<int>(slave_number) << 
                    " ControlWord succesfully set to " << requested_CW);
    result = CONTROL_WORD_SUCCESS;
  }

  return result;
}

/** Set statusWord to requested status
 * @param slave_number = slave_number to which status is set
 * @param status_ = enum of available status
 * @return true if current status is equal to set status, false if not
 */
bool TmcCoeInterpreter::statusWordState(uint8_t slave_number, fsa_state_t state)
{
  bool b_result = true;
  uint16_t statusword = 0;
  uint16_t bitmasked_statusword = 0;

  ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[TmcCoeInterpreter::" << __func__ << "] called");

  statusword = input_pdo_[slave_number]->status_word;
  bitmasked_statusword = statusword & status_state_mask_[state];
  
  if(bitmasked_statusword != state_coding_val_[state])
  {
    b_result = false;
  }

  return b_result; 
}

/** Change Device State
 * @param slave_number = slave_number to which state to set
 * @param state = state to change
 * @return current state in integer
 */
uint8_t TmcCoeInterpreter::deviceStateChange(uint8_t slave_number, nmt_state_t state)
{
  uint8_t return_state = 0;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");
  
  if(slave_number <= ec_slavecount)
  {
    switch (state)
    {
      case INIT:
        ec_slave[slave_number].state = EC_STATE_INIT;
        ec_writestate(slave_number);
        if(ec_statecheck(slave_number, EC_STATE_INIT,  EC_TIMEOUTSTATE) == EC_STATE_INIT)
        {
          ROS_INFO_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) <<
                          " is successful");
          ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_number) << " is on INIT");
        }
        else
        {
          ROS_ERROR_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) << 
                           " failed");
        }
        break;

      case PRE_OP:
        ec_slave[slave_number].state = EC_STATE_PRE_OP;
        ec_writestate(slave_number);
        if(ec_statecheck(slave_number, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE) == EC_STATE_PRE_OP)
        {
          ROS_INFO_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) << 
                          " is successful");
          ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_number) << " is on PRE_OP");
        }
        else
        {
          ROS_ERROR_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) << 
                           " failed");
        }
        break;

      case SAFE_OP:
        ec_slave[slave_number].state = EC_STATE_SAFE_OP;
        ec_writestate(slave_number);
        if(ec_statecheck(slave_number, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE) == EC_STATE_SAFE_OP)
        {
          ROS_INFO_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) << 
                          " is successful");
          ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_number) << " is on SAFE_OP");
        }
        else
        {
          ROS_ERROR_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) << 
                           " failed");
        }
        break;

      case OPERATIONAL:
        ec_slave[slave_number].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave_number);
        if(ec_statecheck(slave_number, EC_STATE_OPERATIONAL,  EC_TIMEOUTSTATE) == EC_STATE_OPERATIONAL)
        {
          ROS_INFO_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) << 
                          " is successful");
          ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_number) << " is on OPERATIONAL");
        }
        else
        {
          ROS_ERROR_STREAM("[" << __func__ << "] State Change for Slave" << static_cast<int>(slave_number) << 
                           " failed");
        }
        break;
      
      case SAFE_OP_ERROR_ACK:
        ec_slave[slave_number].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
        ec_writestate(slave_number);
        ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_number) << " Error is Acknowledged");
        break;

      default:
        ROS_ERROR_STREAM("[" << __func__ << "] Wrong State request");
        break;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave number not recognized");
  }

  ec_readstate();

  return_state = ec_slave[slave_number].state;
  return return_state;
}

/** Read to SDO [High Level]
 * @param slave_number = slave_number to which command to set
 * @param object_name = Object Name to read
 * @param value = will return actual value
 * @return true if read succesful, false if not
 */
bool TmcCoeInterpreter::readSDO(uint8_t slave_number, std::string object_name, std::string *value)
{
  std::string index_str = "";
  uint16_t index_int = 0;
  std::string sub_index_str = "";
  uint16_t sub_index_int = 0;
  std::string datatype = "";
  uint16_t column = 0;
  uint16_t index = 0;
  const int val_int = 0;
  const uint32_t val_uint_max = 0;     // Used for special case, only if datatype of an object is UINT32
  bool b_result = false;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "][High-Level] called");

  if(slave_number <= ec_slavecount)
  {
    while(!b_result && (column <= all_obj_name_[slave_number].size()))
    {
      if(all_obj_name_[slave_number][column] == object_name)
      {
        ROS_DEBUG_STREAM("[" << __func__ << "] Object Name: " << all_obj_name_[slave_number][column] << " found");
        index = column;
        b_result = true;
      }
      column++;
    }

    if(b_result)
    {
      index_str = all_index_[slave_number][index];
      index_int = std::stoi(index_str, nullptr, 16);
      sub_index_str = all_sub_index_[slave_number][index];
      sub_index_int = std::stoi(sub_index_str, nullptr, 16);
      datatype = all_datatype_[slave_number][index];
      
      if(datatype == "UINT32")
      {
        *value = readSDO<uint32_t>(slave_number, index_int, sub_index_int, val_uint_max);
      }
      else
      {
        if(datatype == "UINT8")
        {
          *value = readSDO<uint8_t>(slave_number, index_int, sub_index_int, static_cast<uint8_t>(val_int));
        }
        else if(datatype == "UINT16")
        {
          *value = readSDO<uint16_t>(slave_number, index_int, sub_index_int, static_cast<uint16_t>(val_int));
        }
        else if(datatype == "INT8")
        {
          *value = readSDO<int8_t>(slave_number, index_int, sub_index_int, static_cast<int8_t>(val_int));
        }
        else if(datatype == "INT16")
        {
          *value = readSDO<int16_t>(slave_number, index_int, sub_index_int, static_cast<int16_t>(val_int));
        }
        else
        {
          *value = readSDO<int32_t>(slave_number, index_int, sub_index_int, val_int);
        }
      }
    }
    else 
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Object Name: " << object_name << " not found");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave Number: " << static_cast<int>(slave_number) << " not found");
  }

  if(*value == "")
  {
    b_result = false;
  }

  return b_result;
}

/** Write to SDO [High Level]
 * @param slave_number = slave_number to which command to set
 * @param object_name = Object Name to write
 * @param value = value to set if write and returns actual value after set
 * @return true if write succesful, false if not
 */
bool TmcCoeInterpreter::writeSDO(uint8_t slave_number, std::string object_name, std::string *value)
{
  std::string index_str = "";
  uint16_t index_int = 0;
  std::string sub_index_str = "";
  uint16_t sub_index_int = 0;
  std::string datatype = "";
  uint16_t column = 0;
  uint16_t index = 0;
  int val_int = 0;
  uint32_t val_uint_max = 0;          // Used for special case, only if datatype of an object is UINT32
  bool b_result = false;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "][High-Level] called");

  if(slave_number <= ec_slavecount)
  {
    while(!b_result && (column <= all_obj_name_[slave_number].size()))
    {
      if(all_obj_name_[slave_number][column] == object_name)
      {
        ROS_DEBUG_STREAM("[" << __func__ << "] Object Name: " << all_obj_name_[slave_number][column] << " found");
        index = column;
        b_result = true;
      }
      column++;
    }

    if(b_result)
    {
      index_str = all_index_[slave_number][index];
      index_int = std::stoi(index_str, nullptr, 16);
      sub_index_str = all_sub_index_[slave_number][index];
      sub_index_int = std::stoi(sub_index_str, nullptr, 16);
      datatype = all_datatype_[slave_number][index];
      
      if(datatype == "UINT32")
      {
        val_uint_max = std::stoull(*value);
        *value = writeSDO<uint32_t>(slave_number, index_int, sub_index_int, val_uint_max);
      }
      else
      {
        val_int = std::stoi(*value);

        if(datatype == "UINT8")
        {
          *value = writeSDO<uint8_t>(slave_number, index_int, sub_index_int, static_cast<uint8_t>(val_int));
        }
        else if(datatype == "UINT16")
        {
          *value = writeSDO<uint16_t>(slave_number, index_int, sub_index_int, static_cast<uint16_t>(val_int));
        }
        else if(datatype == "INT8")
        {
          *value = writeSDO<int8_t>(slave_number, index_int, sub_index_int, static_cast<int8_t>(val_int));
        }
        else if(datatype == "INT16")
        {
          *value = writeSDO<int16_t>(slave_number, index_int, sub_index_int, static_cast<int16_t>(val_int));
        }
        else
        {
          *value = writeSDO<int32_t>(slave_number, index_int, sub_index_int, val_int);
        }
      }
    }
    else 
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Object Name: " << object_name << " not found");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave Number: " << static_cast<int>(slave_number) << " not found");
  }

  if(*value == "")
  {
    b_result = false;
  }

  return b_result;
}

/** Getter for Slave Name
 * @param slave_number = slave_number to which name to get
 * @return slave name in string
 */
std::string TmcCoeInterpreter::getSlaveName(uint8_t slave_number)
{
  std::string slave_name = "";

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "] called");

  if(slave_[slave_number])
  {
    slave_name = ec_slave[slave_number].name;
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_number) << " is not enabled");
  }

  return slave_name;
}

/** Getter for b_cycle_finished_ flag that indicates if processdata cyle is finished
 * @return b_cycle_finished_ status
 */
bool TmcCoeInterpreter::isCycleFinished()
{
  ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[TmcCoeInterpreter::" << __func__ << "] called");

  return b_cycle_finished_;
}

/** Getter for cycle_counter_ that indicates how many the cycle has been finished
 * @return cycle_counter value
 */
uint8_t TmcCoeInterpreter::getCycleCounter()
{
  ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[TmcCoeInterpreter::" << __func__ << "] called");

  return cycle_counter_;
}

/** Starts the Cycle counter 
 */
void TmcCoeInterpreter::startCycleCounter()
{
  ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[TmcCoeInterpreter::" << __func__ << "] called");

  b_start_cycle_count_ = true;
}

/** Stops the Cycle counter 
 */
void TmcCoeInterpreter::stopCycleCounter()
{
  ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[TmcCoeInterpreter::" << __func__ << "] called");

  b_start_cycle_count_ = false;
  cycle_counter_ = 0;
}

/** Getter for b_interface_unresponsive_ flag that indicates if the master received no frames from the slave
 * @return b_interface_unresponsive_ status
 */
bool TmcCoeInterpreter::isInterfaceUnresponsive()
{
  ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[TmcCoeInterpreter::" << __func__ << "] called");

  return b_interface_unresponsive_;
}

/** Cyclic processdata
 */
void TmcCoeInterpreter::processData()
{
  ROS_INFO_STREAM("[TmcCoeInterpreter::" << __func__ << "] Process Data Thread running");

  while(!b_exit_threads_)
  {
    b_cycle_finished_ = false;
    ec_send_processdata();
    work_count_ = ec_receive_processdata(EC_TIMEOUTRET);
    
    b_cycle_finished_ = true;
    
    if(b_start_cycle_count_)
    {
      cycle_counter_++;
    }
    else
    {
      cycle_counter_ = 0;
    }
    
    boost::this_thread::sleep_for(boost::chrono::microseconds(PROCESS_DATA_DELAY)); 
  }
}

/** Error checking and recovery
 */
void TmcCoeInterpreter::errorCheck()
{
  bool b_error_cleared = false;
  uint8_t slave_index = 0;
  uint16_t expected_work_count = ec_slavecount * TOTAL_WORK_COUNTER_PER_SLAVE;
  double start_time = 0;
  double end_time = 0;

  ROS_INFO_STREAM("[TmcCoeInterpreter::" << __func__ << "] Error Check Thread running");

  while(!b_exit_threads_ && !b_interface_unresponsive_)
  {
    start_time = ros::Time::now().toSec();
    end_time = start_time;
    slave_index = 1;                                                    //resets slave_index

    while((work_count_ < expected_work_count) && (slave_index <= ec_slavecount))
    {
/* NOTE: PRE OP state is excluded in error correction as it is assumed that the user is accountable in changing the 
   state of the slave. This package allows user to stay in PRE OP state to make any modifications on the configuration
   of the slaves (Please only do this if you have enough knowledge on the slave and ofcourse CoE protocol as it may 
   destroy the firmware of the slave) */
      if(ec_slave[slave_index].state == PRE_OP)
      {
        b_error_cleared = true;
      }
      else
      {
        b_error_cleared = false;
      }

      while(!b_error_cleared && (interface_timeout_ > (end_time - start_time)))
      {
        ec_readstate();
        if((ec_slave[slave_index].state == OPERATIONAL) && statusWordState(slave_index, OPERATION_ENABLED))
        {
          start_time = ros::Time::now().toSec();                      //resets timeout count
          b_error_cleared = true;
        }
        else if((ec_slave[slave_index].state == OPERATIONAL) && !statusWordState(slave_index, OPERATION_ENABLED))
        {
          if(!this->commandCodingTransition(slave_index))
          {
            ROS_WARN_STREAM("[" << __func__ << "] Command Coding Transition fail, changing state back to SAFE OP");
            (void)this->deviceStateChange(slave_index, SAFE_OP);
          }
          else
          {
            ROS_DEBUG_STREAM("[" << __func__ << "] Command Coding Transition success!");
          }
        }
        else
        {
          if(ec_slave[slave_index].state == EC_STATE_NONE)
          {
            ROS_ERROR_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << 
                             " is lost, trying to recover...");
            if(ec_recover_slave(slave_index, EC_TIMEOUTRET3))
            {
              ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << " recovered");
            }
          }
          else if(ec_slave[slave_index].state == SAFE_OP_ERROR)
          {
            ROS_ERROR_STREAM("[" << __func__ << "]Slave" << static_cast<int>(slave_index) << 
                             " is on SAFE_OP state with ERROR : " <<
                            ec_ALstatuscode2string(ec_slave[slave_index].ALstatuscode));
            (void)this->deviceStateChange(slave_index, SAFE_OP_ERROR_ACK);
          }
          else if(ec_slave[slave_index].state == SAFE_OP)
          {
            ROS_DEBUG_STREAM("[" << __func__ << "] Setting state to OPERATIONAL!");
            (void)this->deviceStateChange(slave_index, OPERATIONAL);
          }
          else
          {
            if(ec_reconfig_slave(slave_index, EC_TIMEOUTRET3))
            {
              ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << " reconfigured!");
            }
          }
        }
        boost::this_thread::sleep_for(boost::chrono::microseconds(ERROR_CHECK_DELAY));
        end_time = ros::Time::now().toSec();
      }
      if(interface_timeout_ <= (end_time - start_time))
      {
        b_interface_unresponsive_ = true;
        break;
      }
      else
      {
        slave_index ++;
      }
    }
  }
}

/** Read to SDO [Low Level]
 * @param slave_number = slave_number to which command to set
 * @param index_number = index to read
 * @param subindex_number = subindex to read
 * @param value = used to set datatype of the object
 * @return actual value in string, returns empty if failed
 */
template <typename T> 
std::string TmcCoeInterpreter::readSDO(uint8_t slave_number, uint16_t index_number, uint16_t subindex_number, T value)
{
  int val_size = sizeof(value);
  std::string result_value = "";
  bool b_result = false;
  uint8_t n_retries_ = SDO_PDO_retries_;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "][Low-Level] called");

  while(0 < n_retries_)
  {
    if(ec_SDOread(slave_number, index_number, subindex_number, FALSE, &val_size, &value, EC_TIMEOUTSAFE) > 0)
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] SDO Read Success");
      result_value = std::to_string(value);
      break;
    }

    ROS_WARN_STREAM("[" << __func__ << "] SDO Read [" << static_cast<int>(n_retries_) << "] Retry");
    n_retries_--;
    ros::Duration(0.01).sleep();                                    //Safe tested delay time
  }
  
  if(0 == n_retries_)
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Fail to Read SDO");
  }

  return result_value; 
}

/** Read / Write to SDO [Low Level]
 * @param slave_number = slave_number to which command to set
 * @param index_number = index to read / write
 * @param subindex_number = subindex to read / write
 * @param value = value to set, will return value if set to read
 * @return actual value in string, returns empty if failed
 */
template <typename T> 
std::string TmcCoeInterpreter::writeSDO(uint8_t slave_number, uint16_t index_number, uint16_t subindex_number, T value)
{
  std::string result_value = "";
  bool b_result = false;
  uint8_t n_retries_ = SDO_PDO_retries_;

  ROS_DEBUG_STREAM("[TmcCoeInterpreter::" << __func__ << "][Low-Level] called");

  while(0 < n_retries_)
  {
    if(ec_SDOwrite(slave_number, index_number, subindex_number, FALSE, sizeof(value), &value, EC_TIMEOUTSAFE) > 0)
    {
      ROS_DEBUG_STREAM("[" << __func__ << "] SDO Write Success");
      result_value = std::to_string(value);
      break;
    }
    
    ROS_WARN_STREAM("[" << __func__ << "] SDO Write [" << static_cast<int>(n_retries_) << "] Retry");
    n_retries_--;
    ros::Duration(0.01).sleep();                                    //Safe tested delay time
  }
  
  if(0 == n_retries_)
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Fail to Write SDO");
  }

  return result_value; 
}

/* Explicit Instantiation of the template */
template std::string TmcCoeInterpreter::readSDO<uint8_t>(uint8_t slave_number, uint16_t index_number, 
                                                         uint16_t subindex_number, uint8_t value);
template std::string TmcCoeInterpreter::readSDO<uint16_t>(uint8_t slave_number, uint16_t index_number, 
                                                          uint16_t subindex_number, uint16_t value);
template std::string TmcCoeInterpreter::readSDO<uint32_t>(uint8_t slave_number, uint16_t index_number, 
                                                          uint16_t subindex_number, uint32_t value);
template std::string TmcCoeInterpreter::readSDO<int8_t>(uint8_t slave_number, uint16_t index_number, 
                                                        uint16_t subindex_number, int8_t value);
template std::string TmcCoeInterpreter::readSDO<int16_t>(uint8_t slave_number, uint16_t index_number, 
                                                         uint16_t subindex_number, int16_t value);
template std::string TmcCoeInterpreter::readSDO<int32_t>(uint8_t slave_number, uint16_t index_number, 
                                                         uint16_t subindex_number, int32_t value);

template std::string TmcCoeInterpreter::writeSDO<uint8_t>(uint8_t slave_number, uint16_t index_number, 
                                                          uint16_t subindex_number, uint8_t value);
template std::string TmcCoeInterpreter::writeSDO<uint16_t>(uint8_t slave_number, uint16_t index_number, 
                                                           uint16_t subindex_number, uint16_t value);
template std::string TmcCoeInterpreter::writeSDO<uint32_t>(uint8_t slave_number, uint16_t index_number, 
                                                           uint16_t subindex_number, uint32_t value);
template std::string TmcCoeInterpreter::writeSDO<int8_t>(uint8_t slave_number, uint16_t index_number, 
                                                         uint16_t subindex_number, int8_t value);
template std::string TmcCoeInterpreter::writeSDO<int16_t>(uint8_t slave_number, uint16_t index_number, 
                                                          uint16_t subindex_number, int16_t value);
template std::string TmcCoeInterpreter::writeSDO<int32_t>(uint8_t slave_number, uint16_t index_number, 
                                                          uint16_t subindex_number, int32_t value);
