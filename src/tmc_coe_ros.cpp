/**
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include "tmc_coe_ros.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Constructor */
TmcCoeROS::TmcCoeROS(ros::NodeHandle *p_nh) : 
p_nh_ (p_nh), 
p_motor_(1),
p_tmc_coe_interpreter_(nullptr)
{
  ROS_DEBUG_STREAM("[TmcCoeROS::" <<  __func__ << "] called");
}

/* Destructor */
TmcCoeROS::~TmcCoeROS()
{
  ROS_DEBUG_STREAM("[TmcCoeROS::" <<  __func__ << "] called");
  uint8_t slave_index = 0;
  uint8_t motor_index = 0;
  for(slave_index = 0; slave_index < p_motor_.size(); slave_index++)
  {
    for(motor_index = 0; motor_index < p_motor_[slave_index].size(); motor_index++)
    {
      delete p_motor_[slave_index][motor_index];
      p_motor_[slave_index][motor_index] = nullptr;
      ROS_DEBUG_STREAM("[" <<  __func__ << "] Deleting Slave" << static_cast<int>(slave_index) << " motor" << 
                        static_cast<int>(motor_index));
    }
  }

  if(p_tmc_coe_interpreter_ != nullptr)
  {
    ROS_DEBUG_STREAM("[" <<  __func__ << "] Deleting Interpreter and NodeHandle");
    delete p_tmc_coe_interpreter_;
    p_tmc_coe_interpreter_ = nullptr;
    p_nh_ = nullptr;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool TmcCoeROS::initialize()
{
  bool b_result = true;
  uint8_t index = 0;
  uint16_t total_motor_ = 0;

  ROS_INFO_STREAM("[TmcCoeROS::" <<  __func__ << "] called");

  s_node_name_ = ros::this_node::getName();
  s_namespace_ = ros::this_node::getNamespace();
  ROS_INFO_STREAM("[" << __func__ << "] Namespace: " << s_namespace_);
  ROS_INFO_STREAM("[" << __func__ << "] Node name: " << s_node_name_);

  if(validateInterfaceParams())
  {
    p_tmc_coe_interpreter_ = new TmcCoeInterpreter(param_SDO_PDO_retries_, param_interface_timeout_);
    total_slaves_ = p_tmc_coe_interpreter_->initInterface(param_interface_name_);
    if(total_slaves_ > 0)
    {
      if(enFlagsVectorParamCheck("/en_slave", param_en_slave_, total_slaves_) > 0)
      {
        // Shift elements of param_en_slave - to imitate CoE protocol where slave 1 is device 1
        param_en_slave_.insert(begin(param_en_slave_), 0);
        slave_name_str_.push_back("");
        total_motor_vector_.push_back(0);
        if(p_tmc_coe_interpreter_->safeOPstate(param_en_slave_))
        {
          for(index = 1; index <= total_slaves_; index++)
          {
            if(param_en_slave_[index])
            {
              slave_name_str_.push_back(p_tmc_coe_interpreter_->getSlaveName(index));
              total_motor_ = std::stoi(slave_name_str_[index].substr(5));
              total_motor_ /= 1000;
              total_motor_vector_.push_back(total_motor_);
            }
            else
            {
              slave_name_str_.push_back("");
              total_motor_vector_.push_back(0);
            }
          }

          if(validateAutogenParams())
          { 
            this->createMotor();
            this->initService();
          }
          else
          {
            ROS_ERROR_STREAM("[" << __func__ << "] validateAutogenParams() failed");
            b_result = false;
          } 

          if(!p_tmc_coe_interpreter_->OPstate())
          {
            ROS_ERROR_STREAM("[" << __func__ << "] OPstate() failed");
            b_result = false;
          }
        }
        else
        {
          ROS_ERROR_STREAM("[" << __func__ << "] safeOPstate() failed");
          b_result = false;
        }
      }
      else
      {
        ROS_WARN_STREAM("[" << __func__ << "] No Slaves are enabled");
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] initInterface() failed");
      b_result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] validateInterfaceParams() failed");
    b_result = false;
  }

  return b_result;
}

bool TmcCoeROS::validateInterfaceParams()
{
  bool b_result = true;

  ROS_INFO_STREAM("[TmcCoeROS::" <<  __func__ << "] called");

  /* Interface Name is essential to run the node, expected to exit node once parameter is missing */
  const std::string s_interface_name = s_node_name_ + "/interface_name";
  if(!p_nh_->getParam(s_interface_name, param_interface_name_))
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Failed to get interface_name. Exiting!");
    b_result = false;
  }

  /* SDO PDO retries has limits. Will set default value if limit exceeded */
  const std::string s_SDO_PDO_retries = s_node_name_ + "/SDO_PDO_retries";
  if(!p_nh_->getParam(s_SDO_PDO_retries, param_SDO_PDO_retries_))
  {
    param_SDO_PDO_retries_ = SDO_PDO_RETRIES_DEFAULT;
    p_nh_->setParam(s_SDO_PDO_retries, param_SDO_PDO_retries_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get SDO_PDO_retries, Setting to default value: " <<
                      param_SDO_PDO_retries_);
  }
  else
  {
    if(param_SDO_PDO_retries_ < SDO_PDO_RETRIES_MIN || param_SDO_PDO_retries_ > SDO_PDO_RETRIES_MAX)
    {
      param_SDO_PDO_retries_ = SDO_PDO_RETRIES_DEFAULT;
      p_nh_->setParam(s_SDO_PDO_retries, param_SDO_PDO_retries_);
      ROS_WARN_STREAM("[" << __func__ << "] Set value to SDO_PDO_retries is out of range, "
                      "Setting SDO_PDO_retries to default value: " << param_SDO_PDO_retries_); 
    }
  }

  /* interface_timeout has limits. Will set default value if limit exceeded */
  const std::string s_interface_timeout = s_node_name_ + "/interface_timeout";
  if(!p_nh_->getParam(s_interface_timeout, param_interface_timeout_))
  {
    param_interface_timeout_ = TIMEOUT_DEFAULT;
    p_nh_->setParam(s_interface_timeout, param_interface_timeout_);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get interface_timeout, Setting to default value: " <<
                      param_interface_timeout_);
  }
  else
  {
    if(param_interface_timeout_ < TIMEOUT_MIN || param_interface_timeout_ > TIMEOUT_MAX)
    {
      param_interface_timeout_ = TIMEOUT_DEFAULT;
      p_nh_->setParam(s_interface_timeout, param_interface_timeout_);
      ROS_WARN_STREAM("[" << __func__ << "] Set value to interface_timeout is out of range, "
                      "Setting interface_timeout to default value: " << param_interface_timeout_); 
    }
  }
  return b_result;
}

bool TmcCoeROS::validateAutogenParams()
{
  std::vector<std::string> param_obj_name;
  std::vector<std::string> param_index;
  std::vector<std::string> param_sub_index;
  std::vector<std::string> param_datatype;
  std::vector<std::vector<std::string>> all_obj_name;
  std::vector<std::vector<std::string>> all_index;
  std::vector<std::vector<std::string>> all_sub_index;
  std::vector<std::vector<std::string>> all_datatype;
  uint8_t index = 0;
  bool b_result = true;

  ROS_INFO_STREAM("[TmcCoeROS::" <<  __func__ << "] called");

  // Shift elements of param_en_slave - to imitate CoE protocol where slave 1 is device 1
  all_obj_name.push_back(std::vector<std::string>());
  all_index.push_back(std::vector<std::string>());
  all_sub_index.push_back(std::vector<std::string>());
  all_datatype.push_back(std::vector<std::string>());

  for(index = 0; index <= total_slaves_; index++)
  {
    // Will not check disabled slaves
    if(slave_name_str_[index] != "")
    {
      const std::string s_obj_name = s_node_name_ + "/tmcm_" + slave_name_str_[index].substr(5) + "/obj_name";
      if(!p_nh_->getParam(s_obj_name, param_obj_name))
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Failed to get obj_name. "
          "Check autogenerated YAML if broken or missing. Exiting!");
        b_result = false;
      }
      const std::string s_index = s_node_name_ + "/tmcm_" + slave_name_str_[index].substr(5) + "/index";
      if(b_result && !p_nh_->getParam(s_index, param_index))
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Failed to get index. "
          "Check autogenerated YAML if broken or missing. Exiting!");
        b_result = false;
      }
      const std::string s_sub_index = s_node_name_ + "/tmcm_" + slave_name_str_[index].substr(5) + "/sub_index";
      if(b_result && !p_nh_->getParam(s_sub_index, param_sub_index))
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Failed to get sub_index. "
          "Check autogenerated YAML if broken or missing. Exiting!");
        b_result = false;
      }
      const std::string s_datatype = s_node_name_ + "/tmcm_" + slave_name_str_[index].substr(5) + "/datatype";
      if(b_result && !p_nh_->getParam(s_datatype, param_datatype))
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Failed to get datatype. "
          "Check autogenerated YAML if broken or missing. Exiting!");
        b_result = false;
      }

      /* Check if obj_name, index, sub_index and datatype vectors have the same length/size */
      if(b_result && (param_obj_name.size() != param_index.size()) && 
        (param_index.size() != param_sub_index.size()) && (param_sub_index.size() != param_datatype.size()))
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Vector size mismatch between obj_name, index, sub_index and datatype."
          "Check autogenerated YAML. Exiting!");
        b_result = false;
      }
      else
      {
        all_obj_name.push_back(param_obj_name);
        all_index.push_back(param_index);
        all_sub_index.push_back(param_sub_index);
        all_datatype.push_back(param_datatype);
      }
    }
  }

  if(b_result)
  {
    p_tmc_coe_interpreter_->paramTransfer(all_obj_name, all_index, all_sub_index, all_datatype);
  }
  
  return b_result;
}

void TmcCoeROS::createMotor()
{
  uint8_t slave_index = 0;
  uint8_t motor_index = 0;
  uint16_t motor_type = 0;

  ROS_INFO_STREAM("[TmcCoeROS::" << __func__ << "] called");

  p_motor_.resize(total_slaves_ + 1);
  (void)enFlagsVectorParamCheck("/adhoc_mode", param_adhoc_mode_, total_slaves_);

  for(slave_index = 1; slave_index <= total_slaves_; slave_index ++)
  {
    p_motor_[slave_index].resize(total_motor_vector_[slave_index]);

    if(enFlagsVectorParamCheck("/slv" + std::to_string(slave_index) + "/en_motor", param_en_motor_, 
       total_motor_vector_[slave_index]) > 0)
    {
      std::cout << "\n";
      ROS_INFO_STREAM("[" << __func__ << "] Slave" << static_cast<int>(slave_index) << " name: " << 
                       slave_name_str_[slave_index]);
      motor_type = std::stoi(slave_name_str_[slave_index].substr(5));
      motor_type = (motor_type - (total_motor_vector_[slave_index] * 1000)) / 100;
      ROS_INFO_STREAM("[" << __func__ << "] Total motors: " << static_cast<int>(total_motor_vector_[slave_index]));
    
      /* Check for motor type */
      if(param_adhoc_mode_[slave_index - 1])
      {
        ROS_INFO_STREAM("[" << __func__ << "] Adhoc Mode is enabled");
        for(motor_index = 0; motor_index < total_motor_vector_[slave_index]; motor_index++)
        {
          p_motor_[slave_index][motor_index] = new TmcCoeMotor(p_nh_, p_tmc_coe_interpreter_, 
                                                               slave_index, motor_index);
          p_motor_[slave_index][motor_index]->init();                                                  
        }
      }
      else
      {
        if(motor_type == MOTOR_TYPE_BLDC)
        {
          ROS_INFO_STREAM("[" << __func__ << "] Motor Type : BLDC Motor");
          for(motor_index = 0; motor_index < total_motor_vector_[slave_index]; motor_index++)
          {
            p_motor_[slave_index][motor_index] = new TmcCoeBLDCMotor(p_nh_, p_tmc_coe_interpreter_, 
                                                                slave_index, motor_index);
            p_motor_[slave_index][motor_index]->init();                                                  
          }
        }
        else if((motor_type >= MOTOR_TYPE_STEPPER_MIN || motor_type <= MOTOR_TYPE_STEPPER_MAX))
        {
          ROS_INFO_STREAM("[" << __func__ << "] Motor Type : STEPPER Motor");
          for(motor_index = 0; motor_index < total_motor_vector_[slave_index]; motor_index++)
          {
            p_motor_[slave_index][motor_index] = new TmcCoeStepperMotor(p_nh_, p_tmc_coe_interpreter_,
                                                                slave_index, motor_index);
            p_motor_[slave_index][motor_index]->init();                                                  
          }
        }
        else
        {
          ROS_ERROR_STREAM("[" << __func__ << "] Invalid Motor Type. Will not create Motor Class on slave"
                           << static_cast<int>(slave_index));
        }
      }
    }
    else
    {
      ROS_INFO_STREAM("[" << __func__ << "] No enabled motor in Slave" << static_cast<int>(slave_index) << "\n");
    }
  }
}

uint8_t TmcCoeROS::enFlagsVectorParamCheck(std::string param_name, std::vector<int> &param_var, uint32_t max_val)
{
  std::vector<int> local_vector;
  uint8_t total_en_param_ = 0;
  uint8_t index = 0;

  ROS_DEBUG_STREAM("[TmcCoeROS::" << __func__ << "] called");

  /* Special handling for en_slave*/
  const std::string s_param_name = s_node_name_ + param_name;
  if(!p_nh_->getParam(s_param_name, local_vector))
  {
    for(index = 0; index < max_val; index++)
    {
      local_vector.push_back(0);
    }
    p_nh_->setParam(s_param_name, local_vector);
    ROS_WARN_STREAM("[" << __func__ << "] Failed to get " << param_name << ", setting to default value: 0");
  }
  else
  {
    /* Check if param size is equal to max_val */
    if((local_vector.size()) < max_val)
    {
      for(index = local_vector.size(); index < max_val; index++)
      {
        local_vector.push_back(0);
      }
      ROS_WARN_STREAM("[" << __func__ << "] Missing indeces for " << param_name << 
                      ", setting missing value to default: 0");
    }
    else if((local_vector.size()) > max_val)
    {
      local_vector.erase(local_vector.begin() + max_val, local_vector.end());
      ROS_WARN_STREAM("[" << __func__ << "] Indeces exceeded total " << param_name << 
                      " available, deleting unused indeces");
    }
    /* Check each indeces of param if valid */
    for(index = 0; index < local_vector.size(); index++)
    {
      if(local_vector[index] != 0 && local_vector[index] != 1)
      {
        local_vector[index] = 0;
        ROS_WARN_STREAM("[" << __func__ << "] Set value for " << param_name << static_cast<int>(index) << 
                        " is out of range, setting value to default:" << local_vector[index]);
      }
      if(local_vector[index] == 1)
      {
        total_en_param_++;
      }
    }
    param_var = std::move(local_vector);
    p_nh_->setParam(s_param_name, param_var);
  }

  return total_en_param_;
}

void TmcCoeROS::deInit()
{  
  ROS_INFO_STREAM("[TmcCoeROS::" <<  __func__ << "] called");
  p_tmc_coe_interpreter_->stopInterface();
  ROS_INFO_STREAM("[" << __func__ << "] Successfully de-initialized TMC-CoE.");
}

bool TmcCoeROS::isInterfaceUnresponsive()
{  
  bool b_result = p_tmc_coe_interpreter_->isInterfaceUnresponsive();

  ROS_DEBUG_STREAM_THROTTLE(DEBUG_PERIOD, "[TmcCoeROS::" <<  __func__ << "] called");

  if(b_result)
  {
    ROS_ERROR_STREAM("[" <<  __func__ << "] Interface is Unresponsive! Exiting..."); 
  }

  return b_result;
}

/* Initialize ROS Service */
void TmcCoeROS::initService()
{
  ROS_INFO_STREAM("[TmcCoeROS::" << __func__ << "] called");
  /* Make s_namespace_ empty if namespace is empty */
  if(s_namespace_.compare("/") == 0)
  {
    s_namespace_ = "";
  }

  std::string s_read_sdo_srvname = s_namespace_ + "/read_SDO";
  std::string s_write_sdo_srvname = s_namespace_ + "/write_SDO";
  std::string s_read_pdo_srvname = s_namespace_ + "/read_PDO";
  std::string s_write_pdo_srvname = s_namespace_ + "/write_PDO";
  std::string s_state_change_srvname = s_namespace_ + "/state_change";
  std::string s_cyclic_sync_srvname = s_namespace_ + "/cyclic_sync_mode";

  read_sdo_server_ = p_nh_->advertiseService(s_read_sdo_srvname, &TmcCoeROS::readSDOCallBack, this);
  if(ros::service::exists(s_read_sdo_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] read_sdo server advertised. Service name: " << s_read_sdo_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] read_sdo server failed to advertise.");
  }

  write_sdo_server_ = p_nh_->advertiseService(s_write_sdo_srvname, &TmcCoeROS::writeSDOCallBack, this);
  if(ros::service::exists(s_write_sdo_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] write_sdo server advertised. Service name: " << s_write_sdo_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] write_sdo server failed to advertise.");
  }

  read_pdo_server_ = p_nh_->advertiseService(s_read_pdo_srvname, &TmcCoeROS::readPDOCallBack, this);
  if(ros::service::exists(s_read_pdo_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] read_pdo server advertised. Service name: " << s_read_pdo_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] read_pdo server failed to advertise.");
  }
  
  write_pdo_server_ = p_nh_->advertiseService(s_write_pdo_srvname, &TmcCoeROS::writePDOCallBack, this);
  if(ros::service::exists(s_write_pdo_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] write_pdo server advertised. Service name: " << s_write_pdo_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] write_pdo server failed to advertise.");
  }

  state_change_server_ = p_nh_->advertiseService(s_state_change_srvname, &TmcCoeROS::stateChangeCallback, this);
  if(ros::service::exists(s_state_change_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] state_change server advertised. Service name: " << s_state_change_srvname);
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] state_change server failed to advertise.");
  }

  cyclic_sync_server_ = p_nh_->advertiseService(s_cyclic_sync_srvname, &TmcCoeROS::cyclicSyncModeCallback, this);
  if(ros::service::exists(s_cyclic_sync_srvname, true))
  {
    ROS_INFO_STREAM("[" << __func__ << "] cyclic_sync server advertised. Service name: " << 
                     s_cyclic_sync_srvname << "\n");
  }
  else
  {
    ROS_WARN_STREAM("[" << __func__ << "] cyclic_sync server failed to advertise.\n");
  }
}

/* ROS Service Callbacks */

/* Make sure that the input for index number / sub index number is in hex */
bool TmcCoeROS::readSDOCallBack(adi_tmc_coe::read_write_SDO::Request& req, 
                                adi_tmc_coe::read_write_SDO::Response& res)
{
  bool b_result = true;
  std::string local_value = "";

  ROS_DEBUG_STREAM("[TmcCoeROS::" << __func__ << "] called");

  if(req.slave_number <= total_slaves_ && req.slave_number > 0)
  {
    if(!p_tmc_coe_interpreter_->readSDO(req.slave_number, req.object_name, &local_value))
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Service SDO Read Fail");
      b_result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave number not recognized");
    b_result = false;
  }

  res.output = local_value;
  res.result = b_result;

  return true;
}

/* Make sure that the input for index number / sub index number is in hex */
bool TmcCoeROS::writeSDOCallBack(adi_tmc_coe::read_write_SDO::Request& req, 
                                 adi_tmc_coe::read_write_SDO::Response& res)
{
  bool b_result = true;
  std::string local_value = req.value;

  ROS_DEBUG_STREAM("[TmcCoeROS::" << __func__ << "] called");

  if(req.slave_number <= total_slaves_ && req.slave_number > 0)
  {
    if(!p_tmc_coe_interpreter_->writeSDO(req.slave_number, req.object_name, &local_value))
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Service SDO Write Fail");
      b_result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave number not recognized");
    b_result = false;
  }

  res.output = local_value;
  res.result = b_result;

  return true;
}

bool TmcCoeROS::writePDOCallBack(adi_tmc_coe::read_write_PDO::Request& req, adi_tmc_coe::read_write_PDO::Response& res)
{
  bool b_result = true;
  int32_t actual_value = 0;

  ROS_DEBUG_STREAM("[TmcCoeROS::" << __func__ << "] called");

  /* Make inputs uppercase */
  std::transform(req.cmd.begin(), req.cmd.end(), req.cmd.begin(), ::toupper);

  if(req.slave_number <= total_slaves_ && req.slave_number > 0)
  {
    p_tmc_coe_interpreter_->startCycleCounter();

    if(req.cmd == "MODES OF OPERATION")
    {
      while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->modes_of_operation = req.value;
          if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display == req.value)
          {
            p_tmc_coe_interpreter_->stopCycleCounter();
            break;
          }
        }
      }

      if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display != req.value)
      {
        ROS_ERROR_STREAM("[" << __func__ << "]Modes of Operation not set properly");
        b_result = false;
      }

      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display;
    }
    else if(req.cmd == "CONTROLWORD")
    {
      while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->control_word = req.value;
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->status_word;
    }
    else if(req.cmd == "TARGET POSITION")
    {
      while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->target_position = req.value;
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->position_actual_value;
    }
    else if(req.cmd == "TARGET VELOCITY")
    {
      while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->target_velocity = req.value;
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->velocity_actual_value;
    }
    else if(req.cmd == "TARGET TORQUE")
    {
      while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->target_torque = req.value;
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->torque_actual_value;
    }
    else
    {
      p_tmc_coe_interpreter_->stopCycleCounter();
      ROS_ERROR_STREAM("[" << __func__ << "] Wrong CMD input");
      b_result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave number not recognized");
    b_result = false;
  }

  res.actual_value = actual_value;
  res.result = b_result;

  return true;
}

bool TmcCoeROS::readPDOCallBack(adi_tmc_coe::read_write_PDO::Request& req, adi_tmc_coe::read_write_PDO::Response& res)
{
  bool b_result = true;
  int32_t actual_value = 0;

  ROS_DEBUG_STREAM("[TmcCoeROS::" << __func__ << "] called");

  /* Make inputs uppercase */
  std::transform(req.cmd.begin(), req.cmd.end(), req.cmd.begin(), ::toupper);

  if(req.slave_number <= total_slaves_ && req.slave_number > 0)
  {
    if(req.cmd == "MODES OF OPERATION DISPLAY")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display;
    }
    else if(req.cmd == "STATUSWORD")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->status_word;
    }
    else if(req.cmd == "ACTUAL POSITION")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->position_actual_value;
    }
    else if(req.cmd == "DEMAND POSITION")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->position_demand_value;
    }
    else if(req.cmd == "ACTUAL VELOCITY")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->velocity_actual_value;
    }
    else if(req.cmd == "DEMAND VELOCITY")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->velocity_demand_value;
    }
    else if(req.cmd == "ACTUAL TORQUE")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->torque_actual_value;
    }
    else if(req.cmd == "DEMAND TORQUE")
    {
      actual_value = p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->torque_demand_value;
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Wrong CMD input");
      b_result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave number not recognized");
    b_result = false;
  }

  res.actual_value = actual_value;
  res.result = b_result;

  return true;
}

bool TmcCoeROS::stateChangeCallback(adi_tmc_coe::state_change::Request& req, adi_tmc_coe::state_change::Response& res)
{
  bool b_result = true;
  uint8_t state = 0;
  uint8_t current_state = 0;
  std::string current_state_str = "";

  ROS_DEBUG_STREAM("[TmcCoeROS::" << __func__ << "] called");

  std::transform(req.request_state.begin(), req.request_state.end(), req.request_state.begin(), ::toupper);

  if(req.slave_number <= total_slaves_ && req.slave_number > 0)
  {
    if(req.request_state == "INIT")
    {
      p_tmc_coe_interpreter_->nmt_state_ = INIT;
    }
    else if(req.request_state == "PREOP")
    {
      p_tmc_coe_interpreter_->nmt_state_ = PRE_OP;
    }
    else if(req.request_state == "SAFEOP")
    {
      p_tmc_coe_interpreter_->nmt_state_ = SAFE_OP;
    }
    else if(req.request_state == "OPERATIONAL")
    {
      p_tmc_coe_interpreter_->nmt_state_ = OPERATIONAL;
    }
    else
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Wrong request_state input");
      b_result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave number not recognized");
    b_result = false;
  }
  
  if(b_result)
  {
    current_state = (nmt_state_t) p_tmc_coe_interpreter_->deviceStateChange(req.slave_number, 
                                                                            p_tmc_coe_interpreter_->nmt_state_);

    if(current_state != p_tmc_coe_interpreter_->nmt_state_)
    {
      ROS_ERROR_STREAM("[" << __func__ << "] State Change Failed. Current device state: " << 
                       static_cast<int>(current_state));
      b_result = false;
    }

    switch (current_state)
    {
    case INIT:
      current_state_str = "INIT";
      break;
    case PRE_OP:
      current_state_str = "PREOP";
      break;
    case SAFE_OP:
      current_state_str = "SAFEOP";
      break;
    case OPERATIONAL:
      current_state_str = "OPERATIONAL";
      if(b_result)
      {
        b_result = p_tmc_coe_interpreter_->commandCodingTransition(req.slave_number);
      }
      break;
    }
    res.current_state = current_state_str;
  }
  
  res.result = b_result; 

  return true;
}

bool TmcCoeROS::cyclicSyncModeCallback(adi_tmc_coe::CSx_mode::Request& req, adi_tmc_coe::CSx_mode::Response& res)
{
  bool b_result = true;
  uint32_t index = 0;
  uint16_t motor_type = 0;
  double interpolation_time = req.interpolation_time_period * pow(10, req.interpolation_time_index);
  std::string interpolation_time_period = std::to_string(req.interpolation_time_period);
  std::string interpolation_time_index = std::to_string(req.interpolation_time_index);

  ROS_DEBUG_STREAM("[TmcCoeROS::" << __func__ << "] called");

  std::transform(req.CS_cmd.begin(), req.CS_cmd.end(), req.CS_cmd.begin(), ::toupper);

  /* Remove first 5 letters in slaves name (TMCM-) */
  motor_type = std::stoi(slave_name_str_[req.slave_number].substr(5));

  /* Parse slave model number to get 3rd digit (motor type) */
  motor_type = (motor_type - (total_motor_vector_[req.slave_number] * 1000)) / 100;

  if(req.slave_number <= total_slaves_ && req.slave_number > 0)
  {
    if(req.interpolation_time_period >= 0 && req.interpolation_time_period <= UINT8_MAX && 
       req.interpolation_time_index >= INTERPOLATION_TIME_INDEX_MIN && 
       req.interpolation_time_index <= INTERPOLATION_TIME_INDEX_MAX)
    {
      if(motor_type == MOTOR_TYPE_BLDC)
      {
        if(p_tmc_coe_interpreter_->writeSDO(req.slave_number, "Interpolation time period - Time units", 
                                            &interpolation_time_period))
        {
          ROS_DEBUG_STREAM("[" << __func__ << "] Interpolation Time period set");
        }
        else
        {
          b_result = false;
        }

        if(p_tmc_coe_interpreter_->writeSDO(req.slave_number, "Interpolation time period - Time index", 
                                            &interpolation_time_index))
        {
          ROS_DEBUG_STREAM("[" << __func__ << "] Interpolation Time index set");
        }
        else
        {
          b_result = false;
        }
      }
      else
      {
        if(p_tmc_coe_interpreter_->writeSDO(req.slave_number, 
        "Interpolation Time Period - Interpolation time period value", &interpolation_time_period))
        {
          ROS_DEBUG_STREAM("[" << __func__ << "] Interpolation Time period set");
        }
        else
        {
          b_result = false;
        }

        if(p_tmc_coe_interpreter_->writeSDO(req.slave_number, 
           "Interpolation Time Period - Interpolation time index", &interpolation_time_index))
        {
          ROS_DEBUG_STREAM("[" << __func__ << "] Interpolation Time index set");
        }
        else
        {
          b_result = false;
        }
      }
    }
    else 
    {
      ROS_ERROR_STREAM("[" << __func__ << "] Wrong interpolation_time input");
      b_result = false;
    }

    p_tmc_coe_interpreter_->startCycleCounter();

    if(req.CS_cmd == "CSP")
    {
      while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->modes_of_operation = CYCLIC_SYNC_POS;
          if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display == CYCLIC_SYNC_POS)
          {
            break;
          }
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();

      if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display != CYCLIC_SYNC_POS)
      {
        ROS_ERROR_STREAM("[" << __func__ << "]Modes of Operation not set properly");
        b_result = false;
      }
          
      if(b_result)
      {
        for(index = 0; index < req.value.size(); index++)
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->target_position = req.value[index];
          ROS_DEBUG_STREAM("[" << __func__ << "] Data Sent");
          ros::Duration(interpolation_time).sleep();
        }
      } 
      else
      {
        ROS_ERROR_STREAM("[" << __func__ << "]  Cyclic Synchronous Position Failed");
      } 
    }
    else if(req.CS_cmd == "CSV")
    {
      while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->modes_of_operation = CYCLIC_SYNC_VEL;
          if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display == CYCLIC_SYNC_VEL)
          {
            break;
          }
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();

      if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display != CYCLIC_SYNC_VEL)
      {
        ROS_ERROR_STREAM("[" << __func__ << "]Modes of Operation not set properly");
        b_result = false;
      }
          
      if(b_result)
      {
        for(index = 0; index < req.value.size(); index++)
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->target_velocity = req.value[index];
          ROS_DEBUG_STREAM("[" << __func__ << "] Data Sent");
          ros::Duration(interpolation_time).sleep();
        }
      } 
      else
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Cyclic Synchronous Velocity Failed");
      }
    }
    else if(req.CS_cmd == "CST")
    {
       while(p_tmc_coe_interpreter_->getCycleCounter() <= param_SDO_PDO_retries_)
      {
        if(p_tmc_coe_interpreter_->isCycleFinished())
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->modes_of_operation = CYCLIC_SYNC_TRQ;
          if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display == CYCLIC_SYNC_TRQ)
          {
            break;
          }
        }
      }
      p_tmc_coe_interpreter_->stopCycleCounter();

      if(p_tmc_coe_interpreter_->input_pdo_[req.slave_number]->modes_of_operation_display != CYCLIC_SYNC_TRQ)
      {
        ROS_ERROR_STREAM("[" << __func__ << "]Modes of Operation not set properly");
        b_result = false;
      }
          
      if(b_result)
      {
        for(index = 0; index < req.value.size(); index++)
        {
          p_tmc_coe_interpreter_->output_pdo_[req.slave_number]->target_torque = req.value[index];
          ROS_DEBUG_STREAM("[" << __func__ << "] Data Sent");
          ros::Duration(interpolation_time).sleep();
        }
      } 
      else
      {
        ROS_ERROR_STREAM("[" << __func__ << "] Cyclic Synchronous Torque Failed");
      }
    }
    else
    {
      p_tmc_coe_interpreter_->stopCycleCounter();
      ROS_ERROR_STREAM("[" << __func__ << "] Wrong CS_cmd input");
      b_result = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << __func__ << "] Slave number not recognized");
    b_result = false;
  }

  res.result = b_result;

  return true;
}
