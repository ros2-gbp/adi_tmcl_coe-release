/**
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 **/

#include <signal.h>
#include "tmc_coe_ros.h"

////////////////////////////////////////////////////////////////////////////////


/* Globals */
TmcCoeROS *p_tmc_coe = nullptr;
bool g_shutdown_signal = false;

/* Function Prototypes */
void graceful_shutdown();
void signal_callback_handler(int signum);

void graceful_shutdown()
{
  ROS_INFO_STREAM("> Initiating graceful shutdown...");
  p_tmc_coe->deInit();
  ROS_INFO_STREAM("> TMC CoE Deinitialized");
  
  delete p_tmc_coe;
  p_tmc_coe = nullptr;

  ros::shutdown();
}

void signal_callback_handler(int signum)
{
  ROS_INFO_STREAM("> Caught signal: " << signum << ". Terminating...");
  g_shutdown_signal = true;
}

int main(int argc, char** argv)
{

#ifdef DEBUG
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  /* Initialize ROS */
  ros::init(argc, argv, "tmc_coe_ros_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  try
  {
    ROS_INFO_STREAM("> Starting adi_tmc_coe ...");
    signal(SIGINT, signal_callback_handler);
    signal(SIGTERM, signal_callback_handler);
  
    ROS_INFO_STREAM("> Initializing adi_tmc_coe ...");
    ros::AsyncSpinner spinner(0);
    p_tmc_coe = new TmcCoeROS(&nh);

    if(p_tmc_coe->initialize())
    {
      ROS_INFO_STREAM("> Successfully initialized TMC CoE Interpreter.\n\n");
      spinner.start();
      while(!g_shutdown_signal && ros::ok() && !p_tmc_coe->isInterfaceUnresponsive())
      {
        // Thread running on background 
      }
      spinner.stop();
      throw-1; 
    }
    else
    {
      ROS_ERROR_STREAM("[ERROR] Initializing TMC CoE Interpreter failed!, restart node");
      throw-1; 
    }
  }
  catch(...)
  {
    ROS_ERROR_STREAM("[ERROR] Exception encountered. Exiting...");
    graceful_shutdown();
  }

  return (0);
}
