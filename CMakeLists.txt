cmake_minimum_required(VERSION 3.0.2)
project(adi_tmc_coe)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  roscpp_serialization
  soem
)

#################################
## Getting User Configurations ##
#################################

option(TMC_COE_ROS_DEBUG "Show ROS Debug Stream of tmc_coe_ros" OFF)
IF(TMC_COE_ROS_DEBUG)
add_definitions(-DDEBUG)
MESSAGE(STATUS "TMC_COE_ROS_DEBUG is enabled")
ELSE()
MESSAGE(STATUS "TMC_COE_ROS_DEBUG is disabled")
ENDIF(TMC_COE_ROS_DEBUG)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TmcCoeInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  read_write_SDO.srv
  read_write_PDO.srv
  state_change.srv
  CSx_mode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    tmc_coe_interpreter
    tmc_coe_motor
    tmc_coe_bldc_motor
    tmc_coe_stepper_motor
    tmc_coe_ros
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    geometry_msgs
    roscpp_serialization
  # DEPENDS system_lib
)

# needed to circumvent LD_LIBRARY_PATH being emptied through ethercat_grant
# in addition to not propagating ros_ethercat_loop RUNPATH to dependencies, in contrast to RPATH
SET(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${soem_INCLUDE_DIRS}
)

## Declare a C++ Libraries

### tmc_coe_interpreter Library
add_library(tmc_coe_interpreter src/tmc_coe_interpreter.cpp)
target_link_libraries(tmc_coe_interpreter ${soem_LIBRARIES})

### tmc_coe_motor Library
add_library(tmc_coe_motor src/tmc_coe_motor.cpp)
add_dependencies(tmc_coe_motor ${PROJECT_NAME}_generate_messages_cpp tmc_coe_interpreter)
target_link_libraries(tmc_coe_motor tmc_coe_interpreter)

### tmc_coe_bldc_motor Library
add_library(tmc_coe_bldc_motor src/tmc_coe_bldc_motor.cpp)
add_dependencies(tmc_coe_bldc_motor ${PROJECT_NAME}_generate_messages_cpp tmc_coe_motor tmc_coe_interpreter)
target_link_libraries(tmc_coe_bldc_motor tmc_coe_motor tmc_coe_interpreter)

### tmc_coe_stepper_motor Library
add_library(tmc_coe_stepper_motor src/tmc_coe_stepper_motor.cpp)
add_dependencies(tmc_coe_stepper_motor ${PROJECT_NAME}_generate_messages_cpp tmc_coe_motor tmc_coe_interpreter)
target_link_libraries(tmc_coe_bldc_motor tmc_coe_motor tmc_coe_interpreter)

### tmc_coe_ros Library
add_library(tmc_coe_ros src/tmc_coe_ros.cpp)
add_dependencies(tmc_coe_ros ${PROJECT_NAME}_generate_messages_cpp  tmc_coe_motor tmc_coe_bldc_motor tmc_coe_stepper_motor tmc_coe_interpreter)
add_dependencies(tmc_coe_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tmc_coe_ros tmc_coe_interpreter)
target_link_libraries(tmc_coe_ros tmc_coe_motor)
target_link_libraries(tmc_coe_ros tmc_coe_bldc_motor)
target_link_libraries(tmc_coe_ros tmc_coe_stepper_motor)

## Declare a C++ executable

### tmc_coe_ros_node Executable
add_executable(tmc_coe_ros_node src/tmc_coe_ros_node.cpp)
target_link_libraries(tmc_coe_ros_node tmc_coe_ros ${catkin_LIBRARIES})
add_dependencies(tmc_coe_ros_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############


## Mark executables for installation
install(TARGETS tmc_coe_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libaries for installation
install(TARGETS
  tmc_coe_interpreter
  tmc_coe_ros
  tmc_coe_motor
  tmc_coe_bldc_motor
  tmc_coe_stepper_motor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY
  scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  FILES_MATCHING PATTERN "*.sh"
)

install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# N/A
