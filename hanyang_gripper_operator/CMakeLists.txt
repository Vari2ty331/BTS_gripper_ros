cmake_minimum_required(VERSION 3.0.2)
project(hanyang_gripper_operator)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  cmake_modules
  dynamixel_workbench_operators
  dynamixel_workbench_msgs
  message_generation
)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################


add_message_files(
  FILES
  finger_vel_msg.msg
  dish_size_msg.msg
  dish_size_msg_points.msg
)


generate_messages(
    DEPENDENCIES
    std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################s

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_srvs sensor_msgs trajectory_msgs geometry_msgs cmake_modules message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# add_executable(joint_operator src/joint_operator.cpp)
# add_dependencies(joint_operator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(joint_operator ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

# add_executable(wheel_operator src/wheel_operator.cpp)
# add_dependencies(wheel_operator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(wheel_operator ${catkin_LIBRARIES})

add_executable(velocity_operator src/velocity_operator.cpp)
add_dependencies(velocity_operator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_operator ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(grip_operator src/grip_operator.cpp)
add_dependencies(grip_operator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grip_operator ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(gripper_initialize src/gripper_initialize.cpp)
add_dependencies(gripper_initialize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_initialize ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

#################################################################################
## Install
#################################################################################
install(TARGETS velocity_operator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
