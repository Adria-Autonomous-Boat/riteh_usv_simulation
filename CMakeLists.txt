cmake_minimum_required(VERSION 3.8)
project(riteh_usv_sim)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include Cpp "include" directory
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rtabmap_ros)
find_package(depth_image_proc)
find_package(navigation2)
find_package(nav2_bringup)
find_package(robot_state_publisher)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(MAVSDK REQUIRED) # MAVSDK installed apt package: libmavsdk-dev/now 2.4.1 amd64 (https://github.com/mavlink/MAVSDK/releases/tag/v2.4.1)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGpsGoal.srv"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

####################################
# install the include/usv directory to the install/include/usv
install(DIRECTORY
  include/usv
  include/utils
  include/libs
  DESTINATION include
)

#Add libraries
add_library(geo STATIC src/libs/geo.cpp)
ament_target_dependencies(geo Eigen3)
add_library(pid STATIC src/libs/pid.cpp)
ament_target_dependencies(pid Eigen3)
add_library(frame_transforms STATIC src/libs/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)

add_library(timer STATIC src/utils/timer.cpp)


# Create Cpp executable and link with dependencies
add_executable(differential_kinematics src/usv/pid_algorithm/differential_kinematics.cpp)
ament_target_dependencies(differential_kinematics rclcpp px4_msgs)

add_executable(differential_guidance src/usv/pid_algorithm/differential_guidance.cpp)
ament_target_dependencies(differential_guidance rclcpp rclcpp_lifecycle px4_msgs Eigen3 std_msgs)

add_executable(differential_drive src/usv/pid_algorithm/differential_drive.cpp)
ament_target_dependencies(differential_drive rclcpp lifecycle_msgs px4_msgs)

add_executable(differential_drive_kinematics src/usv/differential_drive_kinematics.cpp)
ament_target_dependencies(differential_drive_kinematics rclcpp px4_msgs geometry_msgs)

add_executable(frame_publisher src/usv/localization/frame_publisher.cpp)
ament_target_dependencies(frame_publisher rclcpp px4_msgs Eigen3 nav_msgs tf2_ros geometry_msgs)

add_executable(pid_qgc_server src/usv/pid_algorithm/pid_qgc_server.cpp)
ament_target_dependencies(pid_qgc_server rclcpp px4_msgs Eigen3)

add_executable(nav2_qgc_server src/usv/nav2_qgc_server.cpp)
ament_target_dependencies(nav2_qgc_server rclcpp px4_msgs Eigen3 rclcpp_action nav2_msgs)

# Link libraries to targets
target_link_libraries(differential_guidance geo pid timer frame_transforms)
target_link_libraries(differential_kinematics timer)
target_link_libraries(differential_drive_kinematics pid timer frame_transforms)
target_link_libraries(frame_publisher frame_transforms)
target_link_libraries(pid_qgc_server geo frame_transforms)
target_link_libraries(nav2_qgc_server geo frame_transforms MAVSDK::mavsdk)

# ---- BT plugin --------------------------------------------------------
add_library(usv_bt_nodes SHARED
  usv_bt/src/gps_anchor_ready_condition.cpp
  usv_bt/src/navigate_channel_action.cpp
  usv_bt/src/vessel_in_range_condition.cpp
  usv_bt/src/colreg_avoid_action.cpp
  usv_bt/src/vessel_clear_condition.cpp
  usv_bt/src/northing_above_condition.cpp
  usv_bt/src/usv_bt_nodes_plugin.cpp
)
target_include_directories(usv_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/usv_bt/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(usv_bt_nodes
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp
)
target_link_libraries(usv_bt_nodes "${cpp_typesupport_target}")

install(TARGETS usv_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY usv_bt/include/
  DESTINATION include
)
install(DIRECTORY usv_bt/trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

add_executable(channel_gate_runner usv_bt/src/channel_gate_runner.cpp)
target_include_directories(channel_gate_runner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/usv_bt/include>
)
ament_target_dependencies(channel_gate_runner rclcpp behaviortree_cpp)
target_link_libraries(channel_gate_runner usv_bt_nodes ament_index_cpp::ament_index_cpp)
# -----------------------------------------------------------------------

# Install Cpp executables in the ros2_ws/install/usv/lib/usv/ folder
install(TARGETS
  differential_drive
  differential_guidance
  differential_kinematics
  differential_drive_kinematics
  frame_publisher
  pid_qgc_server
  nav2_qgc_server
  channel_gate_runner
  DESTINATION lib/${PROJECT_NAME})

########################################

# Install Python modules
# NOTE: the Python module keeps the name `usv_py` (independent of the package
# name) because it is imported as `usv_py.*` from the launch files and nodes.
ament_python_install_package(usv_py)

# Install Python executables in the ros2_ws/install/usv/lib/usv/ folder
install(PROGRAMS
  usv_py/engines_controller_sim.py
  usv_py/bridge_config.py
  usv_py/image_processing/channel_navigator.py
  usv_py/image_processing/vessel_detector.py
  DESTINATION lib/${PROJECT_NAME})

# Scripts for otter ASV
install(PROGRAMS
    scripts/otter_forward.py
    scripts/record_run.sh
    scripts/colreg.sh
    DESTINATION lib/${PROJECT_NAME})
###################################

# Copy other necessary files to share folder
install(DIRECTORY 
  launch
  config
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

#################################

ament_package()
