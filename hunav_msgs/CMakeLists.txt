cmake_minimum_required(VERSION 3.5)
project(hunav_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(action_msgs REQUIRED)

#nav2_package()

# set(action_files
#   "action/GoToWaypoint.action"
#   "action/FollowWaypoints.action"
#   "action/Wait.action"
#   "action/Spin.action"
#   "action/Surprised.action"
#   "action/Scared.action"
#   "action/Curious.action"
#   "action/Threatening.action"
#   "action/Impassive.action"
#   #"action/Regular.action"
# )

set(msg_files
  "msg/Agent.msg"
  "msg/Agents.msg"
)

set(srv_files
  "srv/ComputeAgents.srv"
  "srv/ComputeAgent.srv"
  "srv/MoveAgent.srv"
  "srv/GetAgents.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  #${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
