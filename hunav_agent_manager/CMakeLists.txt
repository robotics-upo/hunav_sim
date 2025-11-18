cmake_minimum_required(VERSION 3.8)
project(hunav_agent_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hunav_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)


include_directories(
  include
  /usr/local/include #to find lightsfm just in case
)

add_executable(hunav_agent_manager src/agent_manager_node.cpp src/agent_say_node.cpp src/agent_manager.cpp src/time_expired_condition.cpp src/conversation_formation_node.cpp src/stop_and_wait_timer_action_node.cpp src/go_to_node.cpp src/approach_agent_node.cpp src/approach_robot_node.cpp src/block_agent_node.cpp src/block_robot_node.cpp src/group_walk_node.cpp src/is_anyone_speaking_node.cpp src/is_speaking_node.cpp src/is_anyone_looking_at_me_node.cpp src/is_looking_at_me_node.cpp src/follow_agent_node.cpp src/look_at_point_node.cpp src/look_at_agent_node.cpp src/look_at_robot_node.cpp src/time_delay_decorator.cpp src/bt_functions.cpp src/bt_node.cpp src/avoid_robot_node.cpp)
ament_target_dependencies(hunav_agent_manager rclcpp hunav_msgs people_msgs geometry_msgs visualization_msgs nav2_behavior_tree tf2 tf2_ros behaviortree_cpp)


#add_executable(test_agent_manager src/test_agent_manager_node.cpp)
#ament_target_dependencies(test_agent_manager rclcpp hunav_msgs geometry_msgs tf2_geometry_msgs tf2 tf2_ros)

add_executable(hunav_loader src/hunav_loader.cpp)
ament_target_dependencies(hunav_loader rclcpp hunav_msgs tf2_geometry_msgs)
# ament_target_dependencies(hunav_loader rclcpp hunav_msgs tf2 tf2_geometry_msgs)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

install(TARGETS
  hunav_agent_manager
  #test_agent_manager
  hunav_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees 
   DESTINATION share/${PROJECT_NAME}/
)

# foreach(dir behavior_trees config launch)
#     install(DIRECTORY ${dir}/
#         DESTINATION share/${PROJECT_NAME}/${dir})
# endforeach(dir)

ament_package()
