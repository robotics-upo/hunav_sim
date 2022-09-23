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
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)


include_directories(
  include
  /usr/local/include #to find lightsfm just in case
)

add_executable(hunav_agent_manager src/agent_manager_node.cpp src/agent_manager.cpp src/time_expired_condition.cpp src/bt_functions.cpp src/bt_node.cpp)
ament_target_dependencies(hunav_agent_manager rclcpp hunav_msgs people_msgs geometry_msgs visualization_msgs nav2_behavior_tree tf2 tf2_ros behaviortree_cpp_v3)


add_executable(test_agent_manager src/test_agent_manager_node.cpp)
ament_target_dependencies(test_agent_manager rclcpp hunav_msgs geometry_msgs tf2 tf2_ros)

add_executable(hunav_loader src/hunav_loader.cpp)
ament_target_dependencies(hunav_loader rclcpp hunav_msgs)


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
  test_agent_manager
  hunav_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees config
   DESTINATION share/${PROJECT_NAME}/
)

# foreach(dir behavior_trees config launch)
#     install(DIRECTORY ${dir}/
#         DESTINATION share/${PROJECT_NAME}/${dir})
# endforeach(dir)

ament_package()
