cmake_minimum_required(VERSION 3.5)
project(my_cpp_py_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# add further dependencies manually
#find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(tf2 REQUIRED)


#include grampc
include_directories(include /home/emelies/ros_mpc_env/grampc-2.2/include) #grampc.h
link_directories(/home/emelies/ros_mpc_env/grampc-2.2/libs) #libgrampc.a

add_executable(mpc_node src/mpc_controller.cpp src/f1tenth_kinematic_probfct.c)
ament_target_dependencies(mpc_node rclcpp nav_msgs ackermann_msgs visualization_msgs)
target_include_directories(mpc_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(mpc_node2 src/mpc_controller_supervisor.cpp src/f1tenth_kinematic_probfct_supervisor.c)
ament_target_dependencies(mpc_node2 rclcpp nav_msgs ackermann_msgs visualization_msgs std_msgs)
target_include_directories(mpc_node2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#add_executable(mpc_node_reset src/mpc_controller_reset_RL_old.cpp src/f1tenth_reset_RL_kinematic_probfct.c)
#ament_target_dependencies(mpc_node_reset rclcpp nav_msgs ackermann_msgs visualization_msgs std_msgs)
#target_include_directories(mpc_node_reset PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include>)

add_library(grampc_lib STATIC IMPORTED)
set_target_properties(grampc_lib PROPERTIES
    IMPORTED_LOCATION "/home/emelies/ros_mpc_env/grampc-2.2/libs/libgrampc.a"
)

target_link_libraries(mpc_node grampc_lib)
target_link_libraries(mpc_node2 grampc_lib)
#target_link_libraries(mpc_node_reset grampc_lib)

install(TARGETS 
    mpc_node
    mpc_node2
    #mpc_node_reset
    DESTINATION lib/${PROJECT_NAME})
    

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit rclcpp visualization_msgs nav_msgs ackermann_msgs)

install (TARGETS 
  talker
  listener
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(my_cpp_node src/my_cpp_node.cpp)
target_include_directories(my_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS my_cpp_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()