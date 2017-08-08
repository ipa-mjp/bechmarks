cmake_minimum_required(VERSION 2.8.3)
project(ipa_benchmark)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	actionlib
	actionlib_msgs
	geometry_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	std_srvs
	tf
)

find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
	DIRECTORY
		srv
	FILES
		pathPlanning.srv
)

## Generate actions in the 'action' folder
add_action_files(
	DIRECTORY
		action
	FILES
		plan.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
	actionlib_msgs
	std_msgs
	geometry_msgs
 )


catkin_package(
  	INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
#  LIBRARIES ipa_benchmark
	CATKIN_DEPENDS roscpp rospy std_msgs moveit_ros_planning
	DEPENDS
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(benchmark_node src/RunBenchmark.cpp src/BenchmarkOptions.cpp src/BenchmarkExecutor.cpp)
add_dependencies(benchmark_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(benchmark_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

