cmake_minimum_required(VERSION 2.8.3)
project(r3map_mapping)

#set(CMAKE_XX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
	std_msgs
	geometry_msgs
	tf
	pcl_ros
	roscpp
	sensor_msgs
	message_generation
	)

add_message_files(
  FILES
  CloudArray.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)


catkin_package(
	CATKIN_DEPENDS 
	std_msgs
	geometry_msgs
	tf
	pcl_ros
	roscpp
	sensor_msgs
	message_runtime
)

#add_definitions(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mapping_node src/map.cpp)

#add_dependencies(mapping_node spotr_msgs_generate_messages_cpp)

target_link_libraries(mapping_node
  ${catkin_LIBRARIES}
)