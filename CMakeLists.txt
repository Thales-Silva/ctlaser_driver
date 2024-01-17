cmake_minimum_required(VERSION 3.0.2)
project(ctlaser_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	message_runtime
)

find_package(Boost REQUIRED COMPONENTS
	system
	thread
)

add_service_files(
	FILES
	set_float.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs message_generation message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}

)

set(${PROJECT_NAME}_headers
	include/${PROJECT_NAME}/ctlaser_driver.hpp
	include/${PROJECT_NAME}/ctlaser_driver_node.hpp
)

set(${PROJECT_NAME}_sources
	src/lib/ctlaser_driver.cpp
	src/lib/ctlaser_driver_node.cpp
)

add_library(${PROJECT_NAME}_lib
	${${PROJECT_NAME}_headers}
	${${PROJECT_NAME}_sources}
)

add_dependencies(${PROJECT_NAME}_lib
#	${PROJECT_NAME}_EXPORTED_TARGETS
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${PROJECT_NAME}_lib
)
