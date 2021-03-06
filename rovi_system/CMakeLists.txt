cmake_minimum_required(VERSION 3.4)
project(rovi_system)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		roslib
		ros_utils
		rovi_models
		rovi_planner
		rovi_vision
		ur5_controllers
		ur5_description
		ur5_gazebo
		ur5_planner
)

# find system libraries
find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		rovi_system
	CATKIN_DEPENDS
		roscpp
		roslib
		ros_utils
		rovi_models
		rovi_planner
		rovi_vision
		ur5_controllers
		ur5_description
		ur5_gazebo
		ur5_planner
	DEPENDS
		EIGEN3
		# Boost
)

# additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	# ${Boost_INCLUDE_DIRS}
)
################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

## rovi_system (project_foo/project_foo.h)
add_library(rovi_system src/lib_rovi_system.cpp)
target_link_libraries(rovi_system ${catkin_LIBRARIES})

################################################################################################

# nodes (executables)

add_executable(demo_planning_moveit examples/demo_planning_moveit.cpp)
target_link_libraries(demo_planning_moveit ${catkin_LIBRARIES} rovi_system)

add_executable(demo_pick_and_place examples/demo_pick_and_place.cpp)
target_link_libraries(demo_pick_and_place ${catkin_LIBRARIES} rovi_system)

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		rovi_system
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/rovi_system/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################################

# experiments (executables from /tests)

message(STATUS "Addings experiments from 'tests/*/*.cpp'...")
file(GLOB files "tests/*/*.cpp")
foreach(file ${files})
	get_filename_component(name ${file} NAME_WE)
	message(STATUS "Found experiment: ${name}...")
	add_executable(${name} ${file})
	target_link_libraries(${name} ${catkin_LIBRARIES} rovi_system)
endforeach()