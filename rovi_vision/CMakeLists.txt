cmake_minimum_required(VERSION 3.4)
project(rovi_vision)

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
)

# OpenCV
find_package(OpenCV 4 REQUIRED)

# PCL
find_package(PCL 1.7 REQUIRED)

# Eigen3 (Finding Eigen is somewhat complicated ..)
# https://github.com/ros/geometry2/blob/noetic-devel/tf2_eigen/CMakeLists.txt
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
if(NOT EIGEN3_INCLUDE_DIRS)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		rovi_vision
	CATKIN_DEPENDS
		roscpp
		roslib
		ros_utils
	DEPENDS
		OpenCV
		PCL
		EIGEN3
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

## rovi_vision
add_library(rovi_vision
	src/DNNTemplateMatching/DNNTemplateMatching.cpp
	src/RANSACRegistrationWithICP/RANSACRegistrationWithICP.cpp
)
target_link_libraries(rovi_vision ${catkin_LIBRARIES})

################################################################################################

# nodes

# add_executable(demo_rovi_vision_DNNTemplateMatching examples/demo_rovi_vision_DNNTemplateMatching.cpp)
# target_link_libraries(demo_rovi_vision_DNNTemplateMatching ${catkin_LIBRARIES} rovi_vision)

# add_executable(demo_rovi_vision_RANSACRegistrationWithICP examples/demo_rovi_vision_RANSACRegistrationWithICP.cpp)
# target_link_libraries(demo_rovi_vision_RANSACRegistrationWithICP ${catkin_LIBRARIES} rovi_vision)

# ...

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		rovi_vision
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/rovi_vision/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)