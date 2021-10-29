cmake_minimum_required(VERSION 3.4)
project(rovi_models)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		gazebo_ros
)

# declare a catkin package
catkin_package(
	CATKIN_DEPENDS
		gazebo_ros
)

################################################################################################

## install gazebo stuff
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})