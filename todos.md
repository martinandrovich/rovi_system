# needs fixing:

- ros_utils
- ur5_gazebo
- refactor ROBOT_DESCRIPTION, l6_T_ee etc. in multiple header files -> move to one in ur5_description.h -> ur5::NUM_JOINTS
- ur5_dynamics::init() take args
- controller interface -> ur5_controllers::wsg:: or ur5_gripper::
- args names: const T& frame OR const T& pose OR const T& w_T_x (e.g. in inv_kin())
- Matrix4d or Eigen::Isometry3d for pose (T) matrix? or geometry_msg? -> CONSISTENCY