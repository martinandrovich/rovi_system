# todos

## plan

- [ ] Refactor `ur5_ros` meta-package
- [ ] Refactor `rovi_system` project structure
- [ ] Review and refactor each module (clean code, comments etc.)
- [ ] Document each module (description, theory, example, API etc.)
- [ ] Proper installation/usage guide (README.md)
- [ ] Demos

## needs fixing

- [ ] ros_utils
- [ ] ur5_gazebo
- [ ] refactor ROBOT_DESCRIPTION, l6_T_ee etc. in multiple header files -> move to one in ur5_description.h -> ur5::NUM_JOINTS
- [ ] ur5_dynamics::init() take args
- [ ] ur5_dynamics use ur5:: (description etc.)
- [ ] controller interface -> ur5_controllers::wsg:: or ur5_gripper::
- [ ] args names: const T& frame OR const T& pose OR const T& w_T_x (e.g. in inv_kin())
- [ ] Matrix4d or Eigen::Isometry3d for pose (T) matrix? or geometry_msg? -> CONSISTENCY
- [ ] replace .editorconfig with .clangformat (or better)
- [ ] pipeline for pose estimation in python (get images? rovi_gazebo service node? python module? return pose to C++)
- [ ] launch files: description -> gazebo -> controller?
