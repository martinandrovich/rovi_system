## general

- [ ] update author/maintainer (email + order)
- [ ] add `README.md` to each pkg to match description of `package.xml`
- [ ] update dependencies (`package.xml` + header files) → build tool, libraries, ROS packages + versions
- [ ] update dependencies/versions in README (PCL 1.7, OpenCV 4 ..)
- [ ] update `.gitignore`
- [x] format exceptions (`msg in function()` with `std::runtime_error`, `std::invalid_argument` etc.)
- [ ] refactor to `ROS_ASSERT_MSG()`

## `ros_utils`

- [x] `ros_utils.h`
	- [x] single header
	- [x] `math.h`
	- [x] `std.h`
- [x] `geometry_msgs.h`
	- [x] `make_pose()`
	- [x] `read_pose()`
- [ ] `gazebo.h`
	- [ ] simulation
		- [ ] `set_simulation()`
	- [x] models and states
	- [ ] sensors
		- [ ] `projector`
		- [ ] `camera`
		- [ ] `camera_stereo`
		- [ ] `kinect`
- [x] `moveit.h`
	- [x] `make_mesh_cobj()`
	- [x] `get_gazebo_cobjs()`
	- [x] `move_base()` → `set_floating_jnt_pose()`
	- [ ] make `get_gazebo_cobjs()` generic with other shapes (e.g. `gazebo_ros_moveit_planning_scene` plugin) 
- [x] `eigen.h`
	- [x] `Eigen::make_tf(xyz, rpy)`
	- [x] `Eigen::make_tf(Pose)`
	- [x] `Eigen::make_tf(xyz, axis, angle)`
- [ ] `pcl.h`
	- [ ] `pcl::load_cloud(path)`
	- [ ] `pcl::show_cloud()` or `pcl::plot_cloud()`

## `ur5_ros`

#### `ur5_ros`

- [x] refactor all namespaces to `ur5::`
- [x] fix all header names (~~`ur5_description/ur5.h`~~ → `ur5_description/ur5_description.h`)
- [ ] include ur5_x everywhere properly (headers, package, cmake)
~
#### `ur5_description`

- [x] `has_ee()`
- [x] change to `ur5::`

#### `ur5_moveit_config`

- [x] run setup assistant
- [x] add `world_offset` virtual joint
- [x] use kinematic chain (instead of joints)
- [x] add `default.launch`
- [x] add `ur5_arm` as parent to `ur5_ee` end-effector group
- [ ] add TrajOpt
- [ ] ~~remove floating joint from planning group (maybe not possible)~~
- [ ] documentation (launch file, floating joint, planning etc.)

#### `ur5_planner`

- [x] re-define interface(s)
- [x] `moveit.h`
	- [x] refactor to new ur5:: interface (use `ur5::LINKS` etc.)
	- [x] make `update_planning_scene()` dynamic wrt. gripper/EE
	- [x] `start_scene_publisher()`
	- [x] proper `terminate()`
	- [x] `move_base()` using `set_floating_jnt_pose()` w/ recursive mutex
	- [x] `attach_object_to_ee()`
	- [x] `plan()`
	- [x] `plan_to_jnt_traj()`
	- [x] change to `JointTrajectoryPoint`
	- [x] add dynamic addition/removal of attached collision objects from Gazebo
	- [x] `set_planner_config()`
	- [x] `get_mutexed_planning_scene()`
	- [x] examples
	- [ ] add more planners (LazyPRM, SBL)
- [ ] `reachability.h`
	- [x] refactor to new ur5:: interface
	- [x] ReachabilityData
	- [x] grasp orientation lambdas (GRASP_SIDE_AT_TCP, GRASP_TOP_AT_TCP)
	- [x] examples

#### `ur5_dynamics`

- [ ] refactor to namespace (from static class)
- [ ] change to `ur5::` → update dependents

#### `ur5_controllers`

- [x] change to `ur5::`
- [x] `command()` (with `Eigen::Vector6d`)
- [x] `command_setpoint()`
- [x] `command_traj()`
- [ ] change to `trajectory_msgs` → `JointTrajectoryPoint` → remove `ur5_msgs`
- [ ] ~~cartesian controller (bridge)~~ → remove `qp_oases`?
- [ ] examples

#### `ur5_gazebo`

- [x] change to `ur5::`
- [x] `get_robot_state()` → `get_state()`
- [x] `get_gripper_state()` → `get_ee_state()`
- [x] `get_ee_given_obj_pose()`
- [ ] `get_tcp_given_obj_pose()` or `get_ee_given_obj_pose_at_tcp()` (needed?)
- [ ] add more pre-defined transforms e.g `w_T_b()`
- [ ] examples

#### `wsg50`

- [ ] ~~`get_state()`~~

## `rovi_system`

#### `rovi_system`

- [x] `.launch` file
- [ ] meta-data in `rovi_system/rovi_system.h` (e.g. table size)
- [x] fix `.setup.bash` file
- [ ] ~~interface, e.g. `rovi_system::get_camera_imgs()` + interface node for python (add to `.launch` file)~~
- [ ] workcell
	- [ ] add obstacles
	- [ ] add graspable objects
- [x] expriments framework
	- [x] file structure
	- [x] documentation
	- [x] rovi_system.m
	- [x] export_fig
	- [x] template w/ examples
	- [x] `get_experiment_dir()`
	- [x] `make_timestamped_data_dir()`
	- [x] `make_custom_data_dir()`
	- [ ] rovi_system.py (same interface as `rovi_system.h`)
- [ ] experiments:
	- [x] template
	- [ ] reachability + object pos in heatmap
	- [ ] point-to-point interpolation (lin + par)
	- [ ] planning (moveit → LazyPRM vs. SBL)
	- [ ] pick and place (integration)
	- [ ] pose estimation dense
	- [ ] pose estimation sparse

#### `rovi_planner`

- [x] define interface(s)
- [x] `traj_lin()` (cartesian)
- [ ] `traj_par()` (cartesian)
- [ ] `export_x()` methods
- [ ] ~~joint space interpolation (trajectory_msgs, template methods e.g. `traj_lin<TrajT>()`?)~~

#### `rovi_vision`

- [x] define interface
- [x] add pkgs
- [ ] define location for DNNTemplateMatching templates
- [ ] integrate Daniel's code → refactor to interface
- [ ] manage dependencies
- [ ] use `Eigen::Isometry3d`
- [ ] use `ros_utils`
- [ ] examples

#### `rovi_models`

- [x] `rovi_models` pkg
- [x] models + world (proper export)
- [ ] change default sensor topics (camera, projector etc.) from `rbrovi/camera` → `camera`
