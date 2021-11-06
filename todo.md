## general

- [ ] update author/maintainer (email + order)
- [ ] add `README.md` to each pkg to match description of `package.xml`
- [ ] update dependencies of each package (`package.xml` + header files)

## `ros_utils`

- [x] `geometry_msgs.h`
- [ ] `gazebo.h`
	- [ ] simulation
	- [x] models and states
	- [ ] ~~computer vision~~
- [x] `moveit.h`
	- [x] `make_mesh_cobj()`
	- [x] `get_gazebo_cobjs()`
	- [x] `move_base()` → `set_floating_jnt_pose()`
- [ ] `eigen.h`
	- [ ] `Eigen::make_tf(xyz, rpy)`
	- [ ] `Eigen::make_tf(Pose)`

## `ur5_ros`

#### `ur5_ros`

- [x] refactor all namespaces to `ur5::`
- [x] fix all header names (~~`ur5_description/ur5.h`~~ → `ur5_description/ur5_description.h`)

#### `ur5_description`

- [x] `has_ee()`
- [x] change to `ur5::`

#### `ur5_moveit_config`

- [x] run setup assistant
- [x] add `world_offset` virtual joint
- [x] use kinematic chain (instead of joints)
- [x] add `default.launch`
- [ ] remove floating joint from planning group*
- [ ] documentation (launch file, floating joint, planning etc.)

#### `ur5_planner`

- [ ] define interface(s)
- [ ] `moveit.h`
	- [ ] refactor to new ur5:: interface (use `ur5::LINKS` etc.)
	- [x] make `update_planning_scene()` dynamic wrt. gripper/EE
	- [x] `start_scene_publisher()`
	- [x] proper `terminate()`
	- [ ] `move_base()` using `move_virtual_joint()` w/ recursive mutex
	- [ ] `attach_object_to_ee()`
	- [ ] `plan()`
	- [ ] `plan_to_jnt_traj()`
	- [ ] examples
- [ ] `reachability.h`
	- [ ] refactor to new ur5:: interface (use `ur5::LINKS` etc.)
	- [ ] examples

#### `ur5_dynamics`

- [ ] refactor to namespace (from static class)
- [ ] change to `ur5::` → update dependents

#### `ur5_controllers`

- [x] change to `ur5::`
- [ ] `exec_traj()`
- [ ] cartesian controller (bridge)
- [ ] change to trajectory_msgs?
- [ ] examples

#### `ur5_gazebo`

- [x] change to `ur5::`
- [x] `get_robot_state()` → `get_state()`
- [x] `get_gripper_state()` → `get_ee_state()`
- [ ] `get_ee_given_pos()` + `get_tcp_given_pos()`
- [ ] add more pre-defined transforms e.g `w_T_b()`
- [ ] examples

#### `wsg50`

- [ ] `get_state()`*

## `rovi_system`

#### `rovi_system`

- [x] `.launch` file
- [ ] meta-data in `rovi_system/rovi_system.h` (e.g. table size)
- [ ] interface, e.g. `rovi_system::get_camera_imgs()`
- [ ] interface node for python (add to `.launch` file)
- [ ] `get_experiment_dir()` + `get_data_dir(experiment_dir)`
- [ ] experiments / tests + data export

#### `rovi_planner`

- [x] define interface(s)
- [x] `traj_lin()` (cartesian)
- [ ] `traj_par()` (cartesian)
- [ ] `export_x()` methods
- [ ] joint space interpolation* (trajectory_msgs, template methods e.g. `traj_lin<TrajT>()`?)

#### `rovi_vision`

- [ ] define interface(s) e.g. `rovi_vision::m1::get_pose("object")`
- [ ] switch to using `Eigen::Isometry3d`
- [ ] examples

#### `rovi_models`

- [x] `rovi_models` pkg
- [x] models + world (proper export)
