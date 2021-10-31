## `ros_utils`

- [x] `geometry_msgs.h`
- [ ] `gazebo.h`
  - [ ] simulation
  - [ ] models and states
  - [ ] computer vision
- [ ] `moveit.h`

## `ur5_ros`

#### `ur5_ros`

- [x] refactor all namespaces to `ur5::`
- [x] fix all header names (~~`ur5_description/ur5.h`~~ → `ur5_description/ur5_description.h`)

#### `ur5_description`

- [x] `has_ee()`
- [x] change to `ur5::`

#### `ur5_planner`

- [ ] define interface(s)
- [ ] `moveit.h`
	- [ ] use `ur5::LINKS` etc.
	- [ ] make `update_planning_scene()` dynamic wrt. gripper/EE
- [ ] `reachability.h`

#### `ur5_dynamics`

- [ ] refactor to namespace (from static class)
- [ ] change to `ur5::` → update dependents

#### `ur5_controllers`

- [x] change to `ur5::`
- [ ] cartesian controller (bridge)
- [ ] change to trajectory_msgs?
- [ ] `exec_traj()`
- [ ] examples

#### `ur5_gazebo`

- [ ] change to `ur5::`
- [ ] `get_robot_state()` → `get_state()`
- [ ] `get_gripper_state()` → `get_ee_state()`
- [ ] `get_ee_given_pos()` + `get_tcp_given_pos()`
- [ ] add more pre-defined transforms e.g `w_T_b()`
- [ ] examples

#### `wsg50`

- [ ] `get_state()`

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
- [ ] `traj_lin()` + `traj_par()` (cartesian)
- [ ] `export_x()` methods
- [ ] examples
- [ ] joint space interpolation (trajectory_msgs?)
- [ ] templated methods: `traj_lin<TrajT>()` + `traj_par<TrajT>()` (KDL sucks at joint interpolation)

#### `rovi_vision`

- [ ] define interface e.g. `rovi_vision::m1::get_pose("object")`
- [ ] switch to using `Eigen::Isometry3d`
- [ ] examples

#### `rovi_models`

- [x] `rovi_models` pkg
- [x] models + world (proper export)
