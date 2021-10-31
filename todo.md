## `ros_utils`

- [ ] `geometry_msgs.h`
- [ ] `gazebo.h`
  - [ ] simulation
  - [ ] models and states
  - [ ] computer vision
- [ ] `moveit.h`

## `ur5_ros`

#### `ur5_ros`

- [ ] refactor all namespaces to `ur5::`
- [ ] fix header names (~~`ur5_description/ur5.h`~~ → `ur5_description/ur5_description.h`)

#### `ur5_description`

- [ ] `has_ee()`

#### `ur5_planner`

- [ ] define interface(s)
- [ ] `moveit.h`
	- [ ] use `ur5::LINKS` etc.
	- [ ] make `update_planning_scene()` dynamic wrt. gripper/EE
- [ ] `reachability.h`

#### `ur5_controllers`

- [ ] cartesian controller (bridge)
- [ ] change to trajectory_msgs?
- [ ] `exec_traj()`
- [ ] examples

#### `ur5_gazebo`

- [ ] `get_gripper_state()` → `get_ee_state()`
- [ ] remove `w_T_b()`
- [ ] examples

#### `wsg50`

- [ ] `get_state()`

## `rovi_system`

#### `rovi_system`

- [x] `.launch` file
- [ ] meta-data in `rovi_system/rovi_system.h` (e.g. table size)
- [ ] interface, e.g. `rovi_system::get_camera_imgs()`
- [ ] interface node for python (add to `.launch` file)
- [ ] experiments / tests + data export

#### `rovi_planner`

- [ ] define interface(s)
- [ ] `traj_lin()` + `traj_par()` (cartesian)
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
