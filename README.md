# ROVI System
Robotics and computer vision system for pick and place tasks with ROS/Gazebo/MoveIt.

The project consists of the following packages:

- [`rovi_system`](/rovi_system) - Main project package with examples and tests
- [`rovi_models`](/rovi_models)
- [`rovi_planner`](/rovi_planner)
- [`rovi_vision`](/rovi_vision)
- [`ur5_ros`](/ur5_ros) - Integration of UR5 robot into ROS/Gazebo environment
- [`ros_utils`](/ros_utils) - Collection of modern utilities for the ROS/Gazebo workflow
- [`qp_oases`](/qp_oases)

For more information, please refer to the README of a specific package.

## Getting started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Dependencies

The project is tested on Ubuntu `20.04.3 LTS`. Built using `catkin build` with `CMake 3.4` and `gcc 9.3.0-17`, mainly depending on:

* [ROS (noetic)][ros] - framework for robot operation
* [Gazebo][gazebo] - robot simulation environment
* [rosdep] - management of ROS system dependecies
* [vcstool] - automated workspace configuration

### Installation

Installation of the project:

<details>
<summary><strong>Installing ROS and dependencies</strong></summary></br>

Install the Desktop-Full Install of ROS as per [their documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) for Ubuntu 20.04 - make sure to install `rosdep`.

Then, install `vcstool` by running ([reference](https://github.com/dirk-thomas/vcstool#how-to-install-vcstool)):

```
sudo apt install python3-vcstool
```

</details>


<details>
<summary><strong>Installing project (workspace)</strong></summary></br>

Make sure you have Git SSH configured properly as per [this guide](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh).

Download the [`setup.bash`](https://github.com/martinandrovich/rovi_system/raw/main/setup.bash) file (← right click and save as) to where the workspace should be created. Open a terminal, navigate to where the file is located, make it executable by running `chmod +x setup.bash` and execute the file in the current shell as `source setup.bash`.

This will create a catkin workspace and download all necessary packages and dependencies. See [Usage](#usage) for how to use the project.

</details>

## Usage

### Running the project

Navigate to the catkin workspace (`rovi_ws`) and source the environment variables by running `source devel/setup.bash`. If installed using the `setup.bash` script, you can simply run `rovi_ws` in the terminal to automatically source the workspace and navigate to its directory.

You can then launch the workcell by running:

```
roslaunch rovi_system workcell.launch
```

The workcell launch file can be configured using several arguments, for example:

```
roslaunch rovi_system workcell.launch ee:=wsg50 controller:=ur5_joint_position_controller
```

An overview of the arguments is located in the [`workcell.launch`](rovi_system/launch/workcell.launch) file.

## License

No license has been decided yet.

<!-- LINKS -->

[semver]: http://semver.org/
[releases]: about:blank
[changelog]: CHANGELOG.md
[wiki]: about:blank

[ros]: http://wiki.ros.org/noetic
[gazebo]: http://gazebosim.org
[rosdep]: https://wiki.ros.org/rosdep
[vcstool]: https://github.com/dirk-thomas/vcstool

[pkg-project_foo]: /project_foo

[androvich-git]: https://github.com/martinandrovich
[robognome-git]: https://github.com/RoboGnome
