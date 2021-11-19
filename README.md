# ROVI System
Robotics and computer vision system for pick and place tasks with ROS/Gazebo/MoveIt.

The project consists of the following packages:

| Package                                                     | Description                                                                    |
|-------------------------------------------------------------|--------------------------------------------------------------------------------|
| [`rovi_system`](/rovi_system)                               | Main project package with examples and tests                                   |
| [`rovi_models`](/rovi_models)                               | Gazebo models and worlds for the the ROVI workcell                             |
| [`rovi_planner`](/rovi_planner)                             | Interpolation-based trajectory generation                                      |
| [`rovi_vision`](/rovi_vision)                               | Computer vision-based pose estimation methods for objects in the ROVI workcell |
| [`ur5_ros`](https://github.com/martinandrovich/ur5_ros)     | Integration of UR5 robot into ROS/Gazebo/MoveIt environment                    |
| [`ros_utils`](https://github.com/martinandrovich/ros_utils) | Collection of modern utilities for the ROS/Gazebo/MoveIt workflow              |
| [`qp_oases`](https://github.com/dscho15/qp_oases)           | Port of qpOASES library to ROS                                                 |

For more information, please refer to the `README.md` of a specific package.

## Getting started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Dependencies

The project is tested on Ubuntu `20.04.3 LTS`. Built using [`catkin_tools`][catkin_tools] with `CMake 3.4` and `gcc 9.3.0-17`, mainly depending on:

* [ROS (noetic)][ros] - framework for robot operation
* [Gazebo][gazebo] - robot simulation environment
* [rosdep] - management of ROS system dependecies
* [vcstool] - automated workspace configuration
* [catkin_tools] - command line tools for working with catkin workspaces

### Installation

Installation of the project:

<details>
<summary><strong>Installing ROS and dependencies</strong></summary></br>

1) Install ROS (Desktop-Full Install) and `rosdep` ([guide](http://wiki.ros.org/noetic/Installation/Ubuntu))
2) Install `vcstool` ([guide](https://github.com/dirk-thomas/vcstool#how-to-install-vcstool))
```
sudo apt install python3-vcstool
```
3) Install `catkin_tools` ([guide](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get))
```
sudo apt install python3-catkin-tools
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

### Experiments

Experiments in `rovi_system` are contained in the `rovi_system/tests/` directory. An experiment of `<name>` can be implemented in either C++ or Python, and is structured as:

<details>
<summary><strong>Structure of an experiment in <code>rovi_system</code></strong></summary></br>

```
rovi_system/tests/                 # directory for all experiments in rovi_system
|
└── <name>/                        # experiment directory
    |
    ├── img/                       # exported plots
    ├── data/                      # directory with time-stamped trials
    |   ├── 20210105_000322/      
    |   └── ...
    |
    ├── test_<name>.cpp            # source code for experiment (ROS node named test_<name>)
    ├── test_<name>.py             # python code for experiment
    ├── test_<name>.launch         # launch file for experiment
    ├── test_<name>.m              # MATLAB code for data manipulation/plotting using export_fig
    └── README.md                  # documentation of experiment
```
</details>

A C++ experiment is automatically added as a ROS node named `test_<name>` (by `rovi_system/CMakeLists.txt`) and can be launched using `rosrun` or `roslaunch` (if provided). Use `rovi_system.h` and `scripts/rovi_system.m` for helper functions (get experiment/data/img directory, plotting etc.) - see the [`template` experiment](/rovi_system/tests/template) for example code.

### Working in VS Code

Since IntelliSense is utter trash for larger projects, it is recommended to use the [`clangd` extension](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) as the language server, together with [`catkin-tools-clangd`](https://pypi.org/project/catkin-tools-clangd/) python package to generate the `compile_commands.json` for `clangd`. The [`ROS` extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) is also a nice addition when working in VS Code.

### Working on the project (compile, commit etc.)

From the root of the workspace (i.e. after running `rovi_ws`), run `catkin build` (or `catkin build_compile_cmd` if using `clagngd` extension) to build the workspace. The coding conventions are defined by the `.clangformat` (TODO), summarized as:

- Indent with tabs, align with spaces
- Comments in lower-case, add URLs to external resources
- Consistent interfaces accross the project (e.g. args and return values)
- Always review the code and examples of a package before adding new code
- Examples of methods/classes etc. is a must (in `/examples`)
- Commit in blocks of relevant code with short and descriptive messages (typically all lower-case)
- Proper includes, cmake and package manifest
- Segregate code properly in packages; generic utilities go in `ros_utils` pkg
- [ROS Best Practices](https://github.com/leggedrobotics/ros_best_practices/wiki)
- ~~Look at `rovi_system/examples/code_conventions.cpp` (TODO) for inspiration~~
- ~~Add any bugs/issues/todos to GitHub Issues~~

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
[catkin_tools]: https://catkin-tools.readthedocs.io

[pkg-project_foo]: /project_foo

[androvich-git]: https://github.com/martinandrovich
[robognome-git]: https://github.com/RoboGnome
