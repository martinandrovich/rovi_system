# Experiments in ROVI system

Experiments in `rovi_system` are contained in the `rovi_system/tests/` directory. An experiment `<name>` can be implemented in either C++ or Python and is structured as:

```
rovi_system/tests/                 # directory for all experiments in rovi_system
|
└── <name>/                        # experiment directory
    |
    ├── data/                      # directory with time-stamped trials
    |   ├── 20210105_000322/      
    |   └── ...
    |
    ├── test_<name>.cpp            # source code for experiment (ROS node named test_<name>)
    ├── test_<name>.py             # python code for experiment
    ├── test_<name>.launch         # launch file for experiment
    ├── test_<name>.m              # MATLAB code for data manipulation/plotting using export_fig
    └── README.md                  # documentation of experiment*
```

A C++ experiment is automatically added as an executable ROS node named `test_<name>` (by `rovi_system/CMakeLists.txt`). It can be executed using `rosrun rovi_system test_<name>` or using `roslaunch rovi_system test_<name>.launch` (if provided). If using Python, remember to make `test_<name>.py` executable with `chmod`.

It is important that the naming scheme above is followed; otherwise CMake will fail. Use `rovi_system.h` to access experiment directories, data directories for a particular experiment, and so forth. The `rovi_system/scripts/rovi_system.m` file provides necessary includes for plotting etc. in MATLAB.

See the files in the `template` experiment for example code.
