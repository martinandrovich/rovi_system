# Experiments in ROVI system

Experiments in the `rovi_system` package are contained within the `rovi_system/tests/` directory. An experiment of `<name>` is comprised of:

- `<name>.cpp` or `<name>.py` (ROS node named `test_<name>`)
- `<name>.m` (manipulation/plotting using `export_fig` in MATLAB)
- `<name>/data/` directory with time-stamped trials, e.g. `<name>/data/20210105_000322/`
- `test_<name>` executable (automatically added in `rovi_system/CMakeLists.txt`)

It is important that the naming scheme above is followed; otherwise CMake will fail. Use `rovi_system.h` to access experiment directories, data directories for a particular experiment, and so forth. The `rovi_system/scripts/rovi_system.m` file provides necessary includes for plotting etc. in MATLAB.

See the files in the `template` experiment for example code.