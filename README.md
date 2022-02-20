# Casper Auto

TODO: Description.

## Folder Structure

## Quick Setup

Under Ubuntu 20.04 and ROS Noetic (with Python 3.8 kernel in default),

1. Download Carla simulator (version 0.9.11 release)[https://github.com/carla-simulator/carla/releases/tag/0.9.11].

2. Setup environment variables for Carla.

Include CARLA Python API to the Python path:

```
export CARLA_ROOT=/path/to/your/carla/installation
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/agents
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg
```

To setup Carla Simulator using Docker please refer to ["Setup_Carla_Simulator"](./docs/Setup_Carla_Simulator.md).

3. Setup the workspace

For your convenience, a quick setup script is provided. Simply run

```
./setup_workspace.sh
```

This script will create the catkin_ws folder, create a symbolic link to the source folder `src`, and build everything with `catkin build`.

> Unlike `catkin_make`, the `catkin` command-line tool is not just a thin wrapper around a the `cmake` and `make` commands. The `catkin build` command builds each package in a workspace’s source space in isolation in order to prevent build-time cross-talk. As such, in its simplest use, `catkin build` behaves similarly to a parallelized version of `catkin_make_isolated`.

## Demo

**TODO**
