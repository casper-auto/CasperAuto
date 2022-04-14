# Speed Profile Planner

Given a predefined path, generate a real-time speed profile with using speed_planner_lib.

## speed_planner_lib

Given a reference line or a series of path points, the program is to generate the speed profile for the path.

- optimizer: ipopt

## Install Dependencies

Try ```./install-ubuntu-MPC.sh```. This will install all dependencies.

The individual steps have been included for reference.  Please note that for any particular command, including execution of ```.sh``` scripts, it may be necessary to add ```sudo``` prior to the command.  It is also a good practice to run ```sudo apt-get update``` prior to installation of new libraries. The main dependencies are Ipopt and CppAD which will be installed by executing ```./install-ubuntu-MPC.sh```.

## Documentation and API

```bash
google-chrome build/doc/html/index.html
```
