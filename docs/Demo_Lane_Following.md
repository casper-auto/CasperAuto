# Carla Lane Following Demo

## Instruction

Step 1: Start Carla Simulator

```
# under carla simulator root folder:
./CarlaUE4.sh -windowed -ResX=320 -ResY=240
```

Step 2: Launch ROS Bridge

```
source catkin_ws/devel/setup.bash
roslaunch demo_entrance carla_ros_bridge_with_ego_vehicle.launch
```

Step 3: Launch lane following demo

```
source catkin_ws/devel/setup.bash
roslaunch demo_entrance lane_following_demo.launch
```

## Demo Video

[![](./carla-lane-following-demo.png)](https://youtu.be/qKbhPBSRioA)
