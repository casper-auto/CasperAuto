# Carla Route Planner

The node `carla_tf_light` makes this feature available in the ROS context.

It uses the current pose of the ego vehicle with role-name "ego_vehicle" as starting point. If the
vehicle is respawned, the route is newly calculated.

## Startup

To run it:

```
roslaunch carla_tf_light tf_light.launch
```
