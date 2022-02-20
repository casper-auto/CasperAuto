# Carla Route Planner

The node `carla_route_planner` makes this feature available in the ROS context.

It uses the current pose of the ego vehicle with role-name "ego_vehicle" as starting point. If the
vehicle is respawned, the route is newly calculated.

## Startup

To run it:

```
roslaunch carla_route_planner carla_route_planner.launch
```
