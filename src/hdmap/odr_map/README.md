## Environment
- Cmake: 3.5.1
- gcc/g++: 5.5
- ROS: kinetic
- jsk-rviz-plugins

## Preparation
1. Put "casper_auto_msgs" folder in the catkin_workspace
```
$ mv casper_auto_msgs ${CATKIN_WS}/src
```

2. Put "odr_map" folder in the catkin_workspace
```
$ mv odr_map ${CATKIN_WS}/src
```

3. If you want to change the map file, change the parameter in the launch file.
```
# Put the xodr file in the data folder.
cd ${CATKIN_WS}/src/odr_map/data/
cp [your xodr file] ./

# Please change the arg parameter
$ vi service_node.launch

<arg name="mode" default="other"/> ←　Set the mode parameter to "other".
<arg name="xodr_file" default="test_left_hand_map.xodr"/>  ← Change to the your xodr file name.

```

## How to service call
1. Run service node
```
$ cd ${CATKIN_WS}
$ catki_make
$ source devel/setup.bash
$ roslaunch odr_map service_node.launch
```

2. In the program you want to use MapObject, describe the service call part with reference to the following
```
    // Set client for service call
    ros::ServiceClient map_api_client = nh.serviceClient<odr_map::GetMapObject>("getAllMapObjects");
    // Define service call object
    odr_map::GetMapObject get_map_object;
    // Set request parameters.
    // For example, get map information within 100 meters of point (0, 0, 0)
    get_map_object.request.x = 0.0;
    get_map_object.request.y = 0.0;
    get_map_object.request.z = 0.0;
    get_map_object.request.distance = 100.0;
    get_map_object.request.step = 1.0; // Step of points sequences.

    // Get MapObject with Service call
    odr_map::MapObject map_object;
    if (map_api_client.call(get_map_object)) {
        map_object = get_map_object.response.map_objects;
    } else {
        ROS_ERROR("Failed to call service \"map_api_server\"");
        return 1;
    }

```

## How to running the Viewer

1. Run viewer
```
$ cd ${CATKIN_WS}
$ catkin_make
$ souce devel/setup.bash
$ roslaunch odr_map test_node.launch

Please click "Path Visualiser" button in the RVIZ widow menu ber.
And mouce over the path in the RVIZ view. You can see the detail information of path.
```
