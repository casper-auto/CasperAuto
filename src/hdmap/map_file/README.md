# map_file package

## points_map_filter

### feature
points_map_filter_node subscribe pointcloud maps and current pose, the node extract pointcloud near to the current pose.

#### subscribed topics
/points_map (sensor_msgs/PointCloud2)  : Raw pointcloud map. This topic usually comes from points_map_loader.  
/current_pose (geometry_msgs/PoseStamped) : Current pose of the car. This topic usually comes from pose_relay node.  

#### published topics
/points_map/filtered (sensor_msgs/PointCloud2) : Filtered pointcloud submap.  

#### parameters

For points_map_filter

| Param | Type | Default value | Options |
|-------|------|---------------|-----------|
| load_grid_size | double | 100.0 | grid size of submap. |
| load_trigger_distance | double | 20.0 | if the car moves load_trigger_distance(m), the map filter publish filtered submap. |

For points_map_loader

| Param | Type | Default value | Options |
|-------|------|---------------|-----------|
| points_map_loader/area | String | "noupdate" | "noupdate", "1x1", "3x3", "5x5", "7x7", "9x9" |
| points_map_loader/mode | String | "" | "", "download" |
| points_map_loader/pcd_paths | String array | [] | - |
| points_map_loader/arealist_path | String array | [] | - |

.pcd file search function is also implemented. Now you can specify multiple files or directories for pcd_paths.
If directories are specified, it automatically search files in it.

### how it works
map_filter_node relay /points_map topic until it recieves /current_pose topic.  
Then, the /current_pose topic recieved, the map_filter_node publish submap.

## demonstration
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/LpKIuI5b4DU/0.jpg)](http://www.youtube.com/watch?v=LpKIuI5b4DU)

## vector_map_loader

This node loads a vector map from csv files.

#### parameters

- `load_mode` - Set the mode of operation for the node. Possible values are:
    - "file" - Default operation mode, requires each csv file to be specified in args.
    - "directory" - Loads all csv files with vector map names in the directory specified by the `map_dir` parameter.
    - "download" - Downloads the vector map csvs from a webhost, use the args to specify
- `map_dir` - Specify the path to the directory containing vector map csv files. Only used in "directory" mode.
- `host` - Hostname of the webserver. Only used in "download" mode.
- `port` - Port of the webserver. Only used in "download" mode.
- `username` - Username. Only used in "download" mode.
- `password` - Password. Only used in "download" mode.

## lanelet2_map_loader
### Feature
lanelet2_map_loader loads Lanelet2 file and publish the map data as autoware_lanelet2_msgs/MapBin message.
The node projects lan/lon coordinates into MGRS coordinates.

### How to run
Run from CLI:
`rosrun map_file lanelet2_map_loader path/to/map.osm`

### Published Topic
/lanelet_map_bin (autoware_lanelet2_msgs/MapBin) : Binary data of loaded Lanelet2 Map.

## lanelet2_map_visualization
### Feature
lanelet2_map_visualization visualizes autoware_lanelet2_msgs/MapBin messages into visualization_msgs/MarkerArray.

### How to run
Run from CLI:
`rosrun map_file lanelet2_map_visualization`

### Subscribed Topics
/lanelet_map_bin (autoware_lanelet2_msgs/MapBin) : binary data of Lanelet2 Map

### Published Topics
/lanelet2_map_viz (visualization_msgs/MarkerArray) : visualization messages for RVIZ
