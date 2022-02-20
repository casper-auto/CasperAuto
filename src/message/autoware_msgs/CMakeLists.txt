cmake_minimum_required(VERSION 2.8.3)
project(autoware_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  jsk_recognition_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    AccelCmd.msg
    AdjustXY.msg
    BrakeCmd.msg
    Centroids.msg
    CloudCluster.msg
    CloudClusterArray.msg
    ColorSet.msg
    ControlCommand.msg
    ControlCommandStamped.msg
    DTLane.msg
    DetectedObject.msg
    DetectedObjectArray.msg
    ExtractedPosition.msg
    Gear.msg
    GeometricRectangle.msg
    ICPStat.msg
    ImageLaneObjects.msg
    ImageObj.msg
    ImageObjRanged.msg
    ImageObjTracked.msg
    ImageObjects.msg
    ImageRect.msg
    ImageRectRanged.msg
    IndicatorCmd.msg
    LampCmd.msg
    Lane.msg
    LaneArray.msg
    NDTStat.msg
    ObjLabel.msg
    ObjPose.msg
    PointsImage.msg
    ProjectionMatrix.msg
    RemoteCmd.msg
    ScanImage.msg
    Signals.msg
    State.msg
    StateCmd.msg
    SteerCmd.msg
    SyncTimeDiff.msg
    SyncTimeMonitor.msg
    TrafficLight.msg
    TrafficLightResult.msg
    TrafficLightResultArray.msg
    TunedResult.msg
    ValueSet.msg
    VehicleCmd.msg
    VehicleLocation.msg
    VehicleStatus.msg
    VscanTracked.msg
    VscanTrackedArray.msg
    Waypoint.msg
    WaypointState.msg
)

add_service_files(
  FILES RecognizeLightState.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    jsk_recognition_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    jsk_recognition_msgs
    message_runtime
    sensor_msgs
    std_msgs
)
