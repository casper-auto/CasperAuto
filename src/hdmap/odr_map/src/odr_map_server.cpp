#include <map>
#include "boost/format.hpp"
#include "ros/ros.h"
#include "odr_map.h"
#include "odr_map/GetMapObject.h"
#include "odr_map/GetMapGeoLocation.h"
#include "odr_map/GetMapPolygons.h"

#include <std_msgs/String.h>

// TODO: class
std::shared_ptr<OdrMap> g_odr_map = nullptr;

bool getAllMapObjects(
        odr_map::GetMapObject::Request& request,
        odr_map::GetMapObject::Response& response) {

    response.map_objects = g_odr_map->GetAllMapObjects();

    return true;
}

bool getMapObjects(
        odr_map::GetMapObject::Request& request,
        odr_map::GetMapObject::Response& response) {
    if (request.distance <= 0) {
        ROS_ERROR("Search distance must be larger than 0!");
        return false;
    }

    response.map_objects = g_odr_map->GetMapObject(request.x, request.y, request.z, request.distance);

    return true;
}

bool getMapGeoLocation(
        odr_map::GetMapGeoLocationRequest& request,
        odr_map::GetMapGeoLocationResponse& response) {

    auto geo_location = g_odr_map->GetMapGeoLocation();
    response.latitude = geo_location.latitude;
    response.longitude = geo_location.longitude;
    response.altitude = geo_location.altitude;

    return true;
}

bool getMapPolygons(odr_map::GetMapPolygons::Request& request,
                    odr_map::GetMapPolygons::Response& response) {
    if (request.distance <= 0) {
        ROS_ERROR("Search distance must be larger than 0!");
        return false;
    }
    auto road_id_set = g_odr_map->GetTargetRoadSetByEuclideanDistance(request.x,
                                                                      request.y,
                                                                      request.z,
                                                                      request.distance);
    g_odr_map->AddPolygonsWithinTargetRoads(road_id_set,
                                            response.boundaries,
                                            response.sidewalks,
                                            response.crosswalks);
    return true;
}


// #define DEBUG
int main(int argc, char** argv) {
    ros::init(argc, argv, "odr_map_server");
    ros::NodeHandle nh;

    std::string xml_str;
    double step;

    nh.getParam("/map_api_server/step", step);

    boost::shared_ptr<std_msgs::String const> shared_map_msg = ros::topic::waitForMessage<std_msgs::String>("/carla/ego_vehicle/opendrive_map", nh);
    xml_str = shared_map_msg->data;

    g_odr_map = std::make_shared<OdrMap>(xml_str, step);

    ros::ServiceServer map_object_service = nh.advertiseService("getMapObjects", getMapObjects);
    ros::ServiceServer all_map_object_service = nh.advertiseService("getAllMapObjects", getAllMapObjects);
    ros::ServiceServer geo_location_service = nh.advertiseService("getMapGeoLocation", getMapGeoLocation);
    ros::ServiceServer map_polygon_service = nh.advertiseService("getMapPolygons", getMapPolygons);

    ros::spin();

    return EXIT_SUCCESS;
}
