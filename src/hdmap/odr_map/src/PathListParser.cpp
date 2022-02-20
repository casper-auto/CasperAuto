#include "PathListParser.h"

UnorderedIdSet PathListParser::Load(const std::string& path) {
    // borrowed from the constructor of odr_map
    std::ifstream ifs(path);
    if (ifs.fail()) {
        std::string error_message("Error: Freeway path list xml file not found. File path : ");
        error_message = error_message + path;
        std::cerr << error_message << std::endl;
        throw std::invalid_argument(error_message);
    }

    std::string xml_str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    UnorderedIdSet unorderedIdSet;
    // borrowed from OpenDriveParser::Load function
    pugi::xml_document xml;
    pugi::xml_parse_result parse_result = xml.load_string(xml_str.c_str());
    if (!parse_result) {
        ROS_ERROR("Cannot parse the freeway path list xml file.");
    } else {
        // parse xml and fill in UnorderedIdSet object
        Parse(xml, unorderedIdSet);
    }
    return unorderedIdSet;
}

std::vector<casper_auto_msgs::MapBasepath> PathListParser::LoadPathConnection(const std::string& path) {
    // borrowed from the constructor of odr_map
    std::ifstream ifs(path);
    if (ifs.fail()) {
        std::string error_message("Error: Path connection xml file not found. File path : ");
        error_message = error_message + path;
        std::cerr << error_message << std::endl;
        throw std::invalid_argument(error_message);
    }

    std::string xml_str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    std::vector<casper_auto_msgs::MapBasepath> path_connection_list;
    // borrowed from OpenDriveParser::Load function
    pugi::xml_document xml;
    pugi::xml_parse_result parse_result = xml.load_string(xml_str.c_str());
    if (!parse_result) {
        ROS_ERROR("Cannot parse Path connection xml file.");
    } else {
        // parse xml and fill in UnorderedIdSet object
        Parse(xml, path_connection_list);
    }
    return path_connection_list;
}

void PathListParser::Parse(const pugi::xml_document& xml,
                           UnorderedIdSet& unorderedIdSet) {
    for (pugi::xml_node node_path : xml.child("root").children("road")) {
        auto road_id = node_path.attribute("road_id").as_uint();
        unorderedIdSet.insert(road_id);
    }
}

void PathListParser::Parse(const pugi::xml_document& xml,
                           std::vector<casper_auto_msgs::MapBasepath>& path_connection_list) {

    if (xml.child("basepath").empty()) return;

    for (pugi::xml_node node_path : xml.child("basepath").children("path")) {
       if( !node_path.attribute("unique_id").empty() ){
           casper_auto_msgs::MapBasepath path_connection;
           path_connection.id.unique_id = node_path.attribute("unique_id").as_int();

           if ( !node_path.attribute("left").empty() ){
               path_connection.left_idxes.push_back(node_path.attribute("left").as_uint());
           }

           if ( !node_path.attribute("right").empty() ){
               path_connection.right_idxes.push_back(node_path.attribute("right").as_uint());
           }

           path_connection_list.push_back(path_connection);
       }
    }
}
