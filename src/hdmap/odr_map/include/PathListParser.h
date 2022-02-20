//
// Created by sen on 20/02/17.
//

#ifndef PATH_LIST_PARSER_H
#define PATH_LIST_PARSER_H

#include <string>
#include <fstream>
#include <ios>
#include <pugixml/pugixml.hpp>
#include <unordered_set>
#include "ros/ros.h"
#include "casper_auto_msgs/MapBasepath.h"

using UnorderedIdSet = std::unordered_set<uint32_t>;

class PathListParser {
public:
    static UnorderedIdSet Load(const std::string& path);

    static std::vector<casper_auto_msgs::MapBasepath> LoadPathConnection(const std::string& path);

private:
    static void Parse(const pugi::xml_document& xml,
                      UnorderedIdSet& unorderedIdSet);

    static void Parse(const pugi::xml_document& xml,
                      std::vector<casper_auto_msgs::MapBasepath>& path_connection_list);
};

#endif // PATH_LIST_PARSER_H
