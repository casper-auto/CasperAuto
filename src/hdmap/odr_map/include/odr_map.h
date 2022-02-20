#ifndef ODR_MAP
#define ODR_MAP

#include "carla/client/Map.h"
#include "carla/road/element/Waypoint.h"
#include "carla/opendrive/OpenDriveParser.h"
#include "carla/client/Waypoint.h"
#include "carla/Memory.h"
#include "carla/road/Road.h"
#include "carla/road/RoadTypes.h"
#include "carla/road/element/LaneMarking.h"
#include "carla/geom/GeoLocation.h"
#include "rpc/msgpack/adaptor/vector.hpp"

#include "casper_auto_msgs/MapBasepath.h"
#include "casper_auto_msgs/MapLane.h"
#include "casper_auto_msgs/MapBoundary.h"
#include "casper_auto_msgs/MapTrafficLight.h"
#include "casper_auto_msgs/MapCrosswalk.h"
#include "casper_auto_msgs/MapStopLine.h"
#include "casper_auto_msgs/LaneInformation.h"
#include "casper_auto_msgs/MapObject.h"
#include "casper_auto_msgs/BoundaryPolygons.h"

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"

#include "PathListParser.h"

using namespace carla::road;
using namespace carla::road::element;
using namespace carla::geom;
using namespace geometry_msgs;

// constants
constexpr double ICOURSE_ZOFFSET = 140.0;
constexpr double DISTANCE_EPSILON = 0.1;
constexpr double EPSILON = 100.0 * std::numeric_limits<double>::epsilon();
constexpr double LANE_DISTANCE = 200.0;
constexpr double BUSH_T_INTERVAL = 1.0;
constexpr double BUSH_S_INTERVAL = 3.2;

// common used alias declarations
using PointVector = std::vector<Point32>;
using PolygonVector = std::vector<geometry_msgs::Polygon>;
using DoublePair = std::pair<double, double>;
using MapBasePathVector = std::vector<casper_auto_msgs::MapBasepath>;
using MapLaneVector = std::vector<casper_auto_msgs::MapLane>;
using RoadMarkTypeColorPair = std::pair<LaneMarking::Type, LaneMarking::Color>;
using UniqueIdSet = std::unordered_set<uint32_t>;
using WaypointSharedPtr = carla::SharedPtr<carla::client::Waypoint>;

// indexMap types
using RoadIdUniqueIdsMap = std::unordered_map<RoadId, std::vector<uint32_t>>;
using RoadIdSectionIdLaneIdUniqueIdMap = std::unordered_map<RoadId,
        std::map<SectionId, std::map<LaneId, uint32_t>>>;

// used by boundary polygon, sidewalk polygon
using BoundaryPolygonPointsMap = std::map<SectionId, std::map<LaneId, PointVector>>;
using SidewalkPolygonPointsMap = std::map<SectionId,
        std::map<LaneId, std::pair<PointVector, PointVector>>>;

// used by boundary, lane.
using RoadMarkStartSEndSVectorPair = std::pair<
        std::pair<SectionId, LaneId>, std::map<int, DoublePair>>; // int road mark id
using RoadMarkStartSEndSInfos = std::vector<RoadMarkStartSEndSVectorPair>;
using RoadMarkIdTypeColorMap = std::unordered_map<int, RoadMarkTypeColorPair>;

// used by base path
using LaneStartSEndSPair = std::pair<std::pair<SectionId, LaneId>, DoublePair>;
using LaneStartSEndSInfos = std::vector<LaneStartSEndSPair>;
//using LaneIdTurnRelationMap = std::unordered_map<LaneId, userData::VectorSignal::TurnRelation>;

class OdrMap {

public:
    enum class PointPosition : uint8_t {
        Center = 0u,
        Outside = 1u
    };

    enum class SearchDirection : uint8_t {
        Forward = 0u,
        Backward = 1u
    };

    enum class RoadMarkType : uint8_t {
        White = 0u, // used by MapLane
        Curb = 1u // used by MapBoundary
    };

    enum class MapLanePosition : uint8_t {
        NotRelated = 0u, // 0：
        Related = 1u, // 1：
        EgoLeft = 2u, // 2：
        EgoRight = 3u // 3：
    };

public:
    struct STLength {
        double s;
        double t;
        double length;

        STLength(double s, double t, double length)
                : s(s), t(t), length(length){}
    };

public:
    OdrMap(const std::string& xml_str, double step);

    ~OdrMap() = default;

    Road& getRoad(RoadId road_id);

    WaypointSharedPtr getWaypoint(float x, float y, float z);

    bool isInRoad(float x, float y, float z);

    bool isInJunction(float x, float y, float z);

    std::vector<WaypointSharedPtr> GenerateWaypoints(double distance) const;

    MapLaneVector GetMapLanes(const Road& road, double step);

    std::vector<casper_auto_msgs::MapBoundary> GetMapBoundaries(const Road& road, double step);

    const casper_auto_msgs::MapObject &GetAllMapObjects() const;

    casper_auto_msgs::MapObject GetMapObject(float x, float y, float z, double distance);

    template <typename Sequence>
    casper_auto_msgs::MapObject GetMapObjectWithinTargetRoads(const Sequence& road_ids) const;

    std::vector<RoadId> GetTargetRoadSetByEuclideanDistance(float x, float y, float z, double distance);

    void AddPolygonsWithinTargetRoads(const std::vector<RoadId>& road_ids,
                                      casper_auto_msgs::BoundaryPolygons& boundaryPolygons,
                                      casper_auto_msgs::BoundaryPolygons& sidewalkPolygons,
                                      casper_auto_msgs::BoundaryPolygons& crosswalkPolygons);

    PolygonVector GetBoundaryPolygons(const Road& road, double step);
    PolygonVector GetSidewalkPolygons(const Road& road, double step);

    carla::geom::GeoLocation GetMapGeoLocation();

    const std::set<RoadId>& GetRoadIdSet();

private:
    template <typename T>
    Point32 ToPoint32(const T& point);

    static void DeleteInvalidUniqueId(MapBasePathVector &basepaths);

    template <typename Container>
    static void DeleteInvalidElement(Container &indices, const UniqueIdSet &valid_ids);

    template <typename MapContainer>
    void DeleteInvalidKeys(MapContainer &container, const UniqueIdSet &invalid_keys);

    // Since s from waypoint has to be used, the parameter waypoint can not be replaced with
    // road_id, section_id, lane_id like MarkLeftRightMapLaneForBasePath.
    UniqueIdSet GetEgoLaneBasePathUniqueIds(
            const WaypointSharedPtr &waypoint,
            double distance);

    void MarkLeftRightMapLaneForBasePath(MapLaneVector &mapLaneVector,
                                         RoadId road_id,
                                         SectionId section_id,
                                         LaneId lane_id,
                                         MapLanePosition left,
                                         MapLanePosition right);

    // Make this function private, because next, prev, left, right, opposite idxes and id.unique_id
    // have not been assigned after this call. They will be filled in AddLinksToAllMapBasePaths function.
    // Whereas id (except id.unique_id), lane, size, speed, turnEvent, points, lane_widths and curvatures
    // will be done here. Because these parts have nothing to do with other MapBasePaths
    // and getting them does not need to use _indexBasePaths (GetBasePathUniqueId).
    MapBasePathVector GetMapBasePaths(Road& road, double step);

    void AddLinksToAllMapBasePaths(MapBasePathVector &allBasepaths);

    DirectedPoint GetDirectedPointAtST(Road& road, double clamped_s, double t);

    void AddAllMapObjects(double step);
    void AddAllPolygons(double step);

    void SearchRoadsWithinDistance(uint32_t current_road_id,
                                   UniqueIdSet& unique_id_set,
                                   uint32_t current_unique_id,
                                   double rest_distance,
                                   SearchDirection searchDirection);

    RoadMarkStartSEndSInfos GetRoadMarkStartEndInfos(const Road &road,
                                                     RoadMarkType roadMarkType);
    LaneStartSEndSInfos GetLaneStartEndInfos(const Road& road,
                                             Lane::LaneType laneType);
    PointVector GetPointsForSRange(const Road& road,
                                   const DoublePair& s_pair,
                                   double step,
                                   LaneId lane_id,
                                   PointPosition pointPosition);
    RoadMarkIdTypeColorMap GetRoadMarkIdTypeColorMapOfLane(const Lane &lane);

    // Used to get points for boundary polygon and sidewalk polygon.
    template <typename PointsContainer>
    void FillPointsContainer(const Road &road,
                             double step,
                             PointsContainer &pointsContainer);

    // used by boundary polygon
    void AddPointsAt(const Road &road,
                     BoundaryPolygonPointsMap &results,
                     double s);

    // used by sidewalk polygon
    void AddPointsAt(const Road &road,
                     SidewalkPolygonPointsMap &results,
                     double s);

    // used by boundary, base path, lane
    Point32 GetPointAt(const Road& road,
                       double s,
                       LaneId laneId,
                       PointPosition pointPosition);

    // used to get road boundary points to draw road polygons
    template <typename T>
    void AddSidePointsAt(BoundaryPolygonPointsMap &results,
                         double s,
                         const T& side_lanes,
                         std::map<LaneId, const Lane*>& lanes,
                         const Road& road);

    // used to get sidewalk points to draw sidewalk polygons
    template <typename T>
    void AddSidePointsAt(SidewalkPolygonPointsMap &results,
                         double s,
                         const T& side_lanes,
                         const Road& road);

    template <typename Container>
    void CopyRoadDataToContainer(const RoadIdUniqueIdsMap &indexMap,
                                 const Container &from,
                                 Container &to,
                                 RoadId roadId) const;

    int32_t GetBasePathUniqueId(RoadId roadId,
                                SectionId sectionId,
                                LaneId laneId) const;

    bool IsWhiteLine(LaneMarking::Type type);

    // used by boundary, lane
    bool IsTargetLine(LaneMarking::Type type,
                      RoadMarkType roadMarkType);

    double GetCurvatureAt(const Road& road, double s) const;
    double GetLaneWidthAt(const Lane& lane, double s) const;

    void ReverseDirection(casper_auto_msgs::MapLane& mapLane);
    void AddPathConnection(std::vector<casper_auto_msgs::MapBasepath> path_connection_list);
private:
    boost::optional<carla::road::Map> _map;
    carla::SharedPtr<carla::client::Map> _client_map;
    casper_auto_msgs::MapObject _mapObject_all;
    std::set<RoadId> _road_id_set;

    PolygonVector _boundaryPolygons;
    PolygonVector _sidewalkPolygons;
    PolygonVector _crosswalkPolygons;
    // indexMap used to get base path, boundary, lane.
    RoadIdUniqueIdsMap _indexMapBasePaths;
    RoadIdUniqueIdsMap _indexMapLanes;
    RoadIdUniqueIdsMap _indexMapBoundaries;
    // indexMap used to get crosswalk, stop line, guard rail, traffic light.
    // _indexMapCrosswalks are both used by _mapObject_all.crosswalks and _crosswalkPolygons
    RoadIdUniqueIdsMap _indexMapCrosswalks;
    RoadIdUniqueIdsMap _indexMapStopLines;
    RoadIdUniqueIdsMap _indexMapGuardRails;
    RoadIdUniqueIdsMap _indexMapTrafficLight;
    // indexMap used to get boundary polygon, sidewalk polygon.
    RoadIdUniqueIdsMap _indexMapBoundaryPolygons;
    RoadIdUniqueIdsMap _indexMapSidewalkPolygons;
    // indexMap used to get unique_id of a MapBasePath.
    RoadIdSectionIdLaneIdUniqueIdMap _indexBasePaths;
    // last waypoint that is outside of Junction
    WaypointSharedPtr _waypoint_last {nullptr};
};

#endif //ODR_MAP
