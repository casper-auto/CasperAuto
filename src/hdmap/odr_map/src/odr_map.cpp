#include <fstream>
#include "boost/format.hpp"
#include "boost/enable_shared_from_this.hpp"

#include "odr_map.h"
#include "carla/road/element/RoadInfoLaneWidth.h"
#include "carla/road/element/RoadInfoMarkRecord.h"
#include "carla/road/element/LaneMarking.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include "carla/road/element/RoadInfoSpeed.h"
#include "carla/ListView.h"

#include "carla/geom/Math.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/Transform.h"

using namespace carla::road::element;
using namespace carla::geom;
using namespace geometry_msgs;

/**
 * @brief
 * @param[in] map_name
 * @param[in] map_path
 */
OdrMap::OdrMap(const std::string& xml_str, double step) {
    _map = carla::opendrive::OpenDriveParser::Load(xml_str);
    _client_map = boost::make_shared<carla::client::Map>("name", xml_str);
    AddAllMapObjects(step);
    AddAllPolygons(step);
}

/**
 * @brief
 * @param road_id
 * @return
 */
carla::road::Road& OdrMap::getRoad(carla::road::RoadId road_id) {
    auto& map_data = _map->GetMap();
    auto& road = map_data.GetRoad(road_id);
    return road;
}

/**
 * @brief
 * @param x
 * @param y
 * @param z
 * @return
 */
WaypointSharedPtr OdrMap::getWaypoint(float x, float y, float z) {
    auto ego_location = carla::geom::Location(x, y * -1.0, z); // Y axis Carla Hack
    auto lane_type_driving = static_cast<uint32_t>(carla::road::Lane::LaneType::Driving);
    auto waypoint = _client_map->GetWaypoint(ego_location);

    if (waypoint == nullptr) { //client/Map.cpp, 50~52
        ROS_ERROR_STREAM("Cannot get closest waypoint from point (" << x << ", "
                                                                    << y << ", " << z << ").");
    }

    return waypoint;
}

/**
 * @brief
 * @param x
 * @param y
 * @param z
 * @return
 */
bool OdrMap::isInRoad(float x, float y, float z) {
  auto location = carla::geom::Location(x, y * -1.0, z); // Y axis Carla Hack
  auto lane_type_driving = static_cast<uint32_t>(carla::road::Lane::LaneType::Driving);
  auto waypoint = _client_map->GetWaypoint(location, false, lane_type_driving);
  return (waypoint != nullptr);
}

/**
 * @brief Check if the query position is in a junction.
 * @param x
 * @param y
 * @param z
 * @return
 */
bool OdrMap::isInJunction(float x, float y, float z) {
  auto waypoint = getWaypoint(x, y, z);
  return waypoint->IsJunction();
}

/**
 * Get all waypoints of the map using carla API
 * @param distance the distance between neighboring waypoints.
 * @return         array of waypoints.
 */
std::vector<WaypointSharedPtr> OdrMap::GenerateWaypoints(double distance) const {
    RELEASE_ASSERT(distance > 0);
    auto all_waypoints = _client_map->GenerateWaypoints(distance);
    return all_waypoints;
}

/**
 *  @brief Get all MapLane structs within given road.
 *  @param  step     the distance of neighboring points at s coodinate
 *  @param  road     road from which we get info
 *  @return  Container where we put structs.
 *
 *  1. Get all lane points using MapAPI::AddAllLanePointsAtBySectionIdLaneIdMarkTypeMarkColor().
 *  2. Put them in MapLane struct vector.
*/
MapLaneVector OdrMap::GetMapLanes(const Road& road,
                                   double step) {
    RoadMarkStartSEndSInfos roadMarkStartSEndSInfos = GetRoadMarkStartEndInfos(
            road, RoadMarkType::White);

    MapLaneVector mapLanes;
    for (const auto& laneRoadMarks : roadMarkStartSEndSInfos) {
        casper_auto_msgs::MapLane mapLane;
        mapLane.id.section_id = laneRoadMarks.first.first;
        auto lane_id = laneRoadMarks.first.second;
        mapLane.id.lane_id = lane_id;
        mapLane.id.road_id = road.GetId();
        const auto& lane = road.GetLaneById(mapLane.id.section_id, mapLane.id.lane_id);

        RoadMarkIdTypeColorMap roadMarkIdTypeColorMap = GetRoadMarkIdTypeColorMapOfLane(lane);

        mapLane.type = static_cast<uint32_t>(lane.GetType());
        mapLane.size = 0;

        for (const auto& markIdSPairMap : laneRoadMarks.second) {
            casper_auto_msgs::LaneInformation laneInformation;
            laneInformation.points = GetPointsForSRange(road, markIdSPairMap.second,
                                                        step, lane_id, PointPosition::Outside);
            // lane mark type and color
            auto search = roadMarkIdTypeColorMap.find(markIdSPairMap.first);
            if (search != roadMarkIdTypeColorMap.end()) {
                laneInformation.road_mark = static_cast<uint8_t>(search->second.first);
                laneInformation.color = static_cast<uint8_t>(search->second.second);
            } else {
                ROS_ERROR("Road mark info for this road_mark_id does not exist!");
            }
            mapLane.size += laneInformation.points.size();
            mapLane.lane_information.emplace_back(std::move(laneInformation));
        }
        // basepath
        // lane id=0
        if (mapLane.id.lane_id) {
            ReverseDirection(mapLane);
        }

        // move
        mapLanes.emplace_back(std::move(mapLane));
    }
    return mapLanes;
}

/**
 * Add next_indices and prev_indices to all MapBasePaths using GetNextLanes and GetPrevLanes.
 * Add opposite, left, right indices to all MapBasePaths.
 * @param allBasepaths Container of all MapBasePaths.
 */
void OdrMap::AddLinksToAllMapBasePaths(MapBasePathVector& allBasepaths) {
    for (auto& basepath : allBasepaths) {
        auto roadId = basepath.id.road_id;
        auto sectionId = basepath.id.section_id;
        auto laneId = basepath.id.lane_id;
        auto& road = getRoad(roadId);
        auto& lane = road.GetLaneById(sectionId, laneId);

        std::set<uint32_t> nextLaneUniqueIds, prevLaneUniqueIds;
        // next_idxes
        auto nextLanes = lane.GetNextLanes();
        for (auto& nextLane : nextLanes) {
            auto roadIdNextLane = nextLane->GetRoad()->GetId();
            auto sectionIdNextLane = nextLane->GetLaneSection()->GetId();
            auto laneIdNextLane = nextLane->GetId();
            auto indexNextLane = GetBasePathUniqueId(roadIdNextLane, sectionIdNextLane, laneIdNextLane);
            if (indexNextLane != -1) {
                nextLaneUniqueIds.emplace(indexNextLane);
            } else {
                ROS_WARN_STREAM("Current Lane(<" << roadId
                                                 << ", " << sectionId << ", " << laneId << ">)'s next lane is <"
                                                 << roadIdNextLane << ", " << sectionIdNextLane << ", "
                                                 << laneIdNextLane << ">. Next lane's type is "
                                                 << static_cast<uint32_t>(nextLane->GetType()) << ".");
            }
        }

        // prev_idxes
        auto prevLanes = lane.GetPreviousLanes();
        for (auto& prevLane : prevLanes) {
            auto roadIdPrevLane = prevLane->GetRoad()->GetId();
            auto sectionIdPrevLane = prevLane->GetLaneSection()->GetId();
            auto laneIdPrevLane = prevLane->GetId();
            auto indexPrevLane = GetBasePathUniqueId(roadIdPrevLane, sectionIdPrevLane, laneIdPrevLane);
            if (indexPrevLane != -1) {
                prevLaneUniqueIds.emplace(indexPrevLane);
            } else {
                ROS_WARN_STREAM("Current Lane(<" << roadId
                                                 << ", " << sectionId << ", " << laneId << ">)'s prev lane is <"
                                                 << roadIdPrevLane << ", " << sectionIdPrevLane << ", "
                                                 << laneIdPrevLane << ">. Prev lane's type is "
                                                 << static_cast<uint32_t>(prevLane->GetType()) << ".");
            }
        }

        for (auto unique_id : nextLaneUniqueIds) {
            basepath.next_idxes.emplace_back(unique_id);
        }
        for (auto unique_id : prevLaneUniqueIds) {
            basepath.prev_idxes.emplace_back(unique_id);
        }

        // opposite_idxes
        // laneId
        auto oppositeLaneId = laneId;

        if (std::abs(laneId) == 1) {
            oppositeLaneId *= -1;
            auto oppositeUniqueId = GetBasePathUniqueId(roadId, sectionId, oppositeLaneId);
            if (oppositeUniqueId != -1) {
                basepath.opposite_idxes.emplace_back(oppositeUniqueId);
            }
        }

        auto rightLaneId = laneId, leftLaneId = laneId;

        // Carla GetLeft, GetRight
        if (laneId > 0) {
            ++rightLaneId;
            if (laneId > 1) {
                --leftLaneId;
            }
        } else {
            --rightLaneId;
            if (laneId < -1) {
                ++leftLaneId;
            }
        }

        if (rightLaneId != laneId) {
            auto rightUniqueId = GetBasePathUniqueId(roadId, sectionId, rightLaneId);
            if (rightUniqueId != -1) {
                basepath.right_idxes.emplace_back(rightUniqueId);
            }
        }
        if (leftLaneId != laneId) {
            auto leftUniqueId = GetBasePathUniqueId(roadId, sectionId, leftLaneId);
            if (leftUniqueId != -1) {
                basepath.left_idxes.emplace_back(leftUniqueId);
            }
        }
    }
}

/**
 * GetDirectedPointAtST is similar to Road::GetDirectedPointIn function.
 * They both use Elevation info.
 * GetDirectedPointIn is used to get the point on lane reference line.
 * While GetDirectedPointAtST is used to get the point using s,t coordinates based on road reference line.
 * @param road  the road where the s,t coordinate system is
 * @param clamped_s
 * @param t
 * @return      the DirectedPoint at (s, t). Besides location, there's also pitch and heading (yaw) information in it.
 */
DirectedPoint OdrMap::GetDirectedPointAtST(Road& road, double s, double t) {
    const auto clamped_s = Math::Clamp(s, 0.0, road.GetLength());
    const auto geometry = road.GetInfo<RoadInfoGeometry>(clamped_s);
    DirectedPoint p = geometry->GetGeometry().PosFromDist(clamped_s - geometry->GetDistance());
    // Apply road'clamped_s elevation record
    const auto elevation_info = road.GetElevationOn(clamped_s);
    p.location.z = static_cast<float>(elevation_info.Evaluate(clamped_s));
    p.pitch = elevation_info.Tangent(clamped_s);
    p.ApplyLateralOffset(-t);
    return p;
}

/**
 * Get all basepaths, lanes, boundaries, crosswalks, stop lines, traffic lights within this map,
 * and put them in _mapObject_all.
 * When someone needs all information about this map, just access _mapObject_all without computation.
 * @param step  the distance between neighboring points that will be used when getting basepaths, lanes, boundaries.
 * It should be given as a parameter for the constructor function.
 */
void OdrMap::AddAllMapObjects(double step) {
    // Step 1:
    int32_t indexLane = -1, indexBoundary = -1;
    int32_t indexCrosswalk = -1, indexStopLine = -1, indexRoadGuard = -1;
    int32_t indexTrafficLight = -1;
    int32_t indexBasePath = -1;
    //
    for (auto road_id : GetRoadIdSet()) {
        // road
        auto& road = getRoad(road_id);

        //
        auto basepaths = GetMapBasePaths(road, step);
        auto lanes = GetMapLanes(road, step);
        auto boundaries = GetMapBoundaries(road, step);

        // get all MapBasePaths for all roads in map using GetMapBasePaths
        //  with next, prev, left, right, opposite indices and id.unique_id unassigned.
        //  They will be assigned in AddLinksToAllMapBasePaths.
        for (auto& basepath : basepaths) {
            indexBasePath++;
            basepath.id.unique_id = indexBasePath;
            _indexMapBasePaths[basepath.id.road_id].emplace_back(indexBasePath);
            _indexBasePaths[basepath.id.road_id][basepath.id.section_id][basepath.id.lane_id]
                    = indexBasePath;
        }
        _mapObject_all.basepaths.insert(_mapObject_all.basepaths.end(),
                                        std::make_move_iterator(basepaths.begin()),
                                        std::make_move_iterator(basepaths.end()));

        // MapLane
        // _indexMapLane
        for (auto& lane : lanes) {
            indexLane++;
            lane.id.unique_id = indexLane;
            _indexMapLanes[lane.id.road_id].emplace_back(indexLane);
        }
        // unique_id
        _mapObject_all.lanes.insert(_mapObject_all.lanes.end(),
                                    std::make_move_iterator(lanes.begin()),
                                    std::make_move_iterator(lanes.end()));

        // MapBoundary
        // _indexMapBoundary
        for (auto& boundary : boundaries) {
            indexBoundary++;
            boundary.id.unique_id = indexBoundary;
            _indexMapBoundaries[boundary.id.road_id].emplace_back(indexBoundary);
        }
        _mapObject_all.boundaries.insert(_mapObject_all.boundaries.end(),
                                         std::make_move_iterator(boundaries.begin()),
                                         std::make_move_iterator(boundaries.end()));
    }
    // Step2: AddLinksToAllMapBasePaths
    AddLinksToAllMapBasePaths(_mapObject_all.basepaths);
}

/**
 * Getter for _mapObject_all.
 * @return the reference for _mapObject_all which contains all basepaths, lanes, boundaries, stop lines,
 * traffic lights, crosswalks.
 */
const casper_auto_msgs::MapObject& OdrMap::GetAllMapObjects() const {
    return _mapObject_all;
}

/**
 * This function is based on Map::ComputeTotalLaneWidth.
 * "Returns a pair containing first = width, second = tangent,
 * for an specific Lane given an s and a iterator over lanes"
 * The original function only calculate total width for point at the center of target lane.
 * In this function, we add an enum to indicate the position of the point,
 * which can be at the center or outside of the lane.
 * @tparam T            template for side lanes. left lanes and right lanes differ in std::make_reverse_iterator.
 * @param container     side lanes container
 * @param s             s coordinate
 * @param lane_id
 * @param pointPosition enum to indicate whether to take the point at center or outside of the lane with lane_id above
 * @return              std::pair<dist, tangent>, dist is the lateral distance from lane reference line.
 *                      tangent is the heading which has not been used.
 */
template<typename T>
static std::pair<double, double> ComputeTotalLaneWidthInOutCenter(
        const T container,
        const double s,
        const LaneId lane_id,
        const OdrMap::PointPosition pointPosition) {
    const bool negative_lane_id = lane_id < 0;
    double dist = 0.0;
    double tangent = 0.0;
    for (const auto& lane : container) {
        auto info = lane.second->template GetInfo<RoadInfoLaneWidth>(s);
        RELEASE_ASSERT(info != nullptr);
        const auto current_polynomial = info->GetPolynomial();
        auto current_dist = current_polynomial.Evaluate(s);
        auto current_tang = current_polynomial.Tangent(s);
        if (lane.first != lane_id) {
            dist += negative_lane_id ? current_dist : -current_dist;
            tangent += current_tang;
        } else if (lane.first == lane_id) {
            if (pointPosition == OdrMap::PointPosition::Center) {
                current_dist *= 0.5;
                dist += negative_lane_id ? current_dist : -current_dist;
                tangent += current_tang * 0.5;
            } else if (pointPosition == OdrMap::PointPosition::Outside) {
                dist += negative_lane_id ? current_dist : -current_dist;
                tangent += current_tang;
            } else {
                ROS_ERROR("This kind of PointPosition has not been dealt with!");
            }
            break;
        }
    }
    return std::make_pair(dist, tangent);
}

/**
 * This function gets the boundary points for one side of road.
 * The rule for taking boundary point is written as long comments within this function.
 * @tparam T            template for side lanes. left lanes and right lanes differ in std::make_reverse_iterator.
 * @param results       container to put boundary points in
 * @param s             s coordinate
 * @param side_lanes    side lanes container
 * @param lanes         all lanes container, from which the first opposite lane can be easily taken.
 *                      (really necessary?)
 * @param road          target road
 */
Point32 OdrMap::GetPointAt(const Road& road,
                            double s,
                            LaneId laneId,
                            PointPosition pointPosition) {
    DirectedPoint dp_lane = road.GetDirectedPointIn(s);
    if (laneId == 0) {
        // road marks are on the center line
    } else {
        std::map<LaneId, const Lane*> lanes(road.GetLanesAt(s));
        std::pair<double, double> computed_width;
        if (laneId < 0) {
            // right lane
            const auto right_lanes = MakeListView(
                    std::make_reverse_iterator(lanes.lower_bound(0)), lanes.rend());
            computed_width = ComputeTotalLaneWidthInOutCenter(right_lanes, s, laneId,
                                                              pointPosition);
        } else {
            // left lane
            const auto left_lanes = MakeListView(
                    lanes.lower_bound(1), lanes.end());
            computed_width = ComputeTotalLaneWidthInOutCenter(left_lanes, s, laneId,
                                                              pointPosition);
        }
        dp_lane.ApplyLateralOffset(computed_width.first);
    }
    return ToPoint32(dp_lane.location);
}

/**
 * Override function to get points used to draw road boundary polygons.
 * Although only the most outside points are needed.
 * Here the points near road reference line are always took.
 */
template<typename T>
void OdrMap::AddSidePointsAt(BoundaryPolygonPointsMap& results,
                              double s,
                              const T& side_lanes,
                              std::map<LaneId, const Lane*>& lanes,
                              const Road& road) {
    if (lanes.empty()) {
        ROS_ERROR("[odr_map AddSidePointsAt] lanes is empty!");
        return;
    }

    //
    int32_t laneIdInside = 0, laneIdOutside = 0;
    for (const auto& lane : side_lanes) {
        if (lane.second->GetType() == Lane::LaneType::Driving ||
            lane.second->GetType() == Lane::LaneType::Parking) {
            laneIdOutside = lane.first;
            if (laneIdInside == 0) {
                laneIdInside = lane.first;
            }
        }
    }

    if (laneIdInside == 0 && laneIdOutside == 0) {
        // driving lane
        return;
    } else if (laneIdInside == 0 || laneIdOutside == 0) {
        ROS_ERROR("laneIdInside and laneIdOutside must be both zero or both non-zero!");
    } else {
        const DirectedPoint dp_lane_zero = road.GetDirectedPointIn(s);
        double lane_width_inside = 0.0, lane_width_outside = 0.0;
        const bool negative_lane_id = laneIdInside < 0;
        DirectedPoint inside_current_dp = dp_lane_zero;
        DirectedPoint outside_current_dp = dp_lane_zero;

        if (std::abs(laneIdInside) == 1) {
            // center - driving
            int32_t oppositeLaneId = -laneIdInside;
            auto search = lanes.find(oppositeLaneId);
            // parking lane
            if (search != lanes.end()
                && (search->second->GetType() == Lane::LaneType::Driving
                    || search->second->GetType() == Lane::LaneType::Parking)) {
            } else {
                results[lanes[0]->GetLaneSection()->GetId()][0].emplace_back(ToPoint32(dp_lane_zero.location));
            }
        } else {
            auto one_more_inside_laneId = negative_lane_id ? (laneIdInside + 1) : (laneIdInside - 1);
            auto search = lanes.find(one_more_inside_laneId);
            if (search != lanes.end()) {
                if (search->second->GetType() == Lane::LaneType::Shoulder) {
                    const auto computed_width = ComputeTotalLaneWidthInOutCenter(
                            side_lanes, s, one_more_inside_laneId, PointPosition::Center);
                    lane_width_inside = computed_width.first;
                } else {
                    const auto computed_width = ComputeTotalLaneWidthInOutCenter(
                            side_lanes, s, one_more_inside_laneId, PointPosition::Outside);
                    lane_width_inside = computed_width.first;
                }
                inside_current_dp.ApplyLateralOffset(lane_width_inside);
                results[search->second->GetLaneSection()->GetId()][search->second->GetId()]
                        .emplace_back(ToPoint32(inside_current_dp.location));
            } else {
                ROS_ERROR("The lane ids are not continuous!");
            }
        }

        auto one_more_outside_laneId = negative_lane_id ? (laneIdOutside - 1) : (laneIdOutside + 1);
        auto search = lanes.find(one_more_outside_laneId);
        int32_t target_laneId = laneIdOutside;
        if (search != lanes.end() && search->second->GetType() == Lane::LaneType::Shoulder) {
            const auto computed_width = ComputeTotalLaneWidthInOutCenter(
                    side_lanes, s, one_more_outside_laneId, PointPosition::Center);
            lane_width_outside = computed_width.first;
            target_laneId = one_more_outside_laneId;
        } else {
            const auto computed_width = ComputeTotalLaneWidthInOutCenter(
                    side_lanes, s, laneIdOutside, PointPosition::Outside);
            lane_width_outside = computed_width.first;
        }
        outside_current_dp.ApplyLateralOffset(lane_width_outside);
        results[lanes[target_laneId]->GetLaneSection()->GetId()][target_laneId]
                .emplace_back(ToPoint32(outside_current_dp.location));
    }
}

/**
 * Override function to get points used to draw sidewalk polygons.
 * When we find a sidewalk type lane, we take points at both sides of it.
 * Compared to AddSidePointsAt for boundary polygon, lanes are not used.
 */
template<typename T>
void OdrMap::AddSidePointsAt(SidewalkPolygonPointsMap& results,
                              double s,
                              const T& side_lanes,
                              const Road& road) {
    for (const auto& lane_pair : side_lanes) {
        const Lane* lane = lane_pair.second;
        if (lane->GetType() == Lane::LaneType::Sidewalk) {
            const auto computed_width = ComputeTotalLaneWidthInOutCenter(
                    side_lanes, s, lane->GetId(), PointPosition::Outside);
            DirectedPoint dp_lane = road.GetDirectedPointIn(s);
            dp_lane.ApplyLateralOffset(computed_width.first);
            results[lane->GetLaneSection()->GetId()][lane->GetId()].first
                    .emplace_back(ToPoint32(dp_lane.location));
            const auto lane_width_info = lane->template GetInfo<RoadInfoLaneWidth>(s);
            auto width = static_cast<float>(lane_width_info->GetPolynomial().Evaluate(s));
            if (lane_pair.first < 0) {
                width = -width;
            }
            dp_lane.ApplyLateralOffset(width);
            results[lane->GetLaneSection()->GetId()][lane->GetId()].second
                    .emplace_back(ToPoint32(dp_lane.location));
        }
    }
}

/**
 * Overload for boundary polygons.
 * This function split lanes into left and right lanes,
 * then call AddSidePointsAt for them respectively.
 * @param results   container to put boundary points in
 * @param s         s coordinate
 * @param road      target road
 */
void OdrMap::AddPointsAt(const Road& road,
                          BoundaryPolygonPointsMap& results,
                          double s) {
    std::map<LaneId, const Lane*> lanes(road.GetLanesAt(s));
    // negative right lanes
    const auto right_lanes = MakeListView(
            std::make_reverse_iterator(lanes.lower_bound(0)), lanes.rend());
    // positive left lanes
    const auto left_lanes = MakeListView(
            lanes.lower_bound(1), lanes.end());

    AddSidePointsAt(results, s, left_lanes, lanes, road);
    AddSidePointsAt(results, s, right_lanes, lanes, road);
}

/**
 * Overload for sidewalk polygons.
 * Separate AddPointsAt for boundary polygon and sidewalk polygon,
 * because parameters for their AddSidePointsAt are different.
 */
void OdrMap::AddPointsAt(const Road& road,
                          SidewalkPolygonPointsMap& results,
                          double s) {
    std::map<LaneId, const Lane*> lanes(road.GetLanesAt(s));
    // negative right lanes
    const auto right_lanes = MakeListView(
            std::make_reverse_iterator(lanes.lower_bound(0)), lanes.rend());
    // positive left lanes
    const auto left_lanes = MakeListView(
            lanes.lower_bound(1), lanes.end());

    //
    AddSidePointsAt(results, s, left_lanes, road);
    AddSidePointsAt(results, s, right_lanes, road);
}

/**
 *  @brief Get all MapBoundary structs within given road.
 *  @param  step     the distance of neighboring points at s coodinate
 *  @param  road     road from which we get info
 *  @return  Container where we put structs.
 *
 *  1. Get all border lane points using MapAPI::GetAllLanePointsBySectionIdLaneId().
 *  2. Put them in MapBoundary struct vector.
*/
std::vector<casper_auto_msgs::MapBoundary> OdrMap::GetMapBoundaries(const Road& road, double step) {
    // type==curb
    RoadMarkStartSEndSInfos roadMarks = GetRoadMarkStartEndInfos(road, RoadMarkType::Curb);

    std::vector<casper_auto_msgs::MapBoundary> mapBoundaries;
    for (const auto& laneRoadMarks : roadMarks) {
        auto section_id = laneRoadMarks.first.first;
        auto lane_id = laneRoadMarks.first.second;
        // get lane type, not road mark type
        auto lane_type = static_cast<uint32_t>(road.GetLaneById(section_id, lane_id).GetType());
        for (const auto& markIdSPairMap : laneRoadMarks.second) {
            casper_auto_msgs::MapBoundary mapBoundary;
            mapBoundary.id.section_id = section_id;
            mapBoundary.id.lane_id = lane_id;
            mapBoundary.id.road_id = road.GetId();
            mapBoundary.points = GetPointsForSRange(road, markIdSPairMap.second,
                                                    step, lane_id, PointPosition::Outside);
            mapBoundary.size = mapBoundary.points.size();
            mapBoundary.type = lane_type;
            mapBoundaries.emplace_back(std::move(mapBoundary));
        }
    }
    return mapBoundaries;
}

/**
 * Used by MapBoundary and MapLane.
 * Get points from s_start to s_end by step.
 * And point at s_end must be taken.
 * pointPosition
 */
PointVector OdrMap::GetPointsForSRange(const Road& road,
                                        const DoublePair& s_pair,
                                        double step,
                                        LaneId lane_id,
                                        PointPosition pointPosition) {
    RELEASE_ASSERT(step > 0);
    PointVector pointVector;
    auto s_start = s_pair.first;
    auto s_end = s_pair.second;
    auto s = s_start;
    while (s < s_end) {
        pointVector.emplace_back(GetPointAt(road, s, lane_id, pointPosition));
        s += step;
    }
    Point32 point32 = GetPointAt(road, s_end, lane_id, pointPosition);
    if (pointVector.empty() ||
        (point32.x != pointVector.back().x
         || point32.y != pointVector.back().y
         || point32.z != pointVector.back().z)) {
        pointVector.emplace_back(point32);
    }
    return pointVector;
}

/**
 * Recursively search for target road as long as the road is within the range of distance.
 * There're 2 conditions on which the recursion ends.
 * First, the same MapBasePath has been searched the second times.
 * Second, the rest_distance <= 0.
 *
 * If recursion does not end, the next_idxes or prev_idxes are searched.
 * The rest_distance are subtracted by the length of road when the road_id of next MapBasePath is different from now.
 * @param current_road_id       if the road_id of next MapBasePath is different from it, distance should be subtracted
 * by the length of next road.
 * @param unique_id_set         always insert the current_unique_id, and if it exists, end the recursion.
 * @param current_unique_id     unique id of current MapBasePath
 * @param rest_distance         if rest_distance > 0, next road should be searched.
 * @param searchDirection       search forward or search backward.
 */
void OdrMap::SearchRoadsWithinDistance(uint32_t current_road_id,
                                        UniqueIdSet& unique_id_set,
                                        uint32_t current_unique_id,
                                        double rest_distance,
                                        SearchDirection searchDirection) {
    if (unique_id_set.insert(current_unique_id).second &&
        rest_distance > 0) {
        RELEASE_ASSERT(current_unique_id < _mapObject_all.basepaths.size());
        const auto& currentBasePath = _mapObject_all.basepaths[current_unique_id];
        // rest_distance > 0
        if (searchDirection == SearchDirection::Forward) {
            for (auto next_unique_id : currentBasePath.next_idxes) {
                RELEASE_ASSERT(next_unique_id < _mapObject_all.basepaths.size());
                const auto& nextBasePath = _mapObject_all.basepaths[next_unique_id];
                auto target_distance = rest_distance;
                auto target_road_id = current_road_id;
                if (nextBasePath.id.road_id != current_road_id) {
                    target_road_id = nextBasePath.id.road_id;
                    target_distance = target_distance - getRoad(target_road_id).GetLength();
                }
                SearchRoadsWithinDistance(target_road_id, unique_id_set, next_unique_id,
                                          target_distance, SearchDirection::Forward);
            }
        } else {
            for (auto prev_unique_id : currentBasePath.prev_idxes) {
                RELEASE_ASSERT(prev_unique_id < _mapObject_all.basepaths.size());
                const auto& prevBasePath = _mapObject_all.basepaths[prev_unique_id];
                auto target_distance = rest_distance;
                auto target_road_id = current_road_id;
                if (prevBasePath.id.road_id != current_road_id) {
                    target_road_id = prevBasePath.id.road_id;
                    target_distance = target_distance - getRoad(target_road_id).GetLength();
                }
                SearchRoadsWithinDistance(target_road_id, unique_id_set, prev_unique_id,
                                          target_distance, SearchDirection::Backward);
            }
        }
    } else {
        return;
    }
}

/**
 * Copy MapObject from _mapObject_all by road_ids
 * using their indexMaps and CopyRoadDataToContainer template function.
 * @param road_ids
 * @return  local info MapObject
 */
template<typename Sequence>
casper_auto_msgs::MapObject OdrMap::GetMapObjectWithinTargetRoads(const Sequence& road_ids) const {
    casper_auto_msgs::MapObject mapObject;

    for (auto roadId : road_ids) {
        CopyRoadDataToContainer(_indexMapBasePaths, _mapObject_all.basepaths,
                                mapObject.basepaths, roadId);
        CopyRoadDataToContainer(_indexMapLanes, _mapObject_all.lanes,
                                mapObject.lanes, roadId);
        CopyRoadDataToContainer(_indexMapBoundaries, _mapObject_all.boundaries,
                                mapObject.boundaries, roadId);
        CopyRoadDataToContainer(_indexMapTrafficLight, _mapObject_all.traffic_lights,
                                mapObject.traffic_lights, roadId);
        CopyRoadDataToContainer(_indexMapCrosswalks, _mapObject_all.crosswalks,
                                mapObject.crosswalks, roadId);
        CopyRoadDataToContainer(_indexMapStopLines, _mapObject_all.stop_lines,
                                mapObject.stop_lines, roadId);
        CopyRoadDataToContainer(_indexMapGuardRails, _mapObject_all.road_guards,
                                mapObject.road_guards, roadId);
    }

    // mapObject.basepaths left, right, next, prev, opposite idxes
    DeleteInvalidUniqueId(mapObject.basepaths);

    return mapObject;
}

/**
 * Get roads surround by Euclidean distance.
 * This implementation refers to Map::GetClosestWaypointOnRoad.
 * @param x
 * @param y
 * @param z
 * @param distance
 * @return
 */
std::vector<RoadId> OdrMap::GetTargetRoadSetByEuclideanDistance(float x, float y, float z, double distance) {
    // GetRoads()
    std::vector<RoadId> road_ids;
    Location pos = Location(x, y, z);
    for (auto road_id : GetRoadIdSet()) {
        const auto& road = getRoad(road_id);
        const auto current_dist = road.GetNearestPoint(pos);
        // driving lane
        auto lane_dist = road.GetNearestLane(current_dist.first, pos, static_cast<uint32_t>(Lane::LaneType::Any));
        if (lane_dist.second <= distance) {
            road_ids.emplace_back(road.GetId());
        }
    }
    return road_ids;
}

carla::geom::GeoLocation OdrMap::GetMapGeoLocation() {
    auto geo_location = _map->GetGeoReference();
    return geo_location;
}

/**
 * Get ego-lane-associated MapBasePath unique ids, starting from waypoint, using SearchRoadsWithinDistance.
 * The basepath unique id of waypoint is supposed to exist for current GetMapBasePaths method,
 * which takes basepath for every driving lane identified with road_id, section_id and lane_id.
 * @param waypoint
 * @param distance
 * @return ego-lane-associated basepath unique ids.
 */
UniqueIdSet OdrMap::GetEgoLaneBasePathUniqueIds(
        const WaypointSharedPtr& waypoint,
        double distance) {
    RELEASE_ASSERT(waypoint != nullptr);
    UniqueIdSet unique_id_set_forward;
    auto road_id = static_cast<RoadId>(waypoint->GetRoadId());
    auto section_id = static_cast<SectionId>(waypoint->GetSectionId());
    auto lane_id = static_cast<LaneId>(waypoint->GetLaneId());
    auto s = waypoint->GetDistance();
    // <roadId, sectionId, laneId> key
    int32_t basePathUniqueId = GetBasePathUniqueId(road_id, section_id, lane_id);
    RELEASE_ASSERT(basePathUniqueId >= 0);
    RELEASE_ASSERT(basePathUniqueId < _mapObject_all.basepaths.size());
    auto road_length = getRoad(road_id).GetLength();
    auto rest_distance_front = distance;
    if (lane_id < 0) {
        rest_distance_front = distance - (road_length - s);
    } else {
        rest_distance_front = distance - s;
    }
    SearchRoadsWithinDistance(waypoint->GetRoadId(), unique_id_set_forward,
                              basePathUniqueId, rest_distance_front, SearchDirection::Forward);
    return unique_id_set_forward;
}

/**
 * Get road boundary polygons for road.
 * Only the most outside points are needed.
 * For every section, we insert its leftmost and rightmost points into
 * left_points and right_points.
 * At the end, we merge them so that they are in an clockwise or counter clockwise order.
 * The same method is used in GetSidewalkPolygons.
 * @param road
 * @param step
 * @return
 */
PolygonVector OdrMap::GetBoundaryPolygons(const Road& road, double step) {
    BoundaryPolygonPointsMap boundaryPolygonPointsMap;
    FillPointsContainer(road, step, boundaryPolygonPointsMap);

    PolygonVector boundaryPolygons;
    PointVector left_points, right_points;
    for (const auto& pair : boundaryPolygonPointsMap) {
        auto& section_right_points = pair.second.begin()->second;
        right_points.insert(right_points.end(),
                            std::make_move_iterator(section_right_points.begin()),
                            std::make_move_iterator(section_right_points.end()));
        auto& section_left_points = pair.second.rbegin()->second;
        left_points.insert(left_points.end(),
                           std::make_move_iterator(section_left_points.begin()),
                           std::make_move_iterator(section_left_points.end()));
    }
    geometry_msgs::Polygon polygon;

    polygon.points.insert(polygon.points.end(),
                          std::make_move_iterator(left_points.begin()),
                          std::make_move_iterator(left_points.end()));
    polygon.points.insert(polygon.points.end(),
                          std::make_move_iterator(right_points.rbegin()),
                          std::make_move_iterator(right_points.rend()));
    if (!polygon.points.empty()) {
        // check polygon's points size
/*        if (polygon.points.size() <= 2) {
            ROS_ERROR("polygon's points number is less than 3!");
        }*/
        // driving lane
        boundaryPolygons.emplace_back(std::move(polygon));
    }
    return boundaryPolygons;
}

/**
 * The merge of points to constitute a polygon is the same with GetBoundaryPolygons.
 * To get all sidewalks across sections, we use lane's successor id to connect
 * sidewalk segments across sections.
 * For a sidewalk that we haven't dealt with, we look at its successor id until the end of the road.
 * For those sidewalks that begin at the middle of road, this method can also deal with.
 * @param road
 * @param step
 * @return
 */
PolygonVector OdrMap::GetSidewalkPolygons(const Road& road, double step) {
    SidewalkPolygonPointsMap sidewalkPolygonPointsMap;
    FillPointsContainer(road, step, sidewalkPolygonPointsMap);

    PolygonVector sidewalkPolygons;
    std::map<std::pair<SectionId, LaneId>, bool> flagMap;
    auto section_num = road.GetLaneSections().size();
    for (auto& sectionMap : sidewalkPolygonPointsMap) {
        for (auto& laneMap : sectionMap.second) {
            if (!flagMap[std::make_pair(sectionMap.first, laneMap.first)]) {
                PointVector outside_points, inside_points;
                outside_points.insert(outside_points.end(),
                                      std::make_move_iterator(laneMap.second.first.begin()),
                                      std::make_move_iterator(laneMap.second.first.end()));
                inside_points.insert(inside_points.end(), laneMap.second.second.begin(), laneMap.second.second.end());

                auto sectionId = sectionMap.first;
                auto laneId = laneMap.first;
                flagMap[std::make_pair(sectionId, laneId)] = true;

                // sectionId + 1 < section_num
                // laneId != 0: successor id
                while (sectionId + 1 < section_num && laneId != 0) {
                    auto& lane = road.GetLaneById(sectionId, laneId);
                    // successor id
                    laneId = lane.GetSuccessor();
                    sectionId++;
                    auto& points_pair = sidewalkPolygonPointsMap[sectionId][laneId];
                    if (!points_pair.first.empty()) {
                        outside_points.insert(outside_points.end(),
                                              std::make_move_iterator(points_pair.first.begin()),
                                              std::make_move_iterator(points_pair.first.end()));
                    }
                    if (!points_pair.second.empty()) {
                        inside_points.insert(inside_points.end(),
                                             std::make_move_iterator(points_pair.second.begin()),
                                             std::make_move_iterator(points_pair.second.end()));
                    }
                    flagMap[std::make_pair(sectionId, laneId)] = true;
                }
                geometry_msgs::Polygon polygon;
                if (laneMap.first > 0) {
                    polygon.points.insert(polygon.points.end(),
                                          std::make_move_iterator(outside_points.begin()),
                                          std::make_move_iterator(outside_points.end()));
                    polygon.points.insert(polygon.points.end(),
                                          std::make_move_iterator(inside_points.rbegin()),
                                          std::make_move_iterator(inside_points.rend()));
                } else {
                    polygon.points.insert(polygon.points.end(),
                                          std::make_move_iterator(inside_points.begin()),
                                          std::make_move_iterator(inside_points.end()));
                    polygon.points.insert(polygon.points.end(),
                                          std::make_move_iterator(outside_points.rbegin()),
                                          std::make_move_iterator(outside_points.rend()));
                }
                sidewalkPolygons.emplace_back(std::move(polygon));
            }
        }
    }
    return sidewalkPolygons;
}

/**
 * Get all boundary polygons and sidewalk polygons for map.
 * Used at the time when reading map.
 * crosswalk polygons are taken when AddAllMapObjects.
 * @param step
 */
void OdrMap::AddAllPolygons(double step) {
    int32_t indexBoundaryPolygon = -1, indexSidewalkPolygon = -1;
    for (auto road_id : GetRoadIdSet()) {
        auto& road = getRoad(road_id);

        auto boundaryPolygons = GetBoundaryPolygons(road, step);
        auto sidewalkPolygons = GetSidewalkPolygons(road, step);

        for (auto& boundaryPolygon : boundaryPolygons) {
            indexBoundaryPolygon++;
            _indexMapBoundaryPolygons[road_id].emplace_back(indexBoundaryPolygon);
        }
        _boundaryPolygons.insert(_boundaryPolygons.end(),
                                 std::make_move_iterator(boundaryPolygons.begin()),
                                 std::make_move_iterator(boundaryPolygons.end()));

        for (auto& sidewalkPolygon : sidewalkPolygons) {
            indexSidewalkPolygon++;
            _indexMapSidewalkPolygons[road_id].emplace_back(indexSidewalkPolygon);
        }
        _sidewalkPolygons.insert(_sidewalkPolygons.end(),
                                 std::make_move_iterator(sidewalkPolygons.begin()),
                                 std::make_move_iterator(sidewalkPolygons.end()));
    }
}

/**
 * template function used to get points for lane, boundary, boundary polygon, sidewalk polygon.
 * Use AddPointsAt at every step distance.
 * @tparam PointsContainer
 * @param road
 * @param step
 * @param pointsContainer
 */
template<typename PointsContainer>
void OdrMap::FillPointsContainer(const Road& road,
                                  double step,
                                  PointsContainer& pointsContainer) {
    RELEASE_ASSERT(step > 0);
    double roadLength = road.GetLength();
    double s = 0.0;
    while (s < roadLength) {
        AddPointsAt(road, pointsContainer, s);
        s += step;
    }
    AddPointsAt(road, pointsContainer, roadLength);
}

/**
 * Get boundary polygons and sidewalk polygons within target roads.
 * Used by ros service api.
 * @param road_ids
 * @param boundaryPolygons
 * @param sidewalkPolygons
 */
void OdrMap::AddPolygonsWithinTargetRoads(const std::vector<RoadId>& road_ids,
                                           casper_auto_msgs::BoundaryPolygons& boundaryPolygons,
                                           casper_auto_msgs::BoundaryPolygons& sidewalkPolygons,
                                           casper_auto_msgs::BoundaryPolygons& crosswalkPolygons) {
    for (auto roadId : road_ids) {
        CopyRoadDataToContainer(_indexMapBoundaryPolygons, _boundaryPolygons,
                                boundaryPolygons.polygons, roadId);
        CopyRoadDataToContainer(_indexMapSidewalkPolygons, _sidewalkPolygons,
                                sidewalkPolygons.polygons, roadId);
        CopyRoadDataToContainer(_indexMapCrosswalks, _crosswalkPolygons,
                                crosswalkPolygons.polygons, roadId);
    }
}

/**
 * Get local Map info within euclidean distance from position x, y, z.
 * @param x
 * @param y
 * @param z
 * @param distance
 * @return
 */
casper_auto_msgs::MapObject OdrMap::GetMapObject(float x, float y, float z, double distance) {
    auto road_id_set = GetTargetRoadSetByEuclideanDistance(x, y, z, distance);
    auto mapObject = GetMapObjectWithinTargetRoads(road_id_set);

    auto waypoint = getWaypoint(x, y, z);
    UniqueIdSet ego_unique_ids_for_signals, ego_unique_ids_for_lanes;
    if (waypoint->GetJunctionId() == -1 || _waypoint_last == nullptr) {
        ego_unique_ids_for_signals = GetEgoLaneBasePathUniqueIds(waypoint, distance);
        ego_unique_ids_for_lanes = GetEgoLaneBasePathUniqueIds(waypoint, LANE_DISTANCE);
        _waypoint_last = waypoint;
    } else {
        ego_unique_ids_for_signals = GetEgoLaneBasePathUniqueIds(_waypoint_last, distance);
        ego_unique_ids_for_lanes = GetEgoLaneBasePathUniqueIds(_waypoint_last, LANE_DISTANCE);
    }

    auto search_start_basepath_id = GetBasePathUniqueId(_waypoint_last->GetRoadId(),
                                                        _waypoint_last->GetSectionId(),
                                                        _waypoint_last->GetLaneId());
    for (auto unique_id : ego_unique_ids_for_lanes) {
        const auto& basepath = _mapObject_all.basepaths.at(unique_id);
        if (unique_id == search_start_basepath_id) {
            MarkLeftRightMapLaneForBasePath(mapObject.lanes,
                                            basepath.id.road_id,
                                            basepath.id.section_id,
                                            basepath.id.lane_id,
                                            MapLanePosition::EgoLeft,
                                            MapLanePosition::EgoRight);
        } else {
            MarkLeftRightMapLaneForBasePath(mapObject.lanes,
                                            basepath.id.road_id,
                                            basepath.id.section_id,
                                            basepath.id.lane_id,
                                            MapLanePosition::Related,
                                            MapLanePosition::Related);
        }
    }

    return mapObject;
}

/**
 * template function used to copy data from _mapObject_all to local MapInfo container.
 * @tparam Container
 * @param indexMap  All map messages have the same indexMap type.
 * @param from      container from which data are copied.
 * @param to        container to which data are copied.
 * @param roadId
 */
template<typename Container>
void OdrMap::CopyRoadDataToContainer(const RoadIdUniqueIdsMap& indexMap,
                                      const Container& from, Container& to, RoadId roadId) const {
    auto search = indexMap.find(roadId);
    if (search != indexMap.end()) {
        for (auto unique_id : search->second) {
            RELEASE_ASSERT(unique_id < from.size());
            auto& object = from[unique_id];
            to.emplace_back(object);
        }
    }
}

/**
 * Get basepath id by <RoadId, SectionId, LaneId> from 3-layer basepath map.
 * @param roadId
 * @param sectionId
 * @param laneId
 * @return
 */
int32_t OdrMap::GetBasePathUniqueId(RoadId roadId, SectionId sectionId, LaneId laneId) const {
    int32_t unique_id = -1;
    auto searchRoad = _indexBasePaths.find(roadId);
    if (searchRoad != _indexBasePaths.end()) {
        auto searchSection = searchRoad->second.find(sectionId);
        if (searchSection != searchRoad->second.end()) {
            auto searchLane = searchSection->second.find(laneId);
            if (searchLane != searchSection->second.end()) {
                unique_id = searchLane->second;
            }
        }
    }
    return unique_id;
}

/**
 * Get start_s and end_s info for all roadMarkType road marks.
 * If the current end_s equals the next start_s, the next width info will be taken.
 * Therefore EPSILON is used between them.
 * @param road
 * @param roadMarkType
 * @return
 */
RoadMarkStartSEndSInfos OdrMap::GetRoadMarkStartEndInfos(const Road& road,
                                                          RoadMarkType roadMarkType) {
    RoadMarkStartSEndSInfos roadMarkVector;
    for (const LaneSection& section : road.GetLaneSections()) {
        for (const auto& pair : section.GetLanes()) {
            const auto& lane = pair.second;
            RoadMarkStartSEndSVectorPair roadMarkStartEndRecord;
            roadMarkStartEndRecord.first.first = section.GetId();
            roadMarkStartEndRecord.first.second = lane.GetId();
            auto road_mark_info_list = lane.GetInfos<RoadInfoMarkRecord>();
            for (auto it = road_mark_info_list.begin();
                 it != road_mark_info_list.end(); ++it) {
                DEBUG_ASSERT(*it != nullptr);
                const auto road_mark_id = (*it)->GetRoadMarkId();
                const auto lane_marking = LaneMarking(**it);
                if (IsTargetLine(lane_marking.type, roadMarkType)) {
                    DoublePair startEndS;
                    startEndS.first = (*it)->GetDistance();

                    auto it_next = it;
                    ++it_next;
                    double s_end;
                    if (it_next != road_mark_info_list.end()) {
                        s_end = (*it_next)->GetDistance();
                    } else {
                        s_end = road.UpperBound(lane.GetDistance());
                    }
                    RELEASE_ASSERT(s_end > startEndS.first);
                    startEndS.second = s_end;
                    roadMarkStartEndRecord.second[road_mark_id] = std::move(startEndS);
                }
            }
            if (!roadMarkStartEndRecord.second.empty()) {
                roadMarkVector.emplace_back(std::move(roadMarkStartEndRecord));
            }
        }
    }

    return roadMarkVector;
}

/**
 * Get std::map<road_mark_id, std::pair<LaneMarking::Type, LaneMarking::Color>> of a Lane.
 * @param lane
 * @return
 */
RoadMarkIdTypeColorMap OdrMap::GetRoadMarkIdTypeColorMapOfLane(const Lane& lane) {
    RoadMarkIdTypeColorMap roadMarkIdRoadMarkInfoMap;
    const auto roadMarkInfos = lane.GetInfos<RoadInfoMarkRecord>();
    for (const auto roadMarkInfo : roadMarkInfos) {
        if (roadMarkInfo != nullptr) {
            const auto& lane_marking = LaneMarking(*roadMarkInfo);
            roadMarkIdRoadMarkInfoMap[roadMarkInfo->GetRoadMarkId()] = std::make_pair(
                    lane_marking.type, lane_marking.color);
        }
    }
    return roadMarkIdRoadMarkInfoMap;
}

/**
 * The six lane mark types are considered to be white line.
 * @param type
 * @return
 */
bool OdrMap::IsWhiteLine(LaneMarking::Type type) {
    return (type == LaneMarking::Type::Solid ||
            type == LaneMarking::Type::Broken ||
            type == LaneMarking::Type::SolidSolid ||
            type == LaneMarking::Type::SolidBroken ||
            type == LaneMarking::Type::BrokenSolid ||
            type == LaneMarking::Type::BrokenBroken);
}

/**
 * Judge if type belongs to roadMarkType.
 * Used by MapBoundary (curb) and MapLane API.
 * Although LaneMarking::Type and OdrMap::RoadMarkType seem to have the same meaning,
 * LaneMarking::Type is much finer than OdrMap::RoadMarkType.
 * @param type
 * @param roadMarkType
 * @return
 */
bool OdrMap::IsTargetLine(LaneMarking::Type type,
                           RoadMarkType roadMarkType) {
    if (roadMarkType == RoadMarkType::White) {
        return IsWhiteLine(type);
    } else if (roadMarkType == RoadMarkType::Curb) {
        return (type == LaneMarking::Type::Curb);
    } else {
        ROS_ERROR("The RoadMarkType is neither White nor Curb!");
        return false;
    }
}

/**
 * Get MapBasePath messages of a Road.
 * Note that id.unique_id, next_idxes, prev_idxes, opposite_idxes, left_idxes and right_idxes have not
 * been assigned any values after this call.
 * They will be filled in AddLinksToAllMapBasePaths.
 * @param road
 * @param step
 * @return
 */
MapBasePathVector OdrMap::GetMapBasePaths(Road& road, double step) {
    RELEASE_ASSERT(step > 0);
    // points, size, speed, turnEvent, lane_widths, curvatures, lane
    LaneStartSEndSInfos laneStartSEndSInfos = GetLaneStartEndInfos(road,
                                                                   Lane::LaneType::Driving);

    MapBasePathVector mapBasepaths;
    for (const auto& laneStartSEndS : laneStartSEndSInfos) {
        casper_auto_msgs::MapBasepath mapBasepath;
        mapBasepath.id.road_id = road.GetId();
        auto section_id = laneStartSEndS.first.first;
        mapBasepath.id.section_id = section_id;
        auto lane_id = laneStartSEndS.first.second;
        mapBasepath.id.lane_id = lane_id;
        if (lane_id == 0 ) continue;;
        auto& lane = road.GetLaneById(section_id, lane_id);

        auto s_start = laneStartSEndS.second.first;
        auto s_end = laneStartSEndS.second.second;
        auto s = s_start;
        while (s < s_end) {
            // point, lane_widths, curvatures
            mapBasepath.points.emplace_back(GetPointAt(road, s, lane_id, PointPosition::Center));
            mapBasepath.curvatures.emplace_back(GetCurvatureAt(road, s));
            mapBasepath.lane_widths.emplace_back(GetLaneWidthAt(lane, s));
            s += step;
        }
        if (s_end - s + step >= DISTANCE_EPSILON ||
            s_end - s_start < DISTANCE_EPSILON) {
            mapBasepath.points.emplace_back(GetPointAt(road, s_end, lane_id, PointPosition::Center));
            mapBasepath.curvatures.emplace_back(GetCurvatureAt(road, s_end));
            mapBasepath.lane_widths.emplace_back(GetLaneWidthAt(lane, s_end));
        }

        if (mapBasepath.id.lane_id > 0) {
            // points, curvatures, lane_widths
            std::reverse(mapBasepath.points.begin(), mapBasepath.points.end());
            std::reverse(mapBasepath.curvatures.begin(), mapBasepath.curvatures.end());
            std::reverse(mapBasepath.lane_widths.begin(), mapBasepath.lane_widths.end());
        }
        mapBasepath.size = mapBasepath.points.size();

        // speed
        auto speed_info = road.GetInfo<RoadInfoSpeed>(lane.GetDistance());
        if (speed_info != nullptr) {
            // speed info is double type in carla
            mapBasepath.speed = static_cast<uint32_t>(speed_info->GetSpeed());
        } else {
            mapBasepath.speed = 0;
        }

        // lane
        mapBasepath.lane = 0;
        auto lanes_in_section = road.GetLaneSectionById(section_id).GetLanesOfType(Lane::LaneType::Driving);
        for (auto lane_in_section : lanes_in_section) {
            if ((lane_id > 0 && lane_in_section->GetId() > lane_id)
                || (lane_id < 0 && lane_in_section->GetId() < lane_id)) {
                mapBasepath.lane++;
            }
        }

        mapBasepaths.emplace_back(std::move(mapBasepath));
    }

    return mapBasepaths;
}

/**
 * Get curvature at s in a Road.
 * @param road
 * @param s
 * @return
 */
double OdrMap::GetCurvatureAt(const Road& road, double s) const {
    RELEASE_ASSERT(s >= 0 && s <= road.GetLength());
    const auto geometry_info = road.GetInfo<RoadInfoGeometry>(s);
    RELEASE_ASSERT(geometry_info != nullptr);
    auto& geometry = geometry_info->GetGeometry();
    if (geometry.GetType() == GeometryType::LINE) {
        return 0.0;
    } else if (geometry.GetType() == GeometryType::ARC) {
        if (auto geometry_arc = dynamic_cast<const GeometryArc*>(&geometry)) {
            return geometry_arc->GetCurvature();
        } else {
            ROS_ERROR_STREAM("Cannot dynamic cast to GeometryArc");
            return 0.0;
        }
    } else {
       ROS_ERROR("GeometrySpiral has not been implemented!");
       return 0;
    }
}

/**
 * Get lane width at s in a Lane.
 * @param lane
 * @param s
 * @return
 */
double OdrMap::GetLaneWidthAt(const Lane& lane, double s) const {
    // road/Map.cpp GetLaneWidth
    RELEASE_ASSERT(s >= 0 && s <= lane.GetRoad()->GetLength());
    const auto lane_width_info = lane.GetInfo<RoadInfoLaneWidth>(s);
    RELEASE_ASSERT(lane_width_info != nullptr);
    return lane_width_info->GetPolynomial().Evaluate(s);
}

/**
 * Get lanes' start s and end s info for laneType in a Road.
 * @param road
 * @param laneType
 * @return
 */
LaneStartSEndSInfos OdrMap::GetLaneStartEndInfos(const Road& road, Lane::LaneType laneType) {
    LaneStartSEndSInfos laneStartSEndSInfos;
    for (const LaneSection& section : road.GetLaneSections()) {
        for (const auto& pair : section.GetLanes()) {
            const auto& lane = pair.second;
            if ((static_cast<uint32_t>(lane.GetType()) & static_cast<uint32_t>(laneType)) > 0) {
                LaneStartSEndSPair laneStartSEndSPair;
                laneStartSEndSPair.first.first = section.GetId();
                laneStartSEndSPair.first.second = lane.GetId();
                laneStartSEndSPair.second.first = lane.GetDistance();
                laneStartSEndSPair.second.second = road.UpperBound(lane.GetDistance()) - EPSILON;
                RELEASE_ASSERT(laneStartSEndSPair.second.second > laneStartSEndSPair.second.first);
                laneStartSEndSInfos.emplace_back(std::move(laneStartSEndSPair));
            }
        }
    }
    return laneStartSEndSInfos;
}

/**
 * Mark MapBasepath's left and right MapLane's position as 1.
 * The MapBasepath is identified with road_id, section_id, lane_id.
 * @param mapLaneVector
 * @param road_id
 * @param section_id
 * @param lane_id
 */
void OdrMap::MarkLeftRightMapLaneForBasePath(MapLaneVector& mapLaneVector,
                                              RoadId road_id,
                                              SectionId section_id,
                                              LaneId lane_id,
                                              MapLanePosition left,
                                              MapLanePosition right) {
    if (lane_id == 0) {
        ROS_ERROR("Center line (lane id=0) does not have left or right MapLane!");
        return;
    }
    auto left_lane_id = lane_id;
    auto right_lane_id = lane_id;
    if (lane_id < 0) {
        left_lane_id++;
    } else {
        left_lane_id--;
    }

    auto position_not_related = static_cast<uint8_t>(MapLanePosition::NotRelated);
    for (auto& mapLane : mapLaneVector) {
        if (mapLane.id.road_id == road_id && mapLane.id.section_id == section_id) {
            if (mapLane.id.lane_id == left_lane_id) {
                mapLane.position = static_cast<uint8_t>(left);
            }
            if (mapLane.id.lane_id == right_lane_id) {
                mapLane.position = static_cast<uint8_t>(right);
            }
            if (mapLane.position != position_not_related
                && mapLane.id.lane_id == 0
                && (lane_id > 0)) {
                ReverseDirection(mapLane);
            }
        }
    }
}

const std::set<RoadId>& OdrMap::GetRoadIdSet() {
    if (_road_id_set.empty()) {
        for (const auto& road_pair : _map->GetMap().GetRoads()) {
            _road_id_set.emplace(road_pair.first);
        }
    }
    return _road_id_set;
}

void OdrMap::ReverseDirection(casper_auto_msgs::MapLane& mapLane) {
    for (auto& laneInformation : mapLane.lane_information) {
        std::reverse(laneInformation.points.begin(), laneInformation.points.end());
    }
    std::reverse(mapLane.lane_information.begin(), mapLane.lane_information.end());
}

/**
 * If invalid unique ids exist in basepath's left, right, opposite, prev, next idxes, delete them.
 * Unique ids that do not belong to local info basepaths, are considered to be invalid.
 * @param basepaths     basepaths in local info taken by GetMapObject.
 */
void OdrMap::DeleteInvalidUniqueId(MapBasePathVector& basepaths) {
    UniqueIdSet uniqueIdSet;
    // 現在周辺情報にあるすべてのbasepathのunique idをstd::unordered_setに保存する。
    for (const auto& basepath : basepaths) {
        uniqueIdSet.insert(basepath.id.unique_id);
    }

    for (auto& basepath : basepaths) {
        DeleteInvalidElement(basepath.prev_idxes, uniqueIdSet);
        DeleteInvalidElement(basepath.next_idxes, uniqueIdSet);
        DeleteInvalidElement(basepath.left_idxes, uniqueIdSet);
        DeleteInvalidElement(basepath.right_idxes, uniqueIdSet);
        DeleteInvalidElement(basepath.opposite_idxes, uniqueIdSet);
    }
}

/**
 * Iterate index vector and erase those that are invalid.
 * @param indices       unique id vector.
 * @param valid_ids    unique ids to compare with.
 */
template<typename Container>
void OdrMap::DeleteInvalidElement(Container& indices,
                                   const UniqueIdSet& valid_ids) {
    for (auto it = indices.begin(); it != indices.end();) {
        if (valid_ids.find(*it) == valid_ids.end()) {
            it = indices.erase(it);
        } else {
            ++it;
        }
    }
}

/**
 * DeleteInvalidElement
 * @tparam MapContainer map type rather than set or vector,
 * because for map, we use it->first, while for set and vector, we use *it.
 * @param container     <key, value> container
 * @param invalid_keys  invalid keys to be compared with.
 */
template<typename MapContainer>
void OdrMap::DeleteInvalidKeys(MapContainer& container, const UniqueIdSet& invalid_keys) {
    for (auto it = container.begin(); it != container.end();) {
        if (invalid_keys.find(it->first) != invalid_keys.end()) {
            it = container.erase(it);
        } else {
            ++it;
        }
    }
}

/**
 * Convert different point object to Point32. Such as Location, Vector3D...
 * That's all because Point32 does not have constructor! So I cannot use emplace_back...
 * @tparam T
 * @param point
 * @return
 */
template<typename T>
Point32 OdrMap::ToPoint32(const T& point) {
    Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    point32.z = point.z;
    return point32;
}

/**
 * Add left-right connection relationship of basepath
 * @param path_connection_list: basepath to add
 * @return
 */
void OdrMap::AddPathConnection(std::vector<casper_auto_msgs::MapBasepath> path_connection_list) {
    if (path_connection_list.empty()) return;

    for (auto path_connection : path_connection_list) {

        // guard condition
        if (path_connection.id.unique_id < 0) {
            ROS_ERROR_STREAM("The unique_id is negative number: id = " << path_connection.id.unique_id);
            continue;
        }
        if (path_connection.id.unique_id >= _mapObject_all.basepaths.size()) {
            ROS_ERROR_STREAM("The unique_id is larger than basepaths.size(): id = " << path_connection.id.unique_id);
            continue;
        }

        for (uint32_t left_id : path_connection.left_idxes) {
            _mapObject_all.basepaths[path_connection.id.unique_id].left_idxes.push_back(left_id);
        }
        for (uint32_t right_id : path_connection.right_idxes) {
            _mapObject_all.basepaths[path_connection.id.unique_id].right_idxes.push_back(right_id);
        }
    }
}
