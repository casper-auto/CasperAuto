/**
 * @file constants.h
 * @brief A collection of constants.
 **/

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <limits> /* DBL_MAX */

// Common Constants
const double kEpsilon = 1e-3;
const double kDBL_MAX = std::numeric_limits<double>::max();

// Behavior parameters
const double kSpeedShift = 1; // 1 - 5
const double kDecelFactor = 2.0;
const double kDeceleration = 2.0;
const double restart_boost = 0;

// Ego car
const double kEgoCarWidth = 2.1;
const double kEgoCarLength = 5.0;

// Dynamic Obstacles (Car, Truck, Bycicle, Pedestrian)
const double kCarWidth = 2.1;
const double kCarLength = 5.0;
const double kTruckWidth = 2.2;
const double kTruckLength = 10.0;
const double kBycicleWidth = 1.0;
const double kBycicleLength = 2.0;
const double kPedestrianWidth = 2.0;
const double kPedestrianLength = 2.0;

// Static Obstacles / Road Geometry Parameters
const double kLaneWidth = 3.2;
const double kStopsignWidth = kLaneWidth;
const double kStopsignLength = 0.5; // Along side the road direction
const double kCrosswalkWidth = 2 * kLaneWidth;
const double kCrosswalkLength = 2.0; // Along side the road direction
const double kTFLightWidth = 2 * kLaneWidth;
const double kTFLightLength = 0.1; // Along side the road direction
const double kCollisionThresh = 0.0;

// Planning Paramters
const double kLookaheadDistance = 50.0;
const double kMinUnitTime = 0.1;
const double kPlannedSRange = 50.0;     // ST graph and MPC
const double kPlannedTRange = 7.0;      // ST graph and MPC
const double kPlannedTimeHorizon = 8.0; // SpeedPlanner output
const double kPlannedUnitTime = 1.0;
const int kSpeedProfileSize = 200;
const double kArriveTolerance = 2.0;
const double kStopTolerance = 0.05;
const double kStopReleaseTime = 3.0;

// Crosswalk processing params
const double distance_to_react_crosswalk_scneraio = 20.0;
const double distance_to_react_enterroad_scneraio = 6.0;
const double distance_to_react_exitroad_scneraio = 6.0;
const double distance_to_react_rightturn_scneraio = 20.0;
const double distance_to_react_lefturn_scneraio = 8.0;

#endif // CONSTANTS_H
