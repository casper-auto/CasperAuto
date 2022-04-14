/**
 * @file trajectory_point.h
 * @brief TrajectoryPoint type.
 **/

#ifndef TRAJECTORY_POINT_H
#define TRAJECTORY_POINT_H

namespace planning {

/**
 * @struct TrajectoryPoint
 * @brief It represents a point in a trajectory that include \f$ x, y, v \f$ and
 * reltive time from the begining of the trajectory.
 */
struct TrajectoryPoint {
  /** @brief holds x value in \f$ m \f$ */
  double x;
  /** @brief holds y value in \f$ m \f$ */
  double y;
  /** @brief speed in \f$ m/sec \f$ */
  double v;
  /** @brief relative time from beginning of the trajectory */
  double relative_time;
  /** @brief speed limit in \f$ m/sec \f$ */
  double v_limit;

  /**
   * @brief Constructor
   */
  TrajectoryPoint() : x(0), y(0), v(0), relative_time(0), v_limit(0) {}

  /**
   * @brief Constructor
   * @param _x in \f$ m \f$
   * @param _y in \f$ m \f$
   *
   */
  TrajectoryPoint(double _x, double _y)
      : x(_x), y(_y), v(0), relative_time(0), v_limit(0) {}

  /**
   * @brief Constructor
   * @param _x in \f$ m \f$
   * @param _y in \f$ m \f$
   * @param _v in \f$ m/sec \f$
   * @param r_time relative time from the begining of the trajectory in \f$ sec
   * \f$
   */
  TrajectoryPoint(double _x, double _y, double _v, double r_time)
      : x(_x), y(_y), v(_v), relative_time(r_time), v_limit(_v) {}

  /**
   * @brief Constructor
   * @param _x in \f$ m \f$
   * @param _y in \f$ m \f$
   * @param _v in \f$ m/sec \f$
   * @param r_time relative time from the begining of the trajectory in \f$ sec
   * \f$
   */
  TrajectoryPoint(double _x, double _y, double _v, double r_time, double v_lmt)
      : x(_x), y(_y), v(_v), relative_time(r_time), v_limit(v_lmt) {}
};

} // namespace planning

#endif // TRAJECTORY_POINT_H
