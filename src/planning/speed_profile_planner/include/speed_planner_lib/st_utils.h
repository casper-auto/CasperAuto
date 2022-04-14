/**
 * @file st_utils.h
 * @brief Define some useful types.
 */

#ifndef ST_UTILS_H
#define ST_UTILS_H

#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Core>

namespace planning {

typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
typedef CPPAD_TESTVECTOR(double) Dvector;
/**
 * @struct STPoint
 * @brief Implementation of a point in a S-T Graph.
 */
struct STPoint {
  /** @brief holds x value in \f$ m \f$ */
  double s;
  /** @brief holds t value in \f$ s \f$ */
  double t;

  /**
   * @brief Constructor
   */
  STPoint() = default;

  /**
   * @brief Copy Constructor
   */
  STPoint(const STPoint &other) {
    s = other.s;
    t = other.t;
  }

  /**
   * @brief Constructor
   */
  STPoint(double _s, double _t) : s(_s), t(_t) {}
};

/**
 * @struct STArea
 * @brief Implemetation of an area in a S-T Graph.
 */
struct STArea {
  /** @brief holds cut_in point */
  STPoint cut_in;
  /** @brief holds cut_out point */
  STPoint cut_out;
  /** @brief holds upper_bound value in \f$ m \f$ */
  double upper_bound;
  /** @brief holds lower_bound value in \f$ m \f$ */
  double lower_bound;

  /**
   * @brief Constructor
   */
  STArea() = default;

  /**
   * @brief Copy Constructor
   */
  STArea(const STArea &other) {
    cut_in = other.cut_in;
    cut_out = other.cut_out;
    upper_bound = other.upper_bound;
    lower_bound = other.lower_bound;
  }

  /**
   * @brief Constructor
   */
  STArea(STPoint _cut_in, STPoint _cut_out, double _upper, double _lower)
      : cut_in(_cut_in), cut_out(_cut_out), upper_bound(_upper),
        lower_bound(_lower) {}
};

/**
 * @struct STCell
 *
 * @brief Representation of a cell in a S-T Graph.
 */
struct STCell {
  /** @brief holds min_s value in \f$ m \f$ */
  double min_s;
  /** @brief holds max_s value in \f$ m \f$ */
  double max_s;
  /** @brief holds type value representing the obstacle type */
  int type;

  /**
   * @brief Constructor
   */
  STCell() = default;

  /**
   * @brief Copy Constructor
   */
  STCell(const STCell &other) {
    min_s = other.min_s;
    max_s = other.max_s;
    type = other.type;
  }

  /**
   * @brief Constructor
   */
  STCell(double _min_s, double _max_s, int _type)
      : min_s(_min_s), max_s(_max_s), type(_type) {}
};

/**
 * @struct STConfig
 *
 * @brief Implementation of configuration state in a S-T Graph.
 */
struct STConfig {
  /** @brief holds s value in \f$ m \f$ */
  double s;
  /** @brief holds s_dot value in \f$ m/s \f$ */
  double s_dot;
  /** @brief holds s_dotdot value in \f$ m/s^2 \f$ */
  double s_dotdot;
  /** @brief holds s_dotdotdot value in \f$ m/s^3 \f$ */
  double s_dotdotdot;
  /** @brief holds s_dot_limit value in \f$ m/s \f$ */
  double s_dot_limit;
};

} // namespace planning

#endif // ST_UTILS_H
