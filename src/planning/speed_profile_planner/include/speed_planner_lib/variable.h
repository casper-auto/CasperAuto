#ifndef VARIABLE_H
#define VARIABLE_H

#include <cppad/cppad.hpp>

#include "speed_planner_lib/st_utils.h"

namespace planning {
/**
 * @file variable.h
 * @brief The class of Variable.
 */

/**
 * @class Variable
 *
 * @brief Implemenattion variable class for MPC.
 */

template <class Double, class Vector> class Variable {
public:
  Variable(Vector &variables, const size_t address = 0);

  void SetAddress(const size_t address);

  Double &operator[](size_t idx);

  const Double &operator[](size_t idx) const;

private:
  Vector &variables_m;
  size_t start_index_m;
};

template <class Double, class Vector>
Variable<Double, Vector>::Variable(Vector &variables, const size_t address)
    : variables_m(variables), start_index_m(address) {}

template <class Double, class Vector>
Double &Variable<Double, Vector>::operator[](size_t idx) {
  return variables_m[start_index_m + idx];
}

template <class Double, class Vector>
const Double &Variable<Double, Vector>::operator[](size_t idx) const {
  return variables_m[start_index_m + idx];
}

template <class Double, class Vector>
void Variable<Double, Vector>::SetAddress(const size_t address) {
  start_index_m = address;
}

} // namespace planning

#endif // VARIABLE_H
