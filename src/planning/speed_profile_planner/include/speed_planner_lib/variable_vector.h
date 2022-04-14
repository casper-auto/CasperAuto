#ifndef VARIABLE_VECTOR_H
#define VARIABLE_VECTOR_H

#include <array>

#include "speed_planner_lib/variable.h"

namespace planning {

/**
 * @file variable_vector.h
 * @brief The class of VariableVector.
 */

/**
 * @class VariableVector
 *
 * @brief Implemenattion template variable vector class for MPC. It encapsulate
 * the state representation.
 */

template <class Double, class Vector> class VariableVector {
public:
  VariableVector(const size_t variable_size, const size_t offset = 0);

  VariableVector(const size_t variable_size, Vector &variables,
                 const size_t offset = 0);

  size_t VariableSize();

  Variable<Double, Vector> s;
  Variable<Double, Vector> s_dot;    // velocity
  Variable<Double, Vector> s_dotdot; // acc

  Variable<Double, Vector> brake;    // actuator commands
  Variable<Double, Vector> throttle; // actuator commands
  Variable<Double, Vector> brake_dot;
  Variable<Double, Vector> throttle_dot;

  Vector &raw_data;

private:
  void Init(Variable<Double, Vector> &var);

  Vector variables_m;
  size_t variable_size_m = 40;
  size_t cur_index_m = 0;
  size_t vector_size_m = 0;
};

template <class Double, class Vector>
VariableVector<Double, Vector>::VariableVector(const size_t variable_size,
                                               const size_t offset)
    : VariableVector(variable_size, variables_m, offset) {
  variables_m.resize(vector_size_m);
}

template <class Double, class Vector>
VariableVector<Double, Vector>::VariableVector(const size_t variable_size,
                                               Vector &variables,
                                               const size_t offset)
    : s(variables), s_dot(variables), s_dotdot(variables), brake(variables),
      throttle(variables), brake_dot(variables), throttle_dot(variables),
      raw_data(variables) {
  cur_index_m = offset;
  variable_size_m = variable_size;
  Init(s);
  Init(s_dot);
  Init(s_dotdot);
  Init(brake);
  Init(throttle);
  Init(brake_dot);
  Init(throttle_dot);
}

template <class Double, class Vector>
void VariableVector<Double, Vector>::Init(Variable<Double, Vector> &var) {
  var.SetAddress(cur_index_m);
  cur_index_m += variable_size_m;
  vector_size_m = cur_index_m;
}

template <class Double, class Vector>
size_t VariableVector<Double, Vector>::VariableSize() {
  return variable_size_m;
}

} // namespace planning

#endif // VARIABLE_VECTOR_H
