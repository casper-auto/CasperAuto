#ifndef PATH_OPTIMIZER_H
#define PATH_OPTIMIZER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <cfloat>

#include "path_optimizer/lbfgsb.hpp"
#include "path_optimizer/meta.h"

namespace planning {

const double INTERP_DISTANCE_RES = 0.2;

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

std::vector<double> linspace(double a, double b, int num = 50, bool endpoint = true);

std::vector<double> diff(std::vector<double> x);

std::vector<double> integrate(std::vector<double> y, std::vector<double> x);

std::vector<double> multiply_w_scalar(double val, std::vector<double> vec);

std::vector<double> add_w_scalar(double val, std::vector<double> vec);

std::vector<double> add(std::vector<double> a, std::vector<double> b);

std::vector<double> subtract(std::vector<double> a, std::vector<double> b);

double norm(std::vector<double> a);

///////////////////////////////////////////////////////////////////////////////
// class PathOptimizer
///////////////////////////////////////////////////////////////////////////////

class PathOptimizer {
public:
  // CONSTRUCTOR
  PathOptimizer();

  /////////////////////////////////////////////////////////////
  // PARAMETER OPTIMIZATION FOR POLYNOMIAL SPIRAL
  /////////////////////////////////////////////////////////////
  std::vector<std::vector<double>> optimizeSpiral(double xf, double yf, double tf);

  /////////////////////////////////////////////////////////////
  // SAMPLE SPIRAL PATH
  /////////////////////////////////////////////////////////////
  std::vector<std::vector<double>> sampleSpiral(std::vector<double> p_in);

private:
  double xf, yf, tf;

  // COMPUTE LIST OF THETAS
  std::vector<double> thetaf(double a, double b, double c, double d,
                        std::vector<double> ss);

  /////////////////////////////////////////////////////////////
  // BELOW ARE THE FUNCTIONS USED FOR THE OPTIMIZER.
  /////////////////////////////////////////////////////////////

  double objective(const Vector &x);

  void objectiveGrad(const Vector &x, Vector &grad);

  double fxf(std::vector<double> p);

  std::vector<double> fxfGrad(std::vector<double> p);

  double fyf(std::vector<double> p);

  std::vector<double> fyfGrad(std::vector<double> p);

  double ftf(std::vector<double> p);

  std::vector<double> ftfGrad(std::vector<double> p);

  double fbe(std::vector<double> p);

  std::vector<double> fbeGrad(std::vector<double> p);
};

} // namespace planning

#endif // PATH_OPTIMIZER_H
