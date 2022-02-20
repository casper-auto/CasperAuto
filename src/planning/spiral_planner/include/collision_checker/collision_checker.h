#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

#include "geometry.hpp"

using namespace geometry;

namespace planning {

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

double logistic(double x);

std::vector<std::vector<double>> compute_dists(std::vector<Point2D> &X,
                                               std::vector<Point2D> &Y);

///////////////////////////////////////////////////////////////////////////////
// class CollisionChecker
///////////////////////////////////////////////////////////////////////////////

class CollisionChecker {
public:
  CollisionChecker() = default;

  CollisionChecker(std::vector<double> &circle_offsets,
                   std::vector<double> &circle_radii, double weight);

  /*
    Returns a bool array on whether each path is collision free.

    args:
        paths: A list of paths in the global frame.
            A path is a list of points of the following format:
                [x_points, y_points, t_points]:
                    x_points: List of x values (m)
                    y_points: List of y values (m)
                    t_points: List of yaw values (rad)
                Example of accessing the ith path, jth point's t value:
                    paths[i][2][j]
        obstacles: A list of [x, y] points that represent points along the
            border of obstacles, in the global frame.
            Format: [[x0, y0],
                      [x1, y1],
                      ...,
                      [xn, yn]]
            , where n is the number of obstacle points and units are [m, m]
   */
  std::vector<bool>
  collisionCheck(std::vector<std::vector<Pose2D>> &paths,
                 std::vector<std::vector<Point2D>> &obstacles);

  /*
    Returns the path index which is best suited for the vehicle to
    traverse.

    Selects a path index which is closest to the center line as well as far
    away from collision paths.

    args:
        paths: A list of paths in the global frame.
            A path is a list of points of the following format:
                [x_points, y_points, t_points]:
                    x_points: List of x values (m)
                    y_points: List of y values (m)
                    t_points: List of yaw values (rad)
                Example of accessing the ith path, jth point's t value:
                    paths[i][2][j]
        collision_check_array: A list of boolean values which classifies
            whether the path is collision-free (true), or not (false). The
            ith index in the collision_check_array list corresponds to the
            ith path in the paths list.
        goal_state: Goal state for the vehicle to reach (centerline goal).
            format: [x_goal, y_goal, v_goal], unit: [m, m, m/s]
    useful variables:
        self._weight: Weight that is multiplied to the best index score.
    returns:
        selected_index: The path index which is best suited for the vehicle to
            navigate with.
   */
  int selectBestPathIndex(std::vector<std::vector<Pose2D>> &paths,
                          std::vector<bool> &collision_check_array,
                          Pose2D &goal_pose);

private:
  std::vector<double> circle_offsets_;
  std::vector<double> circle_radii_;
  double weight_;
};

} // namespace planning

#endif /* COLLISION_CHECKER_H */
