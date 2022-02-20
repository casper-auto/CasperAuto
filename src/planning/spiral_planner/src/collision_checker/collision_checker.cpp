#include "collision_checker/collision_checker.h"

namespace planning {

///////////////////////////////////////////////////////////////////////////////
// Helper Functions
///////////////////////////////////////////////////////////////////////////////

double logistic(double x) {
  /*
  A function that returns a value between 0 and 1 for x in the
  range[0, infinity] and -1 to 1 for x in the range[-infinity, infinity].

  Useful for cost functions.
  */

  return 2.0 / (1 + exp(-x)) - 1.0;
}

std::vector<std::vector<double>> compute_dists(std::vector<Point2D> &X,
                                               std::vector<Point2D> &Y) {
  std::vector<std::vector<double>> res(X.size(), std::vector<double>(Y.size()));
  for (int i = 0; i < X.size(); i++) {
    for (int j = 0; j < Y.size(); j++) {
      res[i][j] = distance2D(X[i], Y[j]);
    }
  }
  return res;
}

///////////////////////////////////////////////////////////////////////////////
// class CollisionChecker
///////////////////////////////////////////////////////////////////////////////

CollisionChecker::CollisionChecker(std::vector<double> &circle_offsets,
                                   std::vector<double> &circle_radii,
                                   double weight)
    : circle_offsets_(circle_offsets), circle_radii_(circle_radii),
      weight_(weight) {}

// Takes in a set of paths and obstacles, and returns an array
// of bools that says whether or not each path is collision free.
std::vector<bool>
CollisionChecker::collisionCheck(std::vector<std::vector<Pose2D>> &paths,
                                 std::vector<std::vector<Point2D>> &obstacles) {
  std::vector<bool> collision_check_array(paths.size());
  for (int i = 0; i < paths.size(); i++) {
    bool collision_free = true;
    std::vector<Pose2D> path = paths[i];
    // Iterate over the points in the path.
    for (int j = 0; j < path.size(); j++) {
      std::vector<Point2D> circle_locations(circle_offsets_.size(),
                                            Point2D(0, 0));
      for (int k = 0; k < circle_locations.size(); k++) {
        circle_locations[k].x =
            path[j].position.x + circle_offsets_[k] * cos(path[j].yaw);
        circle_locations[k].y =
            path[j].position.y + circle_offsets_[k] * sin(path[j].yaw);
      }
      for (int k = 0; k < obstacles.size(); k++) {
        std::vector<std::vector<double>> collision_dists =
            compute_dists(obstacles[k], circle_locations);
        for (std::vector<double> &dists : collision_dists) {
          for (int ii = 0; ii < dists.size(); ii++) {
            if (dists[ii] < circle_radii_[ii]) {
              collision_free = false;
              break;
            }
          }
          if (!collision_free)
            break;
        }
        if (!collision_free)
          break;
      }
      if (!collision_free)
        break;
    }
    collision_check_array[i] = collision_free;
  }

  return collision_check_array;
}

// Selects the best path in the path set, according to how closely
// it follows the lane centerline, and how far away it is from other
// paths that are in collision.
int CollisionChecker::selectBestPathIndex(
    std::vector<std::vector<Pose2D>> &paths,
    std::vector<bool> &collision_check_array, Pose2D &goal_pose) {
  int selected_index = paths.size() / 2;
  double best_score = DBL_MAX;
  for (int i = 0; i < paths.size(); i++) {
    double score = 0;
    // Handle the case of collision-free paths.
    if (collision_check_array[i]) {
      double distance = distance2D(paths[i][paths[i].size() - 1].position,
                                   goal_pose.position);
      score += logistic(distance);
      std::cout  << "Collision free path with cost = " << score << std::endl;
    }
    // Handle the case of colliding paths.
    else {
      // std::cout  << "path " << i << " get score inf!" << std::endl;
      score = 10000;
    }

    // Set the best index to be the path index with the lowest score
    if (score < best_score) {
      best_score = score;
      selected_index = i;
    }
  }

  return selected_index;
}

} // namespace planning
