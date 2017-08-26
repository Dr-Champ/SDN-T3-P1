#include "PathPlanner.h"

PathPlanner::PathPlanner() {

}

void PathPlanner::computePath(vector<double>& next_x_vals, vector<double>& next_y_vals) {
  // Run state machine to determine the next state (keep lane, change lane, etc)
  // - Trajectory generation for each state with minimized jerkiness
  // - Compute associated cost
  // - Pick the state and corresponding trajectory
  // - Convert trajectory into xy coordinates

  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++) {
    next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
  }

  return;
}