#include "PathPlanner.h"
#include "Util.h"
#include <iostream>
#include "spline.h"

PathPlanner::PathPlanner() {

}

/**
 Initialize the next_x/y_vals vectors with previously unused waypoints
 or with the latest current location / heading as returned from the simulator
 (in case there are no unused waypoints left). This is done so that the car's 
 motion is not jerky. All variables are in global x / y coordinates.
 */
void PathPlanner::initializeNextVals(vector<double>& next_x_vals, 
	vector<double>& next_y_vals, double& pos_x, double& pos_y, double& angle, 
	CarLocation& loc, PreviousLocation& prevLoc) {

  int path_size = prevLoc.previous_path_x.size();
  cout << "Reusing " << path_size << " previously unused waypoints" << endl;

  if (path_size == 0) {
	pos_x = loc.car_x;
	pos_y = loc.car_y;
	angle = deg2rad(loc.car_yaw);

	next_x_vals.push_back(pos_x);
	next_y_vals.push_back(pos_y);
  } 
  else {
  	for (int i = 0; i < path_size; i++) {
	  next_x_vals.push_back(prevLoc.previous_path_x[i]);
	  next_y_vals.push_back(prevLoc.previous_path_y[i]);
  	}

	pos_x = prevLoc.previous_path_x[path_size-1];
    pos_y = prevLoc.previous_path_y[path_size-1];
    double pos_x2 = prevLoc.previous_path_x[path_size-2];
    double pos_y2 = prevLoc.previous_path_y[path_size-2];
    angle = atan2(pos_y-pos_y2, pos_x-pos_x2);
  }

}

/**
 Compute some few waypoints in Frenet the car should pass through
 next, including the last unused waypoint. The returned next_s and
 next_d are meant to be used for spline fitting later on. That's 
 why the last unused waypoint is added to the vectors. They should
 not be included into the next_x/y_vals vectors though.
*/
void PathPlanner::computeWaypointsFrenet(vector<double>& next_s, 
	vector<double>& next_d, vector<double>& last_wp_sd) {

  int laneId = 1;
  cout << "Frenet of last unused waypoint (sd): " << last_wp_sd[0] << ", " 
  	  << last_wp_sd[1] << endl;

  // TODO FSM here to follow traffic or change lane
  next_s.push_back(last_wp_sd[0]);
  next_s.push_back(last_wp_sd[0] + (maxSpeed * maxTime / 3.0));
  next_s.push_back(last_wp_sd[0] + (2.0 * maxSpeed * maxTime / 3.0));
  next_s.push_back(last_wp_sd[0] + (3.0 * maxSpeed * maxTime / 3.0));
  next_d.push_back(last_wp_sd[1]);
  next_d.push_back(2 + (4 * laneId));
  next_d.push_back(2 + (4 * laneId));
  next_d.push_back(2 + (4 * laneId));
}

/**
  Receives sparse next Frenet coordinates, transform to local car's coordinate, 
  do Spline fitting, fill in the gaps, transform them back to global coordinate
  and append to next_x/y_vals
*/
void PathPlanner::splineTransformAndAppendNextVals(vector<double>& next_x_vals, 
	vector<double>& next_y_vals, vector<double>& next_s, vector<double>& next_d, 
	Map& map, CarLocation& loc) {

  // Convert path in sd into local xy coordinate
  vector<double> spline_x;
  vector<double> spline_y;
  for (int i = 0; i < next_s.size(); i++) {
  	cout << "Spine waypoint sd: " << next_s[i] << ", " << next_d[i] << endl;
  	vector<double> xy_global = getXY(next_s[i], next_d[i], map.map_waypoints_s, 
  		map.map_waypoints_x, map.map_waypoints_y);
  	vector<double> xy_car = global2carXy(loc, xy_global);

  	spline_x.push_back(xy_car[0]);
  	spline_y.push_back(xy_car[1]);
  }

  // Spline fit and fill the gaps
  tk::spline splineFitter;
  splineFitter.set_points(spline_x, spline_y);
  vector<double> next_x_car;
  vector<double> next_y_car;

  for (int i = 0; i < totalSteps; i++) {

  	double next_x = 0.0;
  	if (i > 1) {
  	  double delta_y = next_y_car[i - 1] - next_y_car[i - 2];
  	  double delta_x = next_x_car[i - 1] - next_x_car[i - 2];
  	  double phi_local = atan2(delta_y, delta_x);
  	  next_x = next_x_car[i - 1] + sStep * cos(phi_local);
  	}
  	else if (i == 1) {
  	  double delta_y = next_y_car[i - 1] - spline_y[0];
  	  double delta_x = next_x_car[i - 1] - spline_x[0];
  	  double phi_local = atan2(delta_y, delta_x);
  	  next_x = next_x_car[i - 1] + sStep * cos(phi_local);
  	}
  	else {
  	  next_x = spline_x[0] + sStep;
  	}

  	next_x_car.push_back(next_x);
  	next_y_car.push_back(splineFitter(next_x));
  	cout << "Spline fit local xy: " << next_x_car[i] << ", " << next_y_car[i] << endl;
  }

  // Convert local to global coordinate and append to the empty positions of 
  // next_x_vals, next_y_vals
  for (int i = 0; i < totalSteps; i++) {
  	if (next_x_vals.size() >= totalSteps) {
  	  break;
  	}

  	vector<double> next_xy_car;
  	next_xy_car.push_back(next_x_car[i]);
  	next_xy_car.push_back(next_y_car[i]);
  	vector<double> next_xy_global = car2globalXy(loc, next_xy_car);

  	next_x_vals.push_back(next_xy_global[0]);
  	next_y_vals.push_back(next_xy_global[1]);

  	// debug
  	int xSize = next_x_vals.size();
  	cout << "Adding " << i << ", " <<  next_x_vals[xSize - 1] << ", " << next_x_vals[xSize - 2] << ", " << next_s[i] 
  		<< ", " << next_d[i] << endl;
  	if (next_x_vals[xSize - 1] - next_x_vals[xSize - 2] > 1) {
  	  cout << "+++++++++++++   Here !!!!!!!! ++++++++++++++ " << endl;
  	}
  }
}

void PathPlanner::computePath(vector<double>& next_x_vals, vector<double>& next_y_vals, 
	Map& map, CarLocation& loc, PreviousLocation& prevLoc) {
  // Run state machine to determine the next state (keep lane, change lane, etc)
  // - Trajectory generation for each state with minimized jerkiness
  // - Compute associated cost
  // - Pick the state and corresponding trajectory
  // - Convert trajectory into xy coordinates

  cout << "=============================================" << endl;
  cout << "Current car location: " << loc.car_x << ", " << loc.car_y << ", " 
  	  << loc.car_s << ", " << loc.car_d << ", " << deg2rad(loc.car_yaw) << ", " 
  	  << loc.car_speed << endl;
  cout << "Last unused waypoint (sd): " << prevLoc.end_path_s << ", " << 
  	  prevLoc.end_path_d << endl;

  double pos_x;
  double pos_y;
  double angle;
  initializeNextVals(next_x_vals, next_y_vals, pos_x, pos_y, angle, loc, 
  	  prevLoc);
  cout << "Last unused waypoint (xy): " << pos_x << ", " << pos_y << ", " 
  	  << angle << endl;

  // Plan the path
  // FIXME getFrenet() sometimes returns incorrect d value, skipping from one lane to
  // another for no obvious reason. This makes the calculations for next waypoints 
  // unstable. See log below.
  vector<double> last_wp_sd = getFrenet(pos_x, pos_y, angle, 
  	  map.map_waypoints_x, map.map_waypoints_y);
  vector<double> next_s;
  vector<double> next_d;
  computeWaypointsFrenet(next_s, next_d, last_wp_sd);

  // Smooth the planned path and append to next_x/y_vals
  splineTransformAndAppendNextVals(next_x_vals, next_y_vals, next_s, next_d, 
	  map, loc);

  return;
}





/**

Spline fit local xy: 212.391, 58.56
Spline fit local xy: 212.832, 58.6284
Spline fit local xy: 213.272, 58.6968
From local 108.884, 22.4355 to global 1018.25, 1153.68 given 909.926, 1128.68, 0.0235622
Adding 0, 1018.25, 1017.93, 233.732, 5.93831
From local 109.295, 22.6083 to global 1018.66, 1153.86 given 909.926, 1128.68, 0.0235622	==> last waypoint added
Adding 1, 1018.66, 1018.25, 270.898, 6
=============================================
Current car location: 915.277, 1128.81, 130.631, 6.07493, 0.0258935, 49.8837		==> we're at d = 6
Last unused waypoint (sd): 234.226, -18.0785
Reusing 238 previously unused waypoints
Last unused waypoint (xy): 1018.66, 1153.86, 0.420502								==> this is still correct
Frenet of last unused waypoint (sd): 236.52, 1.17871								==> now d = 1?
Spine waypoint sd: 236.52, 1.17871
From global 1018.66, 1153.86 to local 103.996, 27.7162 given 915.277, 1128.81, 0.0258935
Spine waypoint sd: 273.687, 6
From global 1054.85, 1163.51 to local 140.427, 38.2978 given 915.277, 1128.81, 0.0258935
Spine waypoint sd: 310.854, 6
From global 1088.73, 1176.36 to local 174.623, 52.0258 given 915.277, 1128.81, 0.0258935
Spine waypoint sd: 348.02, 6
From global 1124.58, 1182.73 to local 210.634, 59.3237 given 915.277, 1128.81, 0.0258935
Spline fit local xy: 104.442, 27.8263
Spline fit local xy: 104.875, 27.9332
Spline fit local xy: 105.308, 28.0401
Spline fit local xy: 105.741, 28.147

*/



