#ifndef PATH_PLANNER
#define PATH_PLANNER

#include <vector>

using namespace std;

struct Map {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

struct CarLocation {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
};

struct PreviousLocation {
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
};

class PathPlanner {
public:
	PathPlanner();

	void computePath(vector<double>& next_x_vals, vector<double>& next_y_vals, 
		Map& map, CarLocation& loc, PreviousLocation& prevLoc);
private:
	void initializeNextVals(vector<double>& next_x_vals, 
		vector<double>& next_y_vals, double& pos_x, double& pos_y, double& angle, 
		CarLocation& loc, PreviousLocation& prevLoc);

	void computeWaypointsFrenet(vector<double>& next_s, vector<double>& next_d, 
		vector<double>& last_wp_sd);

	void splineTransformAndAppendNextVals(vector<double>& next_x_vals, 
	vector<double>& next_y_vals, vector<double>& next_s, vector<double>& next_d, 
	Map& map, CarLocation& loc);

    double maxSpeed = 22.3; 	// mps
    double maxTime = 5.0; 		// sec
    double timeStep = 0.02; 	// sec
    double sStep = maxSpeed * timeStep;
    double totalSteps = maxTime / timeStep;
};

#endif