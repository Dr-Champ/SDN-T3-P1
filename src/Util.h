#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "PathPlanner.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

// Convert global XY coordinate into car's loc
vector<double> global2carXy(CarLocation& loc, vector<double> globalXy);

// Convert car's XY coordinate into global loc
vector<double> car2globalXy(CarLocation& loc, vector<double> carXy);