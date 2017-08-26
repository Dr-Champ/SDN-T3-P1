#ifndef PATH_PLANNER
#define PATH_PLANNER

#include <vector>

using namespace std;

class PathPlanner {
public:
	PathPlanner();

	void computePath(vector<double>& next_x_vals, vector<double>& next_y_vals);
private:
};

#endif