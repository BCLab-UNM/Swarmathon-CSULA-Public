#ifndef GRIDTOZONE
#define GRIDTOZONE

#include <grid_map_ros/grid_map_ros.hpp>
#include <Eigen/Dense>

using namespace std;
using namespace grid_map;
using namespace Eigen;

class GridtoZone {

public:
	GridMap paperMap;
	GridtoZone();
	void setGridMap(GridMap map);

private:
	GridMap liveMap;
	bool verbose = false;

	int accuracy = 2;
	float zonesize = 3.75;
	int zoneclaimed = -1;

	// for a fixed number of turns
	float spiralsize = 2.0;


	// Ask Port
	string layer = "";
	float wallvalues[] = {0.5, 1.0}
	float floorvalues[] = {0.0, 0.25}
	float discorvedvalues[] = {0.25, 0.5, 1.0}

};

#endif // GRIDTOZONE

