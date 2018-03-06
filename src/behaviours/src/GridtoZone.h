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
	string layer = "elevation";
	const float celldivision 	= 0.05;

	const float FOG 	= -1.00;
	const float REVEALED 	= 0.00;
	const float MAT 	= 0.10;
	const float CUBES	= 0.20;
	const float SONAR 	= 0.30;
	const float ROVER 	= 1.00;
	const float WALL 	= 2.00;

	float wallvalues[] = {WALL, ROVER};
	float floorvalues[] = {REVEALED, SONAR};
	float discorvedvalues[] = {REVEALED, MAT};

};

#endif // GRIDTOZONE
