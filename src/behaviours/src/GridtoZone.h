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


	void updatePaperMap();

	Position getZonePosition(int zoneindex);


	int countInSection(Position center, double length, float value);
	int countInSection(Position center, double length, float values[], int arrcount);
	double percentOfZoneExplored(int zoneindex);


	double percentOfSectionExplored(Position center, double length);


	double percentOfZoneDiscovered(int zoneindex);

	double percentOfSectionDiscovered(Position center, double length);

// Ambrosio uncomment this
//	Waypoints shortestPath(Position start, Position end);

	int ClaimZone(int zone);



	int countRoversInZone(int zone);
	bool inZone(Position pos);

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

	float wallvalues[2] = {WALL, ROVER};
	float floorvalues[2] = {REVEALED, SONAR};
	float discorvedvalues[2] = {REVEALED, MAT};

};

#endif // GRIDTOZONE
