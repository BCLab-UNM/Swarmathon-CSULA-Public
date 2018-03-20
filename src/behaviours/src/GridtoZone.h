#ifndef GRIDTOZONE
#define GRIDTOZONE

#include <grid_map_ros/grid_map_ros.hpp>
#include <Eigen/Dense>
#include <vector>
#include "Point.h"

#include <math.h>
#include <cmath>

using namespace std;
using namespace grid_map;
using namespace Eigen;

class GridtoZone {

public:
	static GridtoZone* Instance();

	GridMap paperMap;

	void setGridMap(GridMap map);


	void updatePaperMap();

	Position getZonePosition(int zoneindex);


	int countInSection(Position center, double length, float value);
	int countInSection(Position center, double length, float values[], int arrcount);
	double percentOfZoneExplored(int zoneindex);


	double percentOfSectionExplored(Position center, double length);


	double percentOfZoneDiscovered(int zoneindex);
	double percentOfSectionDiscovered(Position center, double length);

	// Ambrosio do this
	vector<Point> shortestPath(Point start, Point end);

	int ClaimZone(int zone);



	int countRoversInZone(int zone);
	bool inZone(Position pos);

	double percentOfTest();
	int countOfTest();
	double percentInSection(Position center, double length, float values[], int arrcount);
	enum GridDirection { North, East, South, West };

	bool comparefloats(float a, float b, float acc);

	bool otherRoverInZone(int zone, Position pos);

	bool obstaclesInZone(Position pos, float sectionlength);
	bool pathClear(float x1, float y1, float x2, float y2);


private:
	static GridtoZone* m_pInstance;
	GridtoZone();

	GridMap liveMap;
	bool verbose = false;
	bool positionverbose = false;
	bool rovercountverbose = true;

	int accuracy = 2;
	float zonesize = 3.5;
	int zoneclaimed = -1;

	// for a fixed number of turns
	float spiralsize = 2.0;

  	int mapLength;
  	int mapWidth;


	// Ask Port
	string layer = "elevation";
	const float celldivision= 0.05;
	const float ROVERHALF 	= 0.20;
	const float pi = std::acos(-1);

	const float FOG 	= -10.00;
	const float REVEALED 	= 0.00;
	const float MAT 	= 1.0;
	const float SONAR 	= 3.0;
	const float ROVER 	= 10.0;
	const float BUFFER 	= 15.0;
	const float WALL 	= 20.0;


	float wallvalues[3] = {WALL, ROVER, MAT};
	float floorvalues[3] = {REVEALED, SONAR, MAT};
	float discorvedvalues[2] = {REVEALED, MAT};
	float allvalues[7] = {FOG, REVEALED, MAT, SONAR, ROVER, WALL};

	// Used for testing
	// int astarCount=0;

	int pointXToIndex(float X);
 	int pointYToIndex(float Y);
 	float indexToPointX(int i);
  	float indexToPointY(int j);

  	vector<Point> parseRoute(string route, float x, float y);
  	string Astar( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish );
  	vector<Point> findPath(Point start, Point end);

};
#endif // GRIDTOZONE
