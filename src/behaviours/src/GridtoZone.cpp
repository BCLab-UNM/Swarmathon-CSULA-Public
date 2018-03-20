#include "GridtoZone.h"
//#include <angles/angles.h>
#include <ros/ros.h>

// needed for polygon shape
#include <geometry_msgs/PolygonStamped.h>

#include <math.h>
#include <cmath>

using namespace Eigen;
using namespace grid_map;

GridtoZone::GridtoZone() { }

GridtoZone* GridtoZone::m_pInstance = NULL;

// OK //
GridtoZone* GridtoZone::Instance(){
	//cout << "Grid Instance Made" << endl;
	if (!m_pInstance){
		m_pInstance = new GridtoZone;
	}
	return m_pInstance;
}

// OK //
void GridtoZone::setGridMap(GridMap map) {
	Eigen::Vector2d origin(0,0);
	liveMap = map;
	if(verbose){
		cout << "Behavior Map Test" << endl;
		cout << "At (0,0): "<<map.atPosition("elevation", origin) <<endl;
	}
}

bool GridtoZone::otherRoverInZone(int zone, Position rover){
	int count = countRoversInZone(zone);
	Position center = getZonePosition(zone);
	double side = zonesize / 2;

/* No work

	Vector2d A( center.x() + side, center.y() + side );
	Vector2d B( center.x() + side, center.y() - side );
	Vector2d C( center.x() - side, center.y() - side );
	Vector2d D( center.x() - side, center.y() + side );

	Vector2d P( rover.x(), rover.y());

	Vector2d AB = A.cross(B);
	Vector2d AP = A.cross(P);
	Vector2d BC = B.cross(C);
	Vector2d BP = B.cross(P);


	double APdotAB = AP.adjoint()*AB;
	double ABdotAB = AB.adjoint()*AB;
	double BPdotBC = BP.adjoint()*BC;
	double BCdotBC = BC.adjoint()*BC;


// (0< APdotAB<ABdotAB)∧(0<APdotAD<ADdotAD)


	bool roverInZone = (0.0 <= APdotAB && APdotAB <= ABdotAB) && (0.0 <= BPdotBC && BPdotBC <= BCdotBC);

*/

	//https://math.stackexchange.com/a/190403
	float zoneside = zonesize;
	float zonearea = zonesize * zonesize;

	float x1 = center.x() + side;
	float x2 = center.x() + side;
	float x3 = center.x() - side;
	float x4 = center.x() - side;

	float y1 = center.y() + side;
	float y2 = center.y() - side;
	float y3 = center.y() - side;
	float y4 = center.y() + side;

	float b1 = sqrt ( pow (x1 - rover.x(), 2.0) + pow (y1 - rover.y(), 2.0) );
	float b2 = sqrt ( pow (x2 - rover.x(), 2.0) + pow (y2 - rover.y(), 2.0) );
	float b3 = sqrt ( pow (x3 - rover.x(), 2.0) + pow (y3 - rover.y(), 2.0) );
	float b4 = sqrt ( pow (x4 - rover.x(), 2.0) + pow (y4 - rover.y(), 2.0) );

	float u1 = zoneside + b1 + b2;
	float u2 = zoneside + b2 + b3;
	float u3 = zoneside + b3 + b4;
	float u4 = zoneside + b4 + b1;

	float a1 = sqrt( u1 * (u1 - zoneside) * (u1 - b1) * (u1 - b2) );
	float a2 = sqrt( u2 * (u2 - zoneside) * (u2 - b2) * (u2 - b3) );
	float a3 = sqrt( u3 * (u3 - zoneside) * (u3 - b3) * (u3 - b4) );
	float a4 = sqrt( u4 * (u4 - zoneside) * (u4 - b4) * (u4 - b1) );

	float area = a1 + a2 + a3 + a4;

	bool roverInZone = comparefloats(zonearea, area, .5);
	if (roverInZone){
		count--;
	}

	return count >= 1;
}


int GridtoZone::countRoversInZone(int zone){
	getZonePosition(zone);
	return countInSection(getZonePosition(zone),zonesize,ROVER);
}

// forgot what this is suppose to do.
bool GridtoZone::inZone(Position pos){
	return false;
}


// OK //
Position GridtoZone::getZonePosition(int zoneindex){
	double x = zonesize/2;
	double y = zonesize/2;

	int counter = 1;
	int maxcount = 1;
	int count = 0;
	bool turn = true;

	int distance = zonesize;
	// I couldn't get the other Direction from Spiral to work.
	GridDirection direction = West;

	for (int i = 0; i < zoneindex; i++){
		if(counter % 2 == 0 && counter > 0) {
			maxcount++;
			counter = 0;
		}
		switch(direction)
		{
			case North:
				y += distance;
				if(turn){
					direction = West;
					turn = false;
				}
				break;
			case East:
				x += distance;
				if(turn){
					direction = North;
					turn = false;
				}
				break;
			case South:
				y -= distance;
				if(turn){
					direction = East;
					turn = false;
				}
				break;
			case West:
				x -= distance;
				if(turn){
					direction = South;
					turn = false;
				}
				break;
			default:
				break;
		}
		count++;
		if(maxcount == count) {
			count = 0;
			counter++;
			turn = true;
		}
	}

	Position position = Position(x,y);

	if (positionverbose){
		std::cout<<  "Center of zone" << std::endl;
	    std::cout<<  "X: " << x << "  Y: " << y << std::endl;
	}

	return position;
}


void GridtoZone::updatePaperMap(){
	paperMap = liveMap;
}

int GridtoZone::countInSection(Position center, double length, float value){
	float values[] = {value};
	return countInSection(center,length,values,1);
}

int GridtoZone::countInSection(Position center, double length, float values[], int arrcount){
	double side = length / 2;

	grid_map::Polygon polygon;
	polygon.setFrameId(paperMap.getFrameId());
	// TopRight, BottomRight, TopLeft, BottomLeft
	polygon.addVertex(Position( center.x() + side, center.y() + side ));
	polygon.addVertex(Position( center.x() + side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() + side ));

	if (positionverbose){
		std::cout<<  "Corners of zone" << std::endl;	
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() + side << std::endl;
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() + side << std::endl;
	}
 
	int count = 0;
	int count2 = 0;

	for (grid_map::PolygonIterator iterator(paperMap, polygon); !iterator.isPastEnd(); ++iterator) {
		// ask what each value is with Port to check with.
		// ask about the layer
		if (positionverbose){
			//cout << "The value at index " << (*iterator).transpose() << " is " << paperMap.at("elevation", *iterator) << endl;
		}

		float mapValue = paperMap.at("elevation", *iterator);
		count2++;

		for (int i =0; i < arrcount; i++){
//			if (comparefloats(mapValue,values[i],0.5)){
			if (mapValue == values[i]){
				count++;
				break;
			}
		}
	}
	if (positionverbose){
		std::cout<<  "Total itr count: " <<  count2 << std::endl;	
		std::cout<<  "Total value count: " <<  count << std::endl;	
	}
	return count;
}







double GridtoZone::percentInSection(Position center, double length, float values[], int arrcount){
	double side = length / 2;

	grid_map::Polygon polygon;
	polygon.setFrameId(paperMap.getFrameId());
	// TopRight, BottomRight, TopLeft, BottomLeft
	polygon.addVertex(Position( center.x() + side, center.y() + side ));
	polygon.addVertex(Position( center.x() + side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() + side ));

	if (positionverbose){
		std::cout<<  "Corners of zone" << std::endl;	
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() + side << std::endl;
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() + side << std::endl;
	}


	int count = 0;
	int totalcount = 0;

	for (grid_map::PolygonIterator iterator(paperMap, polygon); !iterator.isPastEnd(); ++iterator) {
		totalcount++;

		float mapValue = paperMap.at("elevation", *iterator);

		for (int i =0; i < arrcount; i++){
			if (comparefloats(mapValue,values[i], 0.5)){
				count++;
				break;
			}
		}
	}

	if (positionverbose){
		std::cout<<  "Count stuff" << std::endl;	
	    std::cout<<  "count: " << count << std::endl;
	    std::cout<<  "totalcount: " << totalcount << std::endl;
	}

	return 1.0*count/totalcount;
}











double GridtoZone::percentOfZoneExplored(int zoneindex){
	return percentOfSectionExplored(getZonePosition(zoneindex),zonesize);
}

double GridtoZone::percentOfSectionExplored(Position center, double length){
	double sectionsize = (length / celldivision) * (length / celldivision);
	double sectionlength = (length / celldivision);
	int count = countInSection(center, sectionlength, floorvalues, 3);
	count = countInSection(center, length, allvalues, 7);
	return count/sectionsize;
}


double GridtoZone::percentOfZoneDiscovered(int zoneindex){
	return percentOfSectionDiscovered(getZonePosition(zoneindex),zonesize);
}

double GridtoZone::percentOfSectionDiscovered(Position center, double length){
//	double sectionsize = (length / celldivision) * (length / celldivision);
//	double sectionlength = (length / celldivision);

//	int count = countInSection(center, sectionlength, discorvedvalues,2);

//	int count = countInSection(center, sectionlength, arr,1);

	double per = percentInSection(center, length, discorvedvalues, 2);

	
//	cout << "percentInSection 1: "<< (count/sectionsize) <<endl;
//	cout << "percentInSection: "<< per <<endl;



	return per;
}

// Ambrosio do this
vector<Point> GridtoZone::shortestPath(Point start, Point end){
	// do something
	vector<Point> waypoints;
	return waypoints;
}


int GridtoZone::ClaimZone(int zone){
	// check if available
	zoneclaimed = zone;
	return zone;
	// else
	// return -1;
}

double GridtoZone::percentOfTest(){

	Position center = Position(0,0);
	Position zone0 = Position(1.75,1.75);
	Position pos_1count = Position(2.20,2.20); // expect 0.0002040816


	float values[] = {MAT};
//	int count = percentInSection(Position(0,0),length,values,1);
	return percentInSection(pos_1count,zonesize,values,1);
}

int GridtoZone::countOfTest(){

	Position center = Position(0,0);
	Position zone0 = Position(1.75,1.75);
	Position pos_1count = Position(2.0,2.0); // expect 0.0002040816

	float values[] = {MAT};

	return countInSection(pos_1count, zonesize, MAT);
}


bool GridtoZone::comparefloats(float a, float b, float acc){
	float c = b - a;
	return (c <= acc && c >= -1 * acc);
}

bool GridtoZone::obstaclesInZone(Position pos, float sectionlength){
	return countInSection(pos, sectionlength, wallvalues, 3) > 0;
}

bool GridtoZone::pathClear(float x1, float y1, float x2, float y2){
	GridtoZone::Instance()->updatePaperMap();
	cout << "pathClear working" << endl;
	cout << "x: " << x2 << endl;
	cout << "y: " << y2 << endl;
	
	
	grid_map::Polygon polygonPath;
	polygonPath.setFrameId(paperMap.getFrameId());

	double mapValue = -10.00;

	double distX = x2 - x1;
	double distY = y2 - y1;
	double distTotal = sqrt((distX*distX) + (distY*distY));
	float rad = std::atan2(x2 - x1, y2 - y1);
	float angle = rad * 180/pi;
	double botLeftX=  0			, botRightX=  0;
	double botLeftY= -ROVERHALF	, botRightY=  ROVERHALF;
	double topLeftX =  distTotal, topRightX= distTotal;
	double topLeftY = -ROVERHALF, topRightY= ROVERHALF;

	double rotateCamTopAngleRX = (topRightX * cos(rad) - topRightY * sin(rad));
	double rotateCamTopAngleRY = (topRightX * sin(rad) + topRightY * cos(rad));
	double rotateCamTopAngleLX = (topLeftX * cos(rad) - topLeftY * sin(rad));
	double rotateCamTopAngleLY = (topLeftX * sin(rad) + topLeftY * cos(rad));
			
	double rotateCamBotAngleRX = (botRightX * cos(rad) - botRightY * sin(rad));
	double rotateCamBotAngleRY = (botRightX * sin(rad) + botRightY * cos(rad));
	double rotateCamBotAngleLX = (botLeftX * cos(rad) - botLeftY * sin(rad));
	double rotateCamBotAngleLY = (botLeftX * sin(rad) + botLeftY * cos(rad));	

	polygonPath.addVertex(Position(rotateCamBotAngleRX + x1, rotateCamBotAngleRY + y1));
	polygonPath.addVertex(Position(rotateCamTopAngleRX + x2, rotateCamTopAngleRY + y2));
	polygonPath.addVertex(Position(rotateCamTopAngleLX + x2, rotateCamTopAngleLY + y2));
	polygonPath.addVertex(Position(rotateCamBotAngleLX + x1, rotateCamBotAngleLY + y1));
	
	for(grid_map::PolygonIterator iterator(paperMap, polygonPath); !iterator.isPastEnd(); ++iterator) {
		mapValue = paperMap.at("elevation", *iterator);
		if(mapValue == WALL){
			return false;
		}
	}	
	return true;
}