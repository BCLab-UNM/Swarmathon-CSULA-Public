#include "GridtoZone.h"
//#include <angles/angles.h>
#include <ros/ros.h>

// needed for polygon shape
#include <geometry_msgs/PolygonStamped.h>
using namespace Eigen;

GridtoZone::GridtoZone() { }

GridtoZone* GridtoZone::m_pInstance = NULL;

// OK //
GridtoZone* GridtoZone::Instance(){
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
	
	Vector2f A( center.x() + side, center.y() + side );
	Vector2f B( center.x() + side, center.y() - side );
	Vector2f C( center.x() - side, center.y() - side );
	Vector2f D( center.x() - side, center.y() + side );

	Vector2f P( rover.x(), rover.y());

	Vector2f AB = A.cross(B);
	Vector2f AP = A.cross(P);
	Vector2f BC = B.cross(C);
	Vector2f BP = B.cross(P);


	double APdotAB = AP.adjoint()*AB;
	double ABdotAB = AB.adjoint()*AB;
	double BPdotBC = BP.adjoint()*BC;
	double BCdotBC = BC.adjoint()*BC;


// (0< APdotAB<ABdotAB)∧(0<APdotAD<ADdotAD)
	bool roverInZone = (0.0 <= APdotAB && APdotAB <= ABdotAB) && (0.0 <= BPdotBC && BPdotBC <= BCdotBC);

	if (roverInZone){
		count--;
	}

	return count <= 0;
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
			if (comparefloats(mapValue,values[i])){
				count++;
				break;
			}
		}
	}
	if (positionverbose){
		std::cout<<  "Total itr count: " <<  count2 << std::endl;	
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
			if (comparefloats(mapValue,values[i])){
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
	double sectionsize = (length / celldivision) * (length / celldivision);
	double sectionlength = (length / celldivision);
	int count = countInSection(center, sectionlength, discorvedvalues,2);
	return count/sectionsize;
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


bool GridtoZone::comparefloats(float a, float b){
	float c = b - a;
	return (c <= 0.5 && c >= -0.5);
}