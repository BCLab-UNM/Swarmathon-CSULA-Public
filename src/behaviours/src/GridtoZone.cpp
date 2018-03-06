#include "GridtoZone.h"
#include <angles/angles.h>
#include <ros/ros.h>

// needed for polygon shape
#include <geometry_msgs/PolygonStamped.h>

GridtoZone::GridtoZone() { }

void GridtoZone::setGridMap(GridMap map) {
	Eigen::Vector2d origin(0,0);
	liveMap = map;
	if(verbose){
		cout << "Behavior Map Test" << endl;
		cout << "At (0,0): "<<map.atPosition("elevation", origin) <<endl;
	}
}

int countRoversInZone(int zone){
	float rovervalue = 1.0;
	return countInSection(getZonePosition(zone),zonesize,rovervalue);
}

// forgot what this is suppose to do.
bool inZone(Position pos){
	return false;
}

Position getZone(int zoneindex){
	Position position;
	position.x = zonesize;
	position.y = zonesize;

	int counter = 1;
	int maxcount = 1;
	int count = 0;
	bool turn = true;

	int distance = zonesize;
	Direction direction = West;

	//cout << "\nx: " << position.x << "\ty: " << position.y;

	for (int i = 0; i < zoneindex; i++){
		if(counter % 2 == 0 && counter > 0) {
			maxcount++;
			counter = 0;
		}
		switch(direction)
		{
			case North:
				position.y += distance;
				if(turn){
					direction = West;
					turn = false;
				}
				break;
			case East:
				position.x += distance;
				if(turn){
					direction = North;
					turn = false;
				}
				break;
			case South:
				position.y -= distance;
				if(turn){
					direction = East;
					turn = false;
				}
				break;
			case West:
				position.x -= distance;
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
	return position;
}

void updatePaperMap(){
	paperMap = LiveMap;
}

int countInSection(Poition center, double length, float value){
	float values[] = {value}
	return countInSection(center,length,values);
}

int countInSection(Poition center, double length, float values[]=NULL){
	double side = length / 2;

	grid_map::Polygon polygon;
	polygon.setFrameId(map_.getFrameId());
	// TopRight, BottomRight, TopLeft, BottomLeft
	polygon.addVertex(Position( center.x + side, center.y + side ));
	polygon.addVertex(Position( center.x + side, center.y - side ));
	polygon.addVertex(Position( center.x - side, center.y + side ));
	polygon.addVertex(Position( center.x - side, center.y - side ));

	int count = 0;

	for (grid_map::PolygonIterator iterator(map_, polygon); !iterator.isPastEnd(); ++iterator) {
		// ask what each value is with Port to check with.
		// ask about the layer

		mapValue = paperMap.atPosition(layer, *iterator);
		if ( std::find(std::begin(values), std::end(values), mapValue) != std::end(values) ){
			count++;
		}
	}
	return count;
}

double percentOfZoneExplored(int zoneindex){
	return percentOfSectionExplored(getZonePosition(zone),zonesize);
}

double percentOfSectionExplored(Poition center, double length){
	double sectionsize = (length / celldivision) * (length / celldivision);
	int count = countInSection(center, length, floorvalues);
	return count/sectionsize;
}

double percentOfZoneDiscovered(int zoneindex){
	return percentOfSectionDiscovered(getZonePosition(zone),zonesize);
}

double percentOfSectionDiscovered(Poition center, double length){
	int count = countInSection(center, length, discorvedvalues)
	return count/sectionsize;
}

Waypoints shortestPath(Position start, Position end){
	return null;
}

int ClaimZone(int zone){
	// check if available
	zoneclaimed = zone;
	return zone;
	// else
	// return -1;
}