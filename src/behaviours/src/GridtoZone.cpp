#include "GridtoZone.h"
#include <angles/angles.h>
#include <ros/ros.h>

GridtoZone::GridtoZone() {

}

void GridtoZone::setGridMap(GridMap map) {
	cout << "Behavior Map Test" << endl;
	Eigen::Vector2d origin(0,0);
	cout << "At (0,0): "<<map.atPosition("elevation", origin) <<endl;
}

