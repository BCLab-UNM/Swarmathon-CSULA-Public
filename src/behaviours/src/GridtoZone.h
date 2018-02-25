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
};

#endif // GRIDTOZONE
