#ifndef GRIDTOZONE
#define GRIDTOZONE

#include <random_numbers/random_numbers.h>

class GridtoZone {

public:

  GridtoZone();

  void SetCurrentLocation(Point currentLocation);

protected:

  void ProcessData();

private:

  Point currentLocation;
};

