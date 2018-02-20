#include "GridtoZone.h"
#include <angles/angles.h>

GridtoZone::GridtoZone() {
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}



