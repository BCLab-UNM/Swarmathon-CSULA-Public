#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;


  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

  
}

void SearchController::Reset() {
  result.reset = false;
}

Result SearchController::DoWork() {
    // Search Here
      result.type = waypoint;
      first_waypoint = false;
      if(spiralTurnsCompleted == spiralTurnsGoal){
        std::cout<<  "Zone: " << zone << std::endl;
        spiralTurnsCompleted = 0;
        centralSpiralLocation = rs.getRandomPointInZone(zone);
        s.reset(centralSpiralLocation, North , false , 0.5f);
        zone++;

        searchLocation = centralSpiralLocation;

        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
        return result;

      }
    searchLocation = s.getNextPoition();
    std::cout<<  "X: " << searchLocation.x << "  Y: " << searchLocation.y << std::endl;
    spiralTurnsCompleted++;
 

//	c = (c + 1) % 4;
//    std::cout<<  c << std::endl;
//	if(c == 0) { searchLocation.x = 2; searchLocation.y = 2; }
//	if(c == 1) { searchLocation.x = -2; searchLocation.y = 2; }
//	if(c == 2) { searchLocation.x = -2; searchLocation.y = -2; }
//	if(c == 3) { searchLocation.x = 2; searchLocation.y = -2; }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  

}



void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


