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

/**
 * This code implements a 5 meter square loop starting at the center location.
 */
Result SearchController::DoWork() {

    double rotations = 0;
    double dist= 5;

           result.type = waypoint;
           Point searchLocation;
           if (attemptCount==0)
           {
               searchLocation.theta = 0;
               searchLocation.x = currentLocation.x + (dist);
               searchLocation.y = currentLocation.y + (0);
               attemptCount++;
               std::cout<< "Traveling to 1st waypoint \n" << rotations << " completed rotations";

           }
           else if (attemptCount==1)
           {
               searchLocation.theta = M_PI/2;
               searchLocation.x = currentLocation.x + (0);
               searchLocation.y = currentLocation.y + (dist);
               attemptCount++;
               std::cout<< "Traveling to 2nd waypoint";
           }
           else if (attemptCount==2)
           {
               searchLocation.theta = M_PI;
               searchLocation.x = currentLocation.x + (-dist);
               searchLocation.y = currentLocation.y + (0);
               attemptCount++;
               std::cout<< "Traveling to 3rd waypoint";
           }
           else{
               searchLocation.theta = 3/2 *M_PI;
               searchLocation.x = currentLocation.x +(0);
               searchLocation.y = currentLocation.y + (-dist);
               attemptCount=0;
               std::cout<< "Traveling to 4th waypoint";
               rotations++;
           }





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


