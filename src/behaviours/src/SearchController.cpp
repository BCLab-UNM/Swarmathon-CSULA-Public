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
           result.type = waypoint;
           Point searchLocation;
           if (attemptCount==0)
           {
               searchLocation.theta = 0;
               searchLocation.x =(x); // x = 0
               searchLocation.y =(y); // y = 0
               attemptCount++;
               x=x+d;                 // d = 1
               std::cout<< "Traveling to 1st waypoint \n" << rotations << " completed rotations \n";
                std::cout<< "X: " << x << "  Y: " << y <<endl;
           }
           else if (attemptCount==1)
           {
               searchLocation.theta = M_PI/2;
               searchLocation.x = (x); // x = 1
               searchLocation.y = (y); // y = 0
               attemptCount++;
               y=y+d;
               d=(-d)-1;               // d = -2
               std::cout<< "Traveling to 2nd waypoint";
                std::cout<< "X: " << x << "  Y: " << y <<endl;
                std::cout<< "Traveling to 1st waypoint \n" << rotations << " completed rotations \n";
           }
           else if (attemptCount==2)
           {
               searchLocation.theta = M_PI;
               searchLocation.x = (x); // x = 1
               searchLocation.y = (y); // y = 1
               attemptCount++;
               x=x+d;
               std::cout<< "Traveling to 3rd waypoint";
                std::cout<< "X: " << x << "  Y: " << y <<endl;
           }
           else if (attemptCount==3){
               searchLocation.theta = 3/2 *M_PI;
               searchLocation.x = (x); // x = -1
               searchLocation.y = (y); // y = 1
               attemptCount++;
               y=y+d;
               d=(-d)+1;               // d = 3
               std::cout<< "Traveling to 4th waypoint";
                std::cout<< "X: " << x << "  Y: " << y <<endl;

           }
           else{
               searchLocation.theta = 0;
               searchLocation.x = (x); // x = -1
               searchLocation.y = (y); // y = -1
               attemptCount=1;
               x=x+d;

               std::cout<< "Traveling to 5th waypoint";
                std::cout<< "X: " << x << "  Y: " << y <<endl;
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


