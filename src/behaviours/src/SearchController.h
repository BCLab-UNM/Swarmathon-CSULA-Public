#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

#include "Spiral.cpp"
#include "RandomSelector.cpp"
#include "GridtoZone.h"
/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();

  int ChooseZone();
  Point GetNewSearchPoint();

  Position realtogridPosition(Point point);
//  Point gridtorealPosition(Position position);


protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
double x=0;
double y=0;
double d=1;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;


  // JS was Here
  int zone = 0;
  int spiralTurnsGoal = 8;
  int spiralTurnsCompleted = spiralTurnsGoal;

  int accuracy = 2;
  float areasize = 3.5;
  float areamargin = 0.1;

  float firsttravel = 0.4;
  float sectionlength = 1.2; // <- double check if true

  bool waypointsVerbose = true;
  bool gtzVerbose = true;
  bool rovercountverbose = false;
  bool waypointsDebugVerbose = true;

  Point centralSpiralLocation;

  double coverageWanted = 0.8;
  double sectionCoverageWanted = 0.45;

  RandomSelector rs = RandomSelector(accuracy, areasize, areamargin);
  Spiral s;
  int c = 0;
};

#endif /* SEARCH_CONTROLLER */
