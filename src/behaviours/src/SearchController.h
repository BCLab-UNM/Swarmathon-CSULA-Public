#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

#include "Spiral.cpp"
#include "RandomSelector.cpp"
#include "GridtoZone.h"
#include "node.h"
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

  vector<Point> findPath(Point start, Point end);

  //GRID POINT TYPE
  const double FOG  = -10.00;
  const double REVEALED   = 0.00;
  const double MAT  = 1.0;
  const double MULTICUBES = 2.0;
  const double SONAR  = 3.0;
  const double ROVER  = 10.0;
  const double BUFFER   = 15.0;
  const double WALL   = 20.0;

  // Position realtogridPosition(Point point);



protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;


  int astarCount=0;

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

  bool waypointsVerbose = false;
  bool gtzVerbose = false;
  bool rovercountverbose = false;
  bool waypointsDebugVerbose = false;

  Point centralSpiralLocation;

  double coverageWanted = 0.8;
  double sectionCoverageWanted = 0.45;

  RandomSelector rs = RandomSelector(accuracy, areasize, areamargin);
  Spiral s;
  int c = 0;

  GridtoZone* mapx;
  int mapLength;
  int mapWidth;


  int pointXToIndex(float X);
  int pointYToIndex(float Y);
  float indexToPointX(int i);
  float indexToPointY(int j);

  vector<Point> parseRoute(string route, float x, float y);
  string Astar( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish );

  Position realtogridPosition(Point point);

};

#endif /* SEARCH_CONTROLLER */
