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

      GridtoZone::Instance()->updatePaperMap();
      if(gtzVerbose){
        std::cout << "Testing GTZ" << std::endl;
        std::cout << GridtoZone::Instance()->countOfTest() << std::endl;
        std::cout << GridtoZone::Instance()->percentOfTest() << std::endl;
      }

      //int zone, Position rover
      if(rovercountverbose){
        std::cout << "check for other rover in zone 0: " << GridtoZone::Instance()->otherRoverInZone(0, realtogridPosition(currentLocation)) << std::endl;
      }

      if(spiralTurnsCompleted == spiralTurnsGoal){
        if (waypointsVerbose){ std::cout<<  "Zone: " << zone << std::endl; }
        spiralTurnsCompleted = 0;

        /*=============================

            Do Gripmap work here to choose a zone
            check the zone if it does not have a good coverage or another rover has claimed it; otherwise get another zone
            get random point in zone(see centralSpiralLocation = rs.getRandomPointInZone(zone);)             
            check a the secion around that zone. if ok continue other wise get another point. Try for lets say 10 times otherwise choose adifferent zone.
            if section ok do the rest

        =============================*/

        GridtoZone::Instance()->updatePaperMap();
        centralSpiralLocation = GetNewSearchPoint();
        //centralSpiralLocation = rs.getRandomPointInZone(zone);

        Direction direction = Direction(rng->uniformInteger(0,3));
        bool clockwise = rng->uniformInteger(0,1);

        s.reset(centralSpiralLocation, direction , clockwise , firsttravel);
//        zone++;


        searchLocation = centralSpiralLocation;

        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
        return result;

      }
    searchLocation = s.getNextPoition();
    if (waypointsVerbose){ std::cout<<  "X: " << searchLocation.x << "  Y: " << searchLocation.y << std::endl; }
    spiralTurnsCompleted++;
 

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


//  centralSpiralLocation = rs.getRandomPointInZone(zone);
Point SearchController::GetNewSearchPoint(){
  int zonetries = 25;
  int pointstries = 50;
  Point pt =  rs.getRandomPointInZone(zone);

  for (int i = 0; i <= zonetries; i++){
    int choosenZone = ChooseZone();
    for (int j = 0; j <= pointstries; j++){


      pt =  rs.getRandomPointInZone(choosenZone);

      Position pos = realtogridPosition(pt);

      bool obstacle = GridtoZone::Instance()->obstaclesInZone(pos, sectionlength);
      double percent = GridtoZone::Instance()->percentOfSectionDiscovered(pos, sectionlength);
      bool percentOK = percent <= sectionCoverageWanted;

      if(waypointsDebugVerbose){
        std::cout << "========= GetNewSearchPoint =========" << std::endl;
        std::cout << "Considering point : " << pt.x  << ", "<< pt.y << std::endl;
        std::cout << "Zone : " << zone << std::endl;
        std::cout << "obstacle : " << obstacle << std::endl;
        std::cout << "percent of section : " << percent << std::endl;
      }

      if (!obstacle && percentOK){
        return pt;
      }

      if(i == 24){
        // force a new zone
        zone++;
      }
    }
  }

  std::cout << "It reached the last line at GetNewSearchPoint" << std::endl;
  std::cout << "It reached the last line at GetNewSearchPoint" << std::endl;
  std::cout << "It reached the last line at GetNewSearchPoint" << std::endl;
  std::cout << "Find out why" << std::endl;
  return pt;
}

int SearchController::ChooseZone(){
  GridtoZone::Instance()->updatePaperMap();

  int zoneChecking = zone;
  int tries = 100;
  int prelimCheck = 50;

  for(int i = 0; i <= tries; i++){

    Position position = realtogridPosition(currentLocation);

    bool rovercount = GridtoZone::Instance()->otherRoverInZone(zoneChecking, position);
    rovercount = false;

    Point zonepoint = rs.getZoneCenter(zoneChecking);

    double percentZone = GridtoZone::Instance()->percentOfSectionDiscovered(realtogridPosition(zonepoint), areasize);

//    double percentZone = GridtoZone::Instance()->percentOfZoneDiscovered(zoneChecking);

    if(waypointsDebugVerbose){
      std::cout << "========= ChooseZone ======================" << std::endl;
      std::cout << "zoneChecking: " << zoneChecking  << ", prelimCheck: " << (i <= prelimCheck) << std::endl;
      std::cout << "zonepoint: " << zonepoint.x  << ", " << zonepoint.y << std::endl;
      std::cout << "other rover : " << rovercount  << std::endl;
      std::cout << "zone coverage : " << percentZone  << std::endl;
    }



    if(i <= prelimCheck){
      zoneChecking = zoneChecking % 16; // ((15-1)/3.5)^2
    }
    else{
      zoneChecking = zoneChecking % 36; // ((22-1)/3.5)^2
    }

    if (rovercount){
      zoneChecking++;
    }

    else if(percentZone <= coverageWanted){
        zone = zoneChecking;
        return zoneChecking;
    }
    else{
      zoneChecking++;
    }
  }

  std::cout << "It reached the last line at ChooseZone" << std::endl;
  std::cout << "It reached the last line at ChooseZone" << std::endl;
  std::cout << "It reached the last line at ChooseZone" << std::endl;
  std::cout << "Find out why" << std::endl;
  zone = zoneChecking;
  return 0; // <----- because I have to return something
}






Position SearchController::realtogridPosition(Point point){
  return Position(-1 * point.y, point.x);  
}

/*This is broken
Point gridtorealPosition(Position position){
  float x = -1 * position(0,0);
  float y = 1 * position(0,1);
  return Point(y, x);
}
*/