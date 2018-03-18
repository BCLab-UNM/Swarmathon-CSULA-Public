#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <time.h>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

#include "SearchController.h"
#include "node.h"
#include "node.cpp"
#include "Point.h"
#include "GridtoZone.h"
#include <angles/angles.h>

using namespace Eigen;

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
        std::cout << "check for other rover in zone 0: " << GridtoZone::Instance()->otherRoverInZone(0, Position(currentLocation.x, currentLocation.y)) << std::endl;
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



        centralSpiralLocation = GetNewSearchPoint();
        //centralSpiralLocation = rs.getRandomPointInZone(zone);

        Direction direction = Direction(rng->uniformInteger(0,3));
        bool clockwise = rng->uniformInteger(0,1);

        s.reset(centralSpiralLocation, direction , clockwise , firsttravel);
        zone++;


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
  int pointstries = 25;
  Point pt =  rs.getRandomPointInZone(zone);

  for (int i = 0; i <= zonetries; i++){
    int choosenZone = ChooseZone();
    for (int j = 0; j <= pointstries; j++){


      pt =  rs.getRandomPointInZone(choosenZone);


      Position pos = Position(pt.x, pt.y);

      bool obstacle = GridtoZone::Instance()->obstaclesInZone(pos, sectionlength);
      double percent = GridtoZone::Instance()->percentOfSectionDiscovered(pos, sectionlength);
      bool percentOK = percent <= sectionCoverageWanted;


      if(waypointsDebugVerbose){
        // std::cout << "Considering point : " << pt.x  << ", "<< pt.y << std::endl;
        // std::cout << "Zone : " << zone << std::endl;
        // std::cout << "obstacle : " << obstacle << std::endl;
        // std::cout << "percent of section : " << percent << std::endl;
        // std::cout << "here2" << std::endl;
      }

      if (!obstacle && percentOK){
        return pt;
      }
    }
  }

  // std::cout << "It reached the last line at GetNewSearchPoint" << std::endl;
  // std::cout << "It reached the last line at GetNewSearchPoint" << std::endl;
  // std::cout << "It reached the last line at GetNewSearchPoint" << std::endl;
  // std::cout << "Find out why" << std::endl;
  return pt;

}

int SearchController::ChooseZone(){
  int zoneChecking = zone;
  int tries = 100;
  int prelimCheck = 50;


  for(int i = 0; i <= tries; i++){
    bool rovercount = GridtoZone::Instance()->otherRoverInZone(zoneChecking, Position(currentLocation.x, currentLocation.y));
    double percentZone = GridtoZone::Instance()->percentOfZoneDiscovered(zoneChecking);

    if(waypointsDebugVerbose){
      //std::cout << "zoneChecking: " << zoneChecking  << ", prelimCheck: " << (i <= prelimCheck) << std::endl;
      //std::cout << "other rover : " << rovercount  << std::endl;
      //std::cout << "zone coverage : " << percentZone  << std::endl;
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

  // std::cout << "It reached the last line at ChooseZone" << std::endl;
  // std::cout << "It reached the last line at ChooseZone" << std::endl;
  // std::cout << "It reached the last line at ChooseZone" << std::endl;
  // std::cout << "Find out why" << std::endl;
  zone = zoneChecking;



  return 0; // <----- because I have to return something
}


// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b){
  return a.getPriority() > b.getPriority();
}

//converts a the x value of a Point to a row value of index
int SearchController::pointXToIndex(float X){
  return (mapWidth/2 - 20*X);
}

//converts a the y value of a Point to a column value of index
int SearchController::pointYToIndex(float Y){
  return (mapLength/2 - 20*Y);
}

//converts the row value of an index to a x value of a Point
float SearchController::indexToPointX(int i){
  return (mapWidth/2 - i)/20;
}

//converts the column value of an index to a y value of a Point
float SearchController::indexToPointY(int j){
  return (mapLength/2 - j)/20;
}

// Parses the given route in string form int waypoints
vector<Point> SearchController::parseRoute(string route, float x, float y){
  std::vector<Point> waypoints;

  int routeLength = route.length();
  string routeArray[routeLength];

  // set point to the original starting Point
  Point point;
  point.x=x;
  point.y=y;
  
  const float celldivision=0.05;
  
  float distance = 0.0;


  // Makes route into array of characters
  int count=0;
  stringstream ssin(route);
  while (ssin.good() && count < routeLength){
    routeArray[count]=ssin.get();
    count++;
  }
 
  int routeCount=0; // Used to keep track of the number of steps within the same direction

  // Iterates through all steps in route
  for(int step = 0; step < routeLength; step++){

        // if the current step is not the first and the previos step and the current step are not in the same direction 
        if(step>0 && routeArray[step-1]!=routeArray[step]){
          
          
          distance = celldivision * routeCount; // The distance traveled in the previous direction

          // Add the distance traveled in the previos direction to the point
          if(routeArray[step-1]=="0"){point.y=point.y + (distance);}
          else if(routeArray[step-1]=="1"){point.y=point.y + (distance); point.x=point.x - (distance);}
          else if(routeArray[step-1]=="2"){point.x=point.x - (distance);}
          else if(routeArray[step-1]=="3"){point.y=point.y - (distance); point.x=point.x - (distance);}
          else if(routeArray[step-1]=="4"){point.y=point.y - (distance);}
          else if(routeArray[step-1]=="5"){point.y=point.y - (distance); point.x=point.x + (distance);}
          else if(routeArray[step-1]=="6"){point.x=point.x + (distance);}
          else {point.y=point.y + (distance); point.x=point.x + (distance);} //routeArray[step-1]=="7"


          waypoints.push_back(point);  // Add the point to collection of waypoints
          routeCount=0;  // Reset count since new direction

        }

        routeCount++; // Increment count since another step was taken in the same direction

        // if this is the last step
        if(step==routeLength-1){

          
          distance = celldivision * routeCount; // The distance traveled in the previous direction

          // Add the distance traveled in the previos direction to the point
          if(routeArray[step-1]=="0"){point.y=point.y + (distance);}
          else if(routeArray[step-1]=="1"){point.y=point.y + (distance); point.x=point.x - (distance);}
          else if(routeArray[step-1]=="2"){point.x=point.x - (distance);}
          else if(routeArray[step-1]=="3"){point.y=point.y - (distance); point.x=point.x - (distance);}
          else if(routeArray[step-1]=="4"){point.y=point.y - (distance);}
          else if(routeArray[step-1]=="5"){point.y=point.y - (distance); point.x=point.x + (distance);}
          else if(routeArray[step-1]=="6"){point.x=point.x + (distance);}
          else {point.y=point.y + (distance); point.x=point.x + (distance);} //routeArray[step-1]=="7"

          waypoints.push_back(point);  // Add the point to collection of waypoints
          routeCount=0;  // Reset count since new direction

          }
    }

    return waypoints;
}

string SearchController::Astar( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish )
{

    priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    int pqi; // priority queue, pq, index 
    node* currentNode;
    node* childNode;
    int i, j, x, y, xdx, ydy;
    char directionCharacter;
    pqi=0;


    Position currentPosition; // This is Position is used to check the value of the current Position we are on GridMap

    const int dir=8; // number of possible directions to go at any position
    int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1}; // Easy way to acces all directions
    int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};



    mapx->updatePaperMap(); // Updates Grid map 
  
    const int n=mapWidth; // horizontal size of the map x-axis
    const int m=mapLength; // vertical size size of the map y-axis    

    float closed_nodes_map[n][m]; // map of closed (tried-out) nodes
    float open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
    float dir_map[n][m]; // map of directions
    int count =0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0.0;
            open_nodes_map[x][y]=0.0;
        }
    }

    // create the start node and push into list of open nodes
    currentNode=new node(xStart, yStart, 0.0, 0.0,-1);
    currentNode->updatePriority(xFinish, yFinish);
    pq[pqi].push(*currentNode);
    open_nodes_map[x][y]=currentNode->getPriority(); // mark it on the open nodes map
    delete currentNode;
  
  
    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest prioritymap[
        // from the list of open nodes
        currentNode=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority(),pq[pqi].top().getParentDirection());

        x = currentNode->getxPos(); y = currentNode->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        
        closed_nodes_map[x][y]=1;// mark it on the closed nodes map
        count++;


        // quit searching when the goal state is reached
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                
                j=dir_map[x][y];
                directionCharacter='0'+(j+dir/2)%dir;
                path=directionCharacter+path;
                x+=dx[j];
                y+=dy[j];

            }
            
            delete currentNode; // garbage collection
            while(!pq[pqi].empty()) pq[pqi].pop(); // empty the leftover nodes        
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i]; // Sets the indexes of one othe child nodes to xdy and ydy
            
            currentPosition = Position(indexToPointX(xdx), indexToPointY(ydy)); // Transfroms index to matching point in Gridmap

            // Used to check wether given Point is outside of bounds
            //
            // if(mapx->paperMap.isInside(currentPosition)){
            //   cout<<"inside the map position ("<<currentPosition<<") "<<endl;
            // }
            // else{cout<<"not inside the map position ("<<currentPosition<<") "<<endl;
            // }
            
            float map_value = mapx->paperMap.atPosition("elevation",currentPosition);// Gets the value of Gridmap at current Position
            cout<<map_value<< endl;
            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map_value == ROVER || map_value == WALL || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                childNode=new node( xdx, ydy, currentNode->getLevel(), currentNode->getPriority(), currentNode->getParentDirection());
                childNode->nextLevel(i);
                childNode->updatePriority(xFinish, yFinish);

                // Used for debugging
                // if(mapx->paperMap.atPosition("elevation",currentPosition)!=-10){
                //   cout<<"value at position( "<<currentPosition<<") is "<<mapx->paperMap.atPosition("elevation",currentPosition)<<endl;
                // }

                // Adjusts the value recieved from map to better fit how cost is being interpreted 
                if(map_value == SONAR){
                  childNode->addcostToPriority(-10.0); // Gives sonar higher priority: -10
                } 
                else if(map_value == REVEALED){
                  childNode->addcostToPriority(-10.0); // Gives revealed higher priority: -10
                }
                else if(map_value == FOG){
                  childNode->addcostToPriority(0.0); // Gives Fog lower priority : 0
                }
                else if(map_value == MAT){
                  childNode->addcostToPriority(9.0); // Gives MAT lower priority : 9
                }
                else if(map_value == BUFFER){
                  childNode->addcostToPriority(20.0); // Gives BUFFER lower priority : 20
                }
                else {
                  childNode->addcostToPriority(map_value); // All other values stay as is
                } 

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0.0)
                {

                    open_nodes_map[xdx][ydy]=childNode->getPriority();

                    pq[pqi].push(*childNode);
                    delete childNode;
                    
                    dir_map[xdx][ydy]=(i+dir/2)%dir; // mark its parent node direction
                }
                //else the node is in the open list and if the current priority in the open nodes is larger 
                //than the priority of the node that is currently being observed
                else if(open_nodes_map[xdx][ydy]>childNode->getPriority())
                {
                    
                    open_nodes_map[xdx][ydy]=childNode->getPriority();// update the priority info
                    
                    dir_map[xdx][ydy]=(i+dir/2)%dir;// update the parent direction info

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx && pq[pqi].top().getyPos()==ydy))
                    {                

                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pq[pqi].pop(); // remove the wanted node
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {                

                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*childNode); // add the better node instead
                    delete childNode;
                    
                }
                // Else the current node has already been observed and it has a higher path cost so delete it.
                else delete childNode; // garbage collection
 
            }
        }
        delete currentNode; // garbage collection
    }
    if(pq[pqi].empty()){
      cout<<"empty"<<endl;
    }
    return ""; // no route found
}

vector<Point> SearchController::findPath(Point start, Point end){


  vector<Point> waypoints;

  // Switches x and y values as well as multiplies y value by -1
  // This is done to account for Gridmap's switched Y axis
  float xA = -1 * start.y;
  float yA = start.x;

  float xB = -1 * end.y;
  float yB = end.x;


  // Sets GridMap to map and updates it to most recent map.
  mapx = GridtoZone::Instance();
  mapx->updatePaperMap();

  // Gets the length and width of the Gridmap
  mapLength = mapx->paperMap.getSize()(0);
  mapWidth = mapx->paperMap.getSize()(1);

  // Converts x and y float values of Points to int appropriate index values
  // This is done because a star uses arrays
  int xStart = pointXToIndex(xA);
  int yStart = pointYToIndex(yA);
  int xFinish = pointXToIndex(xB);
  int yFinish = pointYToIndex(yB);


  // Used to time astar
  //clock_t timeStart = clock();


  // Give Astar start indexes and finish indexes
  // Returned will be the fastest route astar found put together as a string
  string route=Astar(xStart, yStart, xFinish, yFinish);


  if(route=="") cout<<"An empty route generated!"<<endl;
  
  //used to time astar
  // clock_t timeEnd = clock();
  // float time_elapsed = float(timeEnd - timeStart);
  // cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;

  //kept for debuging purposes prints the route string returned from astar
  // cout<<"Route:"<<endl;
  // cout<<route<<endl<<endl;

  // Parse the route and get waypoints 
  waypoints=parseRoute(route, start.x,start.y);

  return waypoints;
}

// Position SearchController::realtogridPosition(Point point){
//   return Position(-1 * point.y, point.x);  
// }




