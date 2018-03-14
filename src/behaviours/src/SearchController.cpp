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
        std::cout << "Considering point : " << pt.x  << ", "<< pt.y << std::endl;
        std::cout << "Zone : " << zone << std::endl;
        std::cout << "obstacle : " << obstacle << std::endl;
        std::cout << "percent of section : " << percent << std::endl;
        std::cout << "here2" << std::endl;
      }

      if (!obstacle && percentOK){
        return pt;
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
  int zoneChecking = zone;
  int tries = 100;
  int prelimCheck = 50;

  for(int i = 0; i <= tries; i++){
    bool rovercount = GridtoZone::Instance()->otherRoverInZone(zoneChecking, Position(currentLocation.x, currentLocation.y));
    double percentZone = GridtoZone::Instance()->percentOfZoneDiscovered(zoneChecking);

    if(waypointsDebugVerbose){
      std::cout << "zoneChecking: " << zoneChecking  << ", prelimCheck: " << (i <= prelimCheck) << std::endl;
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
    std::cout << "here1" << std::endl;
    Point start, end;
    start.x=0.0;
    start.y=0.0;
    end.x=8.0;
    end.y=8.0;


    findPath(start,end);
  }

  std::cout << "It reached the last line at ChooseZone" << std::endl;
  std::cout << "It reached the last line at ChooseZone" << std::endl;
  std::cout << "It reached the last line at ChooseZone" << std::endl;
  std::cout << "Find out why" << std::endl;
  zone = zoneChecking;
  return 0; // <----- because I have to return something
}


///////////////btmn

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b){
  return a.getPriority() > b.getPriority();
}


void SearchController::pointToIndex(Point point,int &x,int &y){
  //conversion for code testing
  // x=(int) point.x;
  // y=(int) point.y;  

  //conversion for swarm
  const int conversionNumber=20;
  x=(int) point.x*conversionNumber;
  y=(int) point.y*conversionNumber;
}

void SearchController::indexToPoint(vector<Point> &waypoints){
  const float conversionNumber=20.00;
  for(int i=0;i<waypoints.size();i++){
    waypoints[i].x=waypoints[i].x / conversionNumber;
    waypoints[i].y=waypoints[i].y / conversionNumber;
  }
}


vector<Point> SearchController::parseRoute(string route, int x, int y){
  std::vector<Point> waypoints;
  int len = route.length();
  string arr[len];
  Point point;
  point.x=x;
  point.y=y;

  // cout<< "route length: " << len << endl;
  int count=0;
  stringstream ssin(route);
  while (ssin.good() && count < len){
    // ssin >> arr[count];
    arr[count]=ssin.get();
    ++count;
  }
    
  int routeCount=0;
  for(int b = 0; b < len; b++){
        //cout <<"route:"<< arr[b] << endl;
        
        if(b>0){
            if(arr[b-1]!=arr[b]){
                //cout << "new way point" << endl;
        
                if(arr[b-1]=="0"){point.x=point.x + (routeCount);}
                else if(arr[b-1]=="1"){point.y=point.y + (routeCount); point.x=point.x + (routeCount);}
                else if(arr[b-1]=="2"){point.y=point.y + (routeCount);}
                else if(arr[b-1]=="3"){point.y=point.y - (routeCount); point.x=point.x + (routeCount);}
                else if(arr[b-1]=="4"){point.x=point.x - (routeCount);}
                else if(arr[b-1]=="5"){point.y=point.y - (routeCount); point.x=point.x - (routeCount);}
                else if(arr[b-1]=="6"){point.y=point.y - (routeCount);}
                else if(arr[b-1]=="7"){point.y=point.y + (routeCount); point.x=point.x - (routeCount);}
                else {cout <<"oh oh";}

                //cout << "waypoint( "<<point.y<<", "<<point.x<<")"<<endl;
                waypoints.push_back(point);
                 routeCount=0;

            }
        }
        routeCount++;
        if(b==len-1){
                //cout << "new way point" << endl;
        
                if(arr[b-1]=="0"){point.x=point.x + (routeCount);}
                else if(arr[b-1]=="1"){point.y=point.y + (routeCount); point.x=point.x + (routeCount);}
                else if(arr[b-1]=="2"){point.y=point.y + (routeCount);}
                else if(arr[b-1]=="3"){point.y=point.y - (routeCount); point.x=point.x + (routeCount);}
                else if(arr[b-1]=="4"){point.x=point.x - (routeCount);}
                else if(arr[b-1]=="5"){point.y=point.y - (routeCount); point.x=point.x - (routeCount);}
                else if(arr[b-1]=="6"){point.y=point.y - (routeCount);}
                else if(arr[b-1]=="7"){point.y=point.y + (routeCount); point.x=point.x - (routeCount);}
                else {cout <<"oh oh";}

                //cout << "waypoint( "<<point.y<<", "<<point.x<<")"<<endl;
                waypoints.push_back(point);
                 routeCount=0;

            }
    }
    //cout<<  "waypoints length in parse: " << waypoints.size()<<endl;
    indexToPoint(waypoints);
    return waypoints;
}


string SearchController::Astar( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish )
{
  cout<<"In Astar"<<endl;
    priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    int pqi; // pq index
    node* n0;
    node* m0;
    int i, j, x, y, xdx, ydy;
    char c;
    pqi=0;

    //Grid map 
    GridtoZone* mapx=GridtoZone::Instance();
    mapx->updatePaperMap();
    const int btmn=mapx->paperMap.getLength().y()*20; // horizontal size of the map
    const int btwmn=mapx->paperMap.getSize()(0); // vertical size size of the map

    cout<<"Map size x: "<<btmn<<endl;
    cout<<"Map size y: "<<btwmn<<endl;
    //Eigen::Vector2d * positionPtr;
    Position positionPtr;

    const int dir=8; // number of possible directions to go at any position
    int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
    int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

    const int n=btmn; // horizontal size of the map
    const int m=btwmn; // vertical size size of the map
    
    // const int n=10; // horizontal size of the map
    // const int m=10; // vertical size size of the map
    // static float map[n][m]={{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                   {-1, 0, 0, 0, 0, 0, 0, 0, 0, -1},
    //                   {-1, 0, 0, 0, 0, 0, 0, 0, 0, -1},
    //                   {-1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    //                   {-1, 2, 2, 2, 2, 2, 2, 2, 2, 0},
    //                   {-1, -1, -1, -1, 2, -1, -1, -1, -1, -2},
    //                   {-1, -1, -1, -1, 2, -1, -1, -1, -1, -1},
    //                   {-1, -1, -1, -1, 2, -1, -1, -1, -1, -1},
    //                   {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    //                   {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
    //                  };


    float closed_nodes_map[n][m]; // map of closed (tried-out) nodes
    float open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
    float dir_map[n][m]; // map of directions

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
    n0=new node(xStart, yStart, 0.0, 0.0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map
    delete n0;
    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest prioritymap[
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }
            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();           
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            //Todo: make adjustments since Position takes floats
            cout<<"xdx: "<<xdx<<" ydy: "<<ydy<<endl;
            positionPtr = Position(xdx,ydy);

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || mapx->paperMap.atPosition("elevation",positionPtr)==2 || mapx->paperMap.atPosition("elevation",positionPtr)==1|| closed_nodes_map[xdx][ydy]==1))
            // if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==2 || map[xdx][ydy]==1|| closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(), n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);


                m0->addcostToPriority(mapx->paperMap.atPosition("elevation",positionPtr));
                


                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0.0)
                {
                    bool neg=false;

                    open_nodes_map[xdx][ydy]=m0->getPriority();

                    pq[pqi].push(*m0);
                    delete m0;
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

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
                    pq[pqi].push(*m0); // add the better node instead
                    delete m0;
                    
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

vector<Point> SearchController::findPath(Point start, Point end){

  srand(time(NULL));

  vector<Point> waypoints;

  int xA, yA, xB, yB;
  pointToIndex(start, xA, yA);
  pointToIndex(end, xB, yB);

  cout<<"Start: "<<xA<<","<<yA<<endl;
  cout<<"Finish: "<<xB<<","<<yB<<endl;
  // get the route
  clock_t timeStart = clock();

  string route=Astar(xA, yA, xB, yB);

  if(route=="") cout<<"An empty route generated!"<<endl;
  clock_t timeEnd = clock();
  float time_elapsed = float(timeEnd - timeStart);
  cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
  cout<<"Route:"<<endl;
  cout<<route<<endl<<endl;

    

  waypoints=parseRoute(route, xA,yA);

  return waypoints;
}