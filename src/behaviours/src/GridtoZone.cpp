#include "GridtoZone.h"
//#include <angles/angles.h>
#include <ros/ros.h>

// needed for polygon shape
#include <geometry_msgs/PolygonStamped.h>

#include <math.h>
#include <cmath>

#include "SearchController.h"

using namespace Eigen;
using namespace grid_map;

GridtoZone::GridtoZone() { }

GridtoZone* GridtoZone::m_pInstance = NULL;

// OK //
GridtoZone* GridtoZone::Instance(){
	//cout << "Grid Instance Made" << endl;
	if (!m_pInstance){
		m_pInstance = new GridtoZone;
	}
	return m_pInstance;
}

// OK //
void GridtoZone::setGridMap(GridMap map) {
	Eigen::Vector2d origin(0,0);
	liveMap = map;

	mapLength = GridtoZone::Instance()->paperMap.getSize()(0);
  	mapWidth = 	GridtoZone::Instance()->paperMap.getSize()(1);
	
	if(verbose){
		cout << "Behavior Map Test" << endl;
		cout << "At (0,0): "<<map.atPosition("elevation", origin) <<endl;
	}
}

bool GridtoZone::otherRoverInZone(int zone, Position rover){
	int count = countRoversInZone(zone);
	Position center = getZonePosition(zone);
	double side = zonesize / 2;

/* No work

	Vector2d A( center.x() + side, center.y() + side );
	Vector2d B( center.x() + side, center.y() - side );
	Vector2d C( center.x() - side, center.y() - side );
	Vector2d D( center.x() - side, center.y() + side );

	Vector2d P( rover.x(), rover.y());

	Vector2d AB = A.cross(B);
	Vector2d AP = A.cross(P);
	Vector2d BC = B.cross(C);
	Vector2d BP = B.cross(P);


	double APdotAB = AP.adjoint()*AB;
	double ABdotAB = AB.adjoint()*AB;
	double BPdotBC = BP.adjoint()*BC;
	double BCdotBC = BC.adjoint()*BC;


// (0< APdotAB<ABdotAB)∧(0<APdotAD<ADdotAD)


	bool roverInZone = (0.0 <= APdotAB && APdotAB <= ABdotAB) && (0.0 <= BPdotBC && BPdotBC <= BCdotBC);

*/

	//https://math.stackexchange.com/a/190403
	float zoneside = zonesize;
	float zonearea = zonesize * zonesize;

	float x1 = center.x() + side;
	float x2 = center.x() + side;
	float x3 = center.x() - side;
	float x4 = center.x() - side;

	float y1 = center.y() + side;
	float y2 = center.y() - side;
	float y3 = center.y() - side;
	float y4 = center.y() + side;

	float b1 = sqrt ( pow (x1 - rover.x(), 2.0) + pow (y1 - rover.y(), 2.0) );
	float b2 = sqrt ( pow (x2 - rover.x(), 2.0) + pow (y2 - rover.y(), 2.0) );
	float b3 = sqrt ( pow (x3 - rover.x(), 2.0) + pow (y3 - rover.y(), 2.0) );
	float b4 = sqrt ( pow (x4 - rover.x(), 2.0) + pow (y4 - rover.y(), 2.0) );

	float u1 = zoneside + b1 + b2;
	float u2 = zoneside + b2 + b3;
	float u3 = zoneside + b3 + b4;
	float u4 = zoneside + b4 + b1;

	float a1 = sqrt( u1 * (u1 - zoneside) * (u1 - b1) * (u1 - b2) );
	float a2 = sqrt( u2 * (u2 - zoneside) * (u2 - b2) * (u2 - b3) );
	float a3 = sqrt( u3 * (u3 - zoneside) * (u3 - b3) * (u3 - b4) );
	float a4 = sqrt( u4 * (u4 - zoneside) * (u4 - b4) * (u4 - b1) );

	float area = a1 + a2 + a3 + a4;

	bool roverInZone = comparefloats(zonearea, area, .5);
	if (roverInZone){
		count--;
	}

	return count >= 1;
}


int GridtoZone::countRoversInZone(int zone){
	getZonePosition(zone);
	return countInSection(getZonePosition(zone),zonesize,ROVER);
}

// forgot what this is suppose to do.
bool GridtoZone::inZone(Position pos){
	return false;
}


// OK //
Position GridtoZone::getZonePosition(int zoneindex){
	double x = zonesize/2;
	double y = zonesize/2;

	int counter = 1;
	int maxcount = 1;
	int count = 0;
	bool turn = true;

	int distance = zonesize;
	// I couldn't get the other Direction from Spiral to work.
	GridDirection direction = West;

	for (int i = 0; i < zoneindex; i++){
		if(counter % 2 == 0 && counter > 0) {
			maxcount++;
			counter = 0;
		}
		switch(direction)
		{
			case North:
				y += distance;
				if(turn){
					direction = West;
					turn = false;
				}
				break;
			case East:
				x += distance;
				if(turn){
					direction = North;
					turn = false;
				}
				break;
			case South:
				y -= distance;
				if(turn){
					direction = East;
					turn = false;
				}
				break;
			case West:
				x -= distance;
				if(turn){
					direction = South;
					turn = false;
				}
				break;
			default:
				break;
		}
		count++;
		if(maxcount == count) {
			count = 0;
			counter++;
			turn = true;
		}
	}

	Position position = Position(x,y);

	if (positionverbose){
		std::cout<<  "Center of zone" << std::endl;
	    std::cout<<  "X: " << x << "  Y: " << y << std::endl;
	}

	return position;
}


void GridtoZone::updatePaperMap(){
	paperMap = liveMap;
}

int GridtoZone::countInSection(Position center, double length, float value){
	float values[] = {value};
	return countInSection(center,length,values,1);
}

int GridtoZone::countInSection(Position center, double length, float values[], int arrcount){
	double side = length / 2;

	grid_map::Polygon polygon;
	polygon.setFrameId(paperMap.getFrameId());
	// TopRight, BottomRight, TopLeft, BottomLeft
	polygon.addVertex(Position( center.x() + side, center.y() + side ));
	polygon.addVertex(Position( center.x() + side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() + side ));

	if (positionverbose){
		std::cout<<  "Corners of zone" << std::endl;	
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() + side << std::endl;
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() + side << std::endl;
	}
 
	int count = 0;
	int count2 = 0;

	for (grid_map::PolygonIterator iterator(paperMap, polygon); !iterator.isPastEnd(); ++iterator) {
		// ask what each value is with Port to check with.
		// ask about the layer
		if (positionverbose){
			//cout << "The value at index " << (*iterator).transpose() << " is " << paperMap.at("elevation", *iterator) << endl;
		}

		float mapValue = paperMap.at("elevation", *iterator);
		count2++;

		for (int i =0; i < arrcount; i++){
//			if (comparefloats(mapValue,values[i],0.5)){
			if (mapValue == values[i]){
				count++;
				break;
			}
		}
	}
	if (positionverbose){
		std::cout<<  "Total itr count: " <<  count2 << std::endl;	
		std::cout<<  "Total value count: " <<  count << std::endl;	
	}
	return count;
}







double GridtoZone::percentInSection(Position center, double length, float values[], int arrcount){
	double side = length / 2;

	grid_map::Polygon polygon;
	polygon.setFrameId(paperMap.getFrameId());
	// TopRight, BottomRight, TopLeft, BottomLeft
	polygon.addVertex(Position( center.x() + side, center.y() + side ));
	polygon.addVertex(Position( center.x() + side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() - side ));
	polygon.addVertex(Position( center.x() - side, center.y() + side ));

	if (positionverbose){
		std::cout<<  "Corners of zone" << std::endl;	
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() + side << std::endl;
	    std::cout<<  "X: " << center.x() + side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() - side << std::endl;
	    std::cout<<  "X: " << center.x() - side << "  Y: " << center.y() + side << std::endl;
	}


	int count = 0;
	int totalcount = 0;

	for (grid_map::PolygonIterator iterator(paperMap, polygon); !iterator.isPastEnd(); ++iterator) {
		totalcount++;

		float mapValue = paperMap.at("elevation", *iterator);

		for (int i =0; i < arrcount; i++){
			if (comparefloats(mapValue,values[i], 0.5)){
				count++;
				break;
			}
		}
	}

	if (positionverbose){
		std::cout<<  "Count stuff" << std::endl;	
	    std::cout<<  "count: " << count << std::endl;
	    std::cout<<  "totalcount: " << totalcount << std::endl;
	}

	return 1.0*count/totalcount;
}











double GridtoZone::percentOfZoneExplored(int zoneindex){
	return percentOfSectionExplored(getZonePosition(zoneindex),zonesize);
}

double GridtoZone::percentOfSectionExplored(Position center, double length){
	double sectionsize = (length / celldivision) * (length / celldivision);
	double sectionlength = (length / celldivision);
	int count = countInSection(center, sectionlength, floorvalues, 3);
	count = countInSection(center, length, allvalues, 7);
	return count/sectionsize;
}


double GridtoZone::percentOfZoneDiscovered(int zoneindex){
	return percentOfSectionDiscovered(getZonePosition(zoneindex),zonesize);
}

double GridtoZone::percentOfSectionDiscovered(Position center, double length){
//	double sectionsize = (length / celldivision) * (length / celldivision);
//	double sectionlength = (length / celldivision);

//	int count = countInSection(center, sectionlength, discorvedvalues,2);

//	int count = countInSection(center, sectionlength, arr,1);

	double per = percentInSection(center, length, discorvedvalues, 2);

	
//	cout << "percentInSection 1: "<< (count/sectionsize) <<endl;
//	cout << "percentInSection: "<< per <<endl;



	return per;
}

// Ambrosio do this
vector<Point> GridtoZone::shortestPath(Point start, Point end){
	// do something
	vector<Point> waypoints;

	waypoints=findPath(start, end);
	return waypoints;
}


int GridtoZone::ClaimZone(int zone){
	// check if available
	zoneclaimed = zone;
	return zone;
	// else
	// return -1;
}

double GridtoZone::percentOfTest(){

	Position center = Position(0,0);
	Position zone0 = Position(1.75,1.75);
	Position pos_1count = Position(2.20,2.20); // expect 0.0002040816


	float values[] = {MAT};
//	int count = percentInSection(Position(0,0),length,values,1);
	return percentInSection(pos_1count,zonesize,values,1);
}

int GridtoZone::countOfTest(){

	Position center = Position(0,0);
	Position zone0 = Position(1.75,1.75);
	Position pos_1count = Position(2.0,2.0); // expect 0.0002040816

	float values[] = {MAT};

	return countInSection(pos_1count, zonesize, MAT);
}


bool GridtoZone::comparefloats(float a, float b, float acc){
	float c = b - a;
	return (c <= acc && c >= -1 * acc);
}

bool GridtoZone::obstaclesInZone(Position pos, float sectionlength){
	return countInSection(pos, sectionlength, wallvalues, 3) > 0;
}

bool GridtoZone::pathClear(float x1, float y1, float x2, float y2){
	grid_map::Polygon polygonPath;
	polygonPath.setFrameId(paperMap.getFrameId());

	double mapValue = -10.00;

	double distX = x2 - x1;
	double distY = y2 - y1;
	double distTotal = sqrt((distX*distX) + (distY*distY));
	float rad = std::atan2(x2 - x1, y2 - y1);
	float angle = rad * 180/pi;
	double botLeftX=  0			, botRightX=  0;
	double botLeftY= -ROVERHALF	, botRightY=  ROVERHALF;
	double topLeftX =  distTotal, topRightX= distTotal;
	double topLeftY = -ROVERHALF, topRightY= ROVERHALF;

	double rotateCamTopAngleRX = (topRightX * cos(rad) - topRightY * sin(rad));
	double rotateCamTopAngleRY = (topRightX * sin(rad) + topRightY * cos(rad));
	double rotateCamTopAngleLX = (topLeftX * cos(rad) - topLeftY * sin(rad));
	double rotateCamTopAngleLY = (topLeftX * sin(rad) + topLeftY * cos(rad));
			
	double rotateCamBotAngleRX = (botRightX * cos(rad) - botRightY * sin(rad));
	double rotateCamBotAngleRY = (botRightX * sin(rad) + botRightY * cos(rad));
	double rotateCamBotAngleLX = (botLeftX * cos(rad) - botLeftY * sin(rad));
	double rotateCamBotAngleLY = (botLeftX * sin(rad) + botLeftY * cos(rad));	

	polygonPath.addVertex(Position(rotateCamBotAngleRX + x1, rotateCamBotAngleRY + y1));
	polygonPath.addVertex(Position(rotateCamTopAngleRX + x2, rotateCamTopAngleRY + y2));
	polygonPath.addVertex(Position(rotateCamTopAngleLX + x2, rotateCamTopAngleLY + y2));
	polygonPath.addVertex(Position(rotateCamBotAngleLX + x1, rotateCamBotAngleLY + y1));
	
	for(grid_map::PolygonIterator iterator(paperMap, polygonPath); !iterator.isPastEnd(); ++iterator) {
		mapValue = paperMap.at("elevation", *iterator);
		if(mapValue == WALL){
			return false;
		}
	}	
	return true;
}

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b){
  return a.getPriority() > b.getPriority();
}

//converts a the x value of a Point to a row value of index
int GridtoZone::pointXToIndex(float X){
	//cout<<"pointXToIndex x "<<X<<" converted"<<(mapWidth/2 - 20*X)<<endl;
  return (mapWidth/2 - 20*X);
}

//converts a the y value of a Point to a column value of index
int GridtoZone::pointYToIndex(float Y){
	//cout<<"pointXToIndex y "<<Y<<" converted"<<(mapWidth/2 - 20*Y)<<endl;
  return (mapLength/2 - 20*Y);
}

//converts the row value of an index to a x value of a Point
float GridtoZone::indexToPointX(int i){
	//cout<<"indexToPointX x "<<i<<" converted"<<(mapWidth/2 - i)/20.0<<endl;
  return (mapWidth/2 - i)/20.0;
}

//converts the column value of an index to a y value of a Point
float GridtoZone::indexToPointY(int j){
	//cout<<"indexToPointY y "<<j<<" converted"<<(mapWidth/2 - j)/20.0<<endl;
  return (mapLength/2 - j)/20.0;
}

// Parses the given route in string form int waypoints
vector<Point> GridtoZone::parseRoute(string route, float x, float y){
  std::vector<Point> waypoints;

  int routeLength = route.length();
  string routeArray[routeLength];

  // set point to the original starting Point
  Point point;
  point.x=(floor(x*20))/20.0;
  point.y=(floor(y*20))/20.0;
  
  const float celldivision=0.05;
  
  float distance = 0.0;


  // Makes route into array of characters
  int count=0;
  stringstream ssin(route);
  while (ssin.good() && count < routeLength){
    routeArray[count]=ssin.get();
    count++;
  }
  cout << "Parse Route start Point ("<<x<<", "<<y<<") " <<endl; 
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

string GridtoZone::Astar( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish )
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



    GridtoZone::Instance()->updatePaperMap(); // Updates Grid map 
  
    const int n = mapLength; // horizontal size of the map x-axis
    const int m = mapWidth; // vertical size size of the map y-axis

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
            
            float map_value = GridtoZone::Instance()->paperMap.atPosition("elevation",currentPosition);// Gets the value of Gridmap at current Position
            //cout<<map_value<< endl;
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
                  childNode->addcostToPriority(2.0); // Gives sonar higher priority: 2
                } 
                else if(map_value == REVEALED){
                  childNode->addcostToPriority(0.0); // Gives revealed higher priority: 0
                }
                else if(map_value == FOG){
                  childNode->addcostToPriority(10.0); // Gives Fog lower priority : 10
                }
                else if(map_value == MAT){
                  childNode->addcostToPriority(21.0); // Gives MAT lower priority : 21
                }
                else if(map_value == BUFFER){
                  childNode->addcostToPriority(40.0); // Gives BUFFER lower priority : 40
                }
                else {
                  childNode->addcostToPriority(10000); // All other values should be 10000 just put for precaution in case of error
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

vector<Point> GridtoZone::findPath(Point start, Point end){


  vector<Point> waypoints;

  // Switches x and y values as well as multiplies y value by -1
  // This is done to account for Gridmap's switched Y axis
  float xA = -1 * start.y;
  float yA = start.x;

  float xB = -1 * end.y;
  float yB = end.x;


  // Sets GridMap to map and updates it to most recent map.
  GridtoZone::Instance()->updatePaperMap();
  mapLength = GridtoZone::Instance()->paperMap.getSize()(0);
  mapWidth = GridtoZone::Instance()->paperMap.getSize()(1);

  // Converts x and y float values of Points to int appropriate index values
  // This is done because a star uses arrays
  int xStart = pointXToIndex(xA);
  int yStart = pointYToIndex(yA);
  int xFinish = pointXToIndex(xB);
  int yFinish = pointYToIndex(yB);

  cout<< " start (" << xStart<< ", "<<yStart<<") "<< " finish (" << xFinish<< ", "<<yFinish<<") "<<endl;

  // Give Astar start indexes and finish indexes
  // Returned will be the fastest route astar found put together as a string
  string route=Astar(xStart, yStart, xFinish, yFinish);


  if(route=="") cout<<"An empty route generated!"<<endl;
  
  //kept for debuging purposes prints the route string returned from astar
  // cout<<"Route:"<<endl;
  // cout<<route<<endl<<endl;

  // Parse the route and get waypoints 
  waypoints=parseRoute(route, start.x,start.y);

  return waypoints;
}
