#include <ros/ros.h>

// ROS libraries
#define _USE_MATH_DEFINES
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include <sstream>
#include <Eigen/Dense>
#include <unistd.h>
#include <math.h>

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <grid_map_msgs/GridMap.h>

#include <signal.h>

#include <exception>

using namespace std;
using namespace grid_map;
using namespace Eigen;

float heartbeat_publish_interval = 2;
const float CELLDIVISION = 0.05;
const float ROVERHALF = 0.20;
const float ROVPLUSCELL = ROVERHALF + 2*CELLDIVISION;
//GRID POINT TYPE
const double FOG 	= -10.00;
const double REVEALED 	= 0.00;
const double MAT 	= 1.0;
const double MULTICUBES	= 2.0;
const double SONAR 	= 3.0;
const double ROVER 	= 10.0;
const double BUFFER 	= 15.0;
const double WALL 	= 20.0;

/*----------------MAKE SURE TO TURN FALSE WHEN YOU ARE NOT RUNNING THE SIMULATION----------------*/
/*->->->->->->->->->*/	bool SIMMODE = false;	/*<-<-<-<-<-<-<-<-<-<-<-<-<-<-*/
/*----------------MAKE SURE TO TURN FALSE WHEN YOU ARE NOT RUNNING THE SIMULATION----------------*/

//Publisher
ros::Publisher gridswarmPublisher;
ros::Publisher heartbeatPublisher;
ros::Publisher polygonPublisher;
//Subscriber
ros::Subscriber roverNameSubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber sonarLeftSubscriber	,sonarLeftSubscriber1	,sonarLeftSubscriber2	;
ros::Subscriber sonarCenterSubscriber	,sonarCenterSubscriber1	,sonarCenterSubscriber2	;
ros::Subscriber sonarRightSubscriber	,sonarRightSubscriber1	,sonarRightSubscriber2	;
ros::Subscriber odometrySubscriber	,odometrySubscriber1	,odometrySubscriber2	;
ros::Subscriber orntnSubscriber		,orntnSubscriber1	,orntnSubscriber2	;

//Timer
ros::Timer publish_heartbeat_timer;

std::string publishedName;
//Global
  const float pi = std::acos(-1);
  const int namesArrSize=6;
  string namesArr[namesArrSize] = {"test","test","test","test","test","test"};//"achilles","ajax","aeneas"
  int arrCount = 0;
  float sleft[namesArrSize];
  float scenter[namesArrSize];
  float sright[namesArrSize];
  float orntn[namesArrSize];
  float xpos[namesArrSize];
  float ypos[namesArrSize];
  char host[128];
  bool firstgo = true;
  bool modeAuto = false;
  bool noSonar = true;
  bool o0once = true, o1once = true, o2once = true;
  float x0offset = 0, x1offset = 0, x2offset = 0;
  float y0offset = 0, y1offset = 0, y2offset = 0;

  bool cs_testing = true;

using namespace std;
using namespace grid_map;
using namespace ros;
//using namespace Eigen;

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

void roverNameHandler(const std_msgs::String& message);
void modeHandler(const std_msgs::UInt8::ConstPtr& message);

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
  void orntnHandler(const std_msgs::Float32& message);
  void sonarHandlerLeft(const sensor_msgs::Range::ConstPtr& sonarLeft);
  void sonarHandlerCenter(const sensor_msgs::Range::ConstPtr& sonarCenter);
  void sonarHandlerRight(const sensor_msgs::Range::ConstPtr& sonarRight);
void odometryHandler1(const nav_msgs::Odometry::ConstPtr& message);
  void orntnHandler1(const std_msgs::Float32& message);
  void sonarHandlerLeft1(const sensor_msgs::Range::ConstPtr& sonarLeft);
  void sonarHandlerCenter1(const sensor_msgs::Range::ConstPtr& sonarCenter);
  void sonarHandlerRight1(const sensor_msgs::Range::ConstPtr& sonarRight);
void odometryHandler2(const nav_msgs::Odometry::ConstPtr& message);
  void orntnHandler2(const std_msgs::Float32& message);
  void sonarHandlerLeft2(const sensor_msgs::Range::ConstPtr& sonarLeft);
  void sonarHandlerCenter2(const sensor_msgs::Range::ConstPtr& sonarCenter);
  void sonarHandlerRight2(const sensor_msgs::Range::ConstPtr& sonarRight);

int main(int argc, char **argv){
  gethostname(host, sizeof (host));
  string hostname(host);
  
 if (argc >= 2) {
   publishedName = argv[1];
   cout << "Welcome to the world of tomorrow " << publishedName
        << "!  GridMapSwarm turnDirectionule started." << endl;
 } else {
   publishedName = hostname;
   cout << "No Name Selected. Default is: " << publishedName << endl;
 }
  
  // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
  ros::init(argc, argv, (hostname + "_GRID"), ros::init_options::NoSigintHandler);
  ros::NodeHandle gNH;
//SUBSCRIBER
  roverNameSubscriber = gNH.subscribe(("/chainName"), 1, roverNameHandler);
  modeSubscriber = gNH.subscribe((publishedName + "/mode"), 1, modeHandler);


//PUBLISH
  heartbeatPublisher = gNH.advertise<std_msgs::String>((publishedName + "/gridSwarm/heartbeat"), 1,true);
  publish_heartbeat_timer = gNH.createTimer(ros::Duration(heartbeat_publish_interval),publishHeartBeatTimerEventHandler);
 polygonPublisher = gNH.advertise<geometry_msgs::PolygonStamped>("/polygon", 1, true);
  ros::Rate rate(30.0);
  do{
  	ros::spinOnce();
  	rate.sleep();
  }while(modeAuto == false);

  if (publishedName != namesArr[0]){
	cout << publishedName << " not first listed. Ending Grid-Map" <<endl;
	return 0;
  }

  cout << publishedName << " was Listed first listed. Starting Grid-Map" <<endl;
  gridswarmPublisher = gNH.advertise<grid_map_msgs::GridMap>("/grid_map", 1);
  for(int i = 0; i < namesArrSize; i++){
//	cout << "namesArr[" << i << "] =" << namesArr[i] <<":Start Loop"<<endl;
	if (namesArr[i] != "test"){
		arrCount = i;
		publishedName = namesArr[i];
		cout << "Entered Subscriber loop: "<< publishedName <<"["<<arrCount<<"]"<< endl;
		if (i == 0){
			odometrySubscriber = gNH.subscribe((publishedName + "/odom"), 10, odometryHandler);
			orntnSubscriber = gNH.subscribe((publishedName + "/filtered_orientation"), 10, orntnHandler);
			sonarLeftSubscriber = gNH.subscribe((publishedName + "/sonarLeft"), 10, sonarHandlerLeft);
			sonarCenterSubscriber = gNH.subscribe((publishedName + "/sonarCenter"), 10, sonarHandlerCenter);
			sonarRightSubscriber = gNH.subscribe((publishedName + "/sonarRight"), 10, sonarHandlerRight);
		}else if(i == 1){
			odometrySubscriber1 = gNH.subscribe((publishedName + "/odom"), 10, odometryHandler1);
			orntnSubscriber1 = gNH.subscribe((publishedName + "/filtered_orientation"), 10, orntnHandler1);
			sonarLeftSubscriber1 = gNH.subscribe((publishedName + "/sonarLeft"), 10, sonarHandlerLeft1);
			sonarCenterSubscriber1 = gNH.subscribe((publishedName + "/sonarCenter"), 10, sonarHandlerCenter1);
			sonarRightSubscriber1 = gNH.subscribe((publishedName + "/sonarRight"), 10, sonarHandlerRight1);
		}else if(i == 2){
			odometrySubscriber2 = gNH.subscribe((publishedName + "/odom"), 10, odometryHandler2);
			orntnSubscriber2 = gNH.subscribe((publishedName + "/filtered_orientation"), 10, orntnHandler2);
			sonarLeftSubscriber2 = gNH.subscribe((publishedName + "/sonarLeft"), 10, sonarHandlerLeft2);
			sonarCenterSubscriber2 = gNH.subscribe((publishedName + "/sonarCenter"), 10, sonarHandlerCenter2);
			sonarRightSubscriber2 = gNH.subscribe((publishedName + "/sonarRight"), 10, sonarHandlerRight2);
		}
	}//END OF IF STATEMENT
  }//END OF FOR LOOP
  cout << " + Exit Subscriber loop -"<< "- arrCount:"<<arrCount<< endl;

  // Create grid Rover Specific Map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(15.5, 15.5), CELLDIVISION);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));  

  while (ros::ok()) {
	ros::Time time = ros::Time::now();
	for(int count = arrCount; count >= 0; count--){
		//CAMERA and ROVER cover Area
		grid_map::Polygon polygon;
		polygon.setFrameId(map.getFrameId());
		//These points make an area of where the ROVER is
		double midLeftX=  ROVERHALF, midRightX=  ROVERHALF;
		double midLeftY= -ROVERHALF, midRightY=  ROVERHALF;      
		double botLeftX= -ROVERHALF, botRightX= -ROVERHALF;
		double botLeftY= -ROVERHALF, botRightY=  ROVERHALF;
		//These point create the area the CAMERA would see. Does not actually see what the camera sees, it just marks the area as seen.
		double topLeftX =  0.57+ROVERHALF, topRightX= 0.57+ROVERHALF;
		double topLeftY = -0.215         , topRightY= 0.215;
		double tipX = 0.60+ROVERHALF; 
		double tipY = 0;
		//Do the roatation math 
		//Mid points
		double rotateCamMLX = (midLeftX * cos(orntn[count]) - midLeftY * sin(orntn[count]));
		double rotateCamMLY = (midLeftX * sin(orntn[count]) + midLeftY * cos(orntn[count]));
		double rotateCamMRX = (midRightX * cos(orntn[count]) - midRightY * sin(orntn[count]));
		double rotateCamMRY = (midRightX * sin(orntn[count]) + midRightY * cos(orntn[count]));
		//Bottom points
		double rotateCamBRX = (botRightX * cos(orntn[count]) - botRightY * sin(orntn[count]));
		double rotateCamBRY = (botRightX * sin(orntn[count]) + botRightY * cos(orntn[count]));
		double rotateCamBLX = (botLeftX * cos(orntn[count]) - botLeftY * sin(orntn[count]));
		double rotateCamBLY = (botLeftX * sin(orntn[count]) + botLeftY * cos(orntn[count]));
		//Top points
		double rotateCamTLX = (topLeftX * cos(orntn[count]) - topLeftY * sin(orntn[count]));
		double rotateCamTLY = (topLeftX * sin(orntn[count]) + topLeftY * cos(orntn[count]));
		double rotateCamTRX = (topRightX * cos(orntn[count]) - topRightY * sin(orntn[count]));
		double rotateCamTRY = (topRightX * sin(orntn[count]) + topRightY * cos(orntn[count]));
		//Tip points
		double rotateCamPtTipx = (tipX * cos(orntn[count]) - tipY * sin(orntn[count]));
		double rotateCamPtTipy = (tipX * sin(orntn[count]) + tipY * cos(orntn[count]));
		//Add the points to a polygon
		polygon.addVertex(Position(rotateCamBLX + xpos[count], rotateCamBLY + ypos[count]));
		polygon.addVertex(Position(rotateCamMLX + xpos[count], rotateCamMLY + ypos[count]));
		polygon.addVertex(Position(rotateCamTLX + xpos[count], rotateCamTLY + ypos[count]));
		polygon.addVertex(Position(rotateCamPtTipx + xpos[count], rotateCamPtTipy + ypos[count]));
		polygon.addVertex(Position(rotateCamTRX + xpos[count], rotateCamTRY + ypos[count]));
		polygon.addVertex(Position(rotateCamMRX + xpos[count], rotateCamMRY + ypos[count]));
		polygon.addVertex(Position(rotateCamBRX + xpos[count], rotateCamBRY + ypos[count]));
		
		geometry_msgs::PolygonStamped message;
		grid_map::PolygonRosConverter::toMessage(polygon, message);		
		for(grid_map::PolygonIterator iterator(map,polygon); !iterator.isPastEnd(); ++iterator) 
		{
			if (map.at("elevation", *iterator) != WALL && map.at("elevation", *iterator) != MAT){
				map.at("elevation", *iterator) = REVEALED;
			}	
		}
		//OFFSET FOR SONAR
		if (noSonar == false){
			float fromCenterX = xpos[count] + ROVERHALF * cos(orntn[count]);
			float fromCenterY = ypos[count] + ROVERHALF * sin(orntn[count]);
			//CENTER SONAR
			bool overlap = false;
			float cx = (cos(orntn[count]) * scenter[count]) + fromCenterX;
			float cy = (sin(orntn[count]) * scenter[count]) + fromCenterY;
			Eigen::Vector2d c(cx,cy);
			Eigen::Vector2d start(fromCenterX, fromCenterY);
			for(grid_map::LineIterator iterator(map,start,c); !iterator.isPastEnd(); ++iterator) {
				if (map.at("elevation", *iterator) == FOG){
					map.at("elevation", *iterator) = SONAR;
				}
			}
			if (scenter[count] <= 2.0 && scenter[count] >= 0.2){
				for(int inner = 0; inner <= arrCount; inner++){
					float qx = xpos[inner];
					float qy = ypos[inner];;
					if (cx <= (qx + ROVPLUSCELL) && cx >= (qx - ROVPLUSCELL) && cy <= (qy + ROVPLUSCELL) && cy >= (qy - ROVPLUSCELL)){
						overlap = true;
					}
				}
				if (map.isInside(c) && overlap == false){
					grid_map::Polygon polc;
					double botLeftX= cx-0.30, botRightX= cx+0.30;
					double botLeftY= cy-0.30, botRightY= cy-0.30;
					double topLeftX= cx-0.30, topRightX= cx+0.30;
					double topLeftY= cy+0.30, topRightY= cy+0.30;
					polc.addVertex(Position(botLeftX, botLeftY));
					polc.addVertex(Position(topLeftX, topLeftY));
					polc.addVertex(Position(topRightX,topRightY));
					polc.addVertex(Position(botRightX,botRightY));
					
					for(grid_map::PolygonIterator iterator(map, polc); !iterator.isPastEnd(); ++iterator) {
						if (map.at("elevation", *iterator) != WALL){
							map.at("elevation", *iterator) = BUFFER;
						}
					}	
					map.atPosition("elevation", c) = WALL;
				}
			}
			//LEFT SONAR
			overlap = false;
			float lx = (cos((pi/6.7)+orntn[count]) * sleft[count]) + fromCenterX;
			float ly = (sin((pi/6.7)+orntn[count]) * sleft[count]) + fromCenterY;
			Eigen::Vector2d l(lx,ly);
			for(grid_map::LineIterator iterator(map,start,l); !iterator.isPastEnd(); ++iterator) {
				if (map.at("elevation", *iterator) == FOG){
					map.at("elevation", *iterator) = SONAR;
				}
			}
			if (sleft[count] <= 2.0 && sleft[count] >= 0.2){
				for(int inner = 0; inner <= arrCount; inner++){
					float qx = xpos[inner];
					float qy = ypos[inner];
					if (lx <= (qx + ROVPLUSCELL) && lx >= (qx - ROVPLUSCELL) && ly <= (qy + ROVPLUSCELL) && ly >= (qy - ROVPLUSCELL)){
						overlap = true;
					}
				}
				if (map.isInside(l) && overlap == false){
					grid_map::Polygon poll;
					double botLeftX= lx-0.30, botRightX= lx+0.30;
					double botLeftY= ly-0.30, botRightY= ly-0.30;
					double topLeftX= lx-0.30, topRightX= lx+0.30;
					double topLeftY= ly+0.30, topRightY= ly+0.30;
					poll.addVertex(Position(botLeftX, botLeftY));
					poll.addVertex(Position(topLeftX, topLeftY));
					poll.addVertex(Position(topRightX,topRightY));
					poll.addVertex(Position(botRightX,botRightY));
					
					for(grid_map::PolygonIterator iterator(map, poll); !iterator.isPastEnd(); ++iterator) {
						if (map.at("elevation", *iterator) != WALL){
							map.at("elevation", *iterator) = BUFFER;
						}
					}	
					map.atPosition("elevation", l) = WALL;
				}
			}
			//RIGHT SONAR
			overlap = false;
			float rx = (cos(-1*(pi/6.7)+orntn[count]) * sright[count]) + fromCenterX;
			float ry = (sin(-1*(pi/6.7)+orntn[count]) * sright[count]) + fromCenterY;
			Eigen::Vector2d r(rx,ry);
			for(grid_map::LineIterator iterator(map,start,r); !iterator.isPastEnd(); ++iterator) {
				if (map.at("elevation", *iterator) == FOG){
					map.at("elevation", *iterator) = SONAR;
				}
			}
			if (sright[count] <= 2.0 && sright[count] >= 0.2){
				for(int inner = 0; inner <= arrCount; inner++){
					float qx = xpos[inner];
					float qy = ypos[inner];
					if (rx <= (qx + ROVPLUSCELL) && rx >= (qx - ROVPLUSCELL) && ry <= (qy + ROVPLUSCELL) && ry >= (qy - ROVPLUSCELL)){
						overlap = true;
					}
				}
				if (map.isInside(r) && overlap == false){
					grid_map::Polygon polr;
					double botLeftX= rx-0.30, botRightX= rx+0.30;
					double botLeftY= ry-0.30, botRightY= ry-0.30;
					double topLeftX= rx-0.30, topRightX= rx+0.30;
					double topLeftY= ry+0.30, topRightY= ry+0.30;
					polr.addVertex(Position(botLeftX, botLeftY));
					polr.addVertex(Position(topLeftX, topLeftY));
					polr.addVertex(Position(topRightX,topRightY));
					polr.addVertex(Position(botRightX,botRightY));
					
					for(grid_map::PolygonIterator iterator(map, polr); !iterator.isPastEnd(); ++iterator) {
						if (map.at("elevation", *iterator) != WALL){
							map.at("elevation", *iterator) = BUFFER;
						}
					}	
					map.atPosition("elevation", r) = WALL;
				}
			}
		}//END OF SONAR BOOLEAN
	}//END OF FOR LOOP
	if (firstgo == true){
		cout << "Creating the initial FOG" << endl;
		//CREATE FOG
		for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
			Position position;
			map.getPosition(*it, position);
			map.at("elevation", *it) = FOG;
		}//AREA AROUND MAT
		for (float length = -1.00; length <= 1.00;){
			for(float width = -1.00; width <= 1.00;){
				Eigen::Vector2d mat(length,width);
				map.atPosition("elevation", mat) = REVEALED;
				width += CELLDIVISION;
			}
			length += CELLDIVISION;
		}
	}
	firstgo = false;
	//CENTER MAT being Discovered
	for (float length = -0.50; length <= 0.50;){
		for(float width = -0.50; width <= 0.50;){
			Eigen::Vector2d mat(length,width);
			map.atPosition("elevation", mat) = MAT;
			width += CELLDIVISION;
		}
		length += CELLDIVISION;
	}
	//ROVERS
	for(int count = arrCount; count >= 0; count--){
		float qx = xpos[count];
		float qy = ypos[count];
		Eigen::Vector2d q(qx,qy);
		if (map.isInside(q)){
			map.atPosition("elevation", q) = ROVER;
		}
	}
	if (noSonar == true){
		for(int count = arrCount; count >= 0; count--){
			float x = xpos[count];
			float y = ypos[count];
			//cout<<count<<"("<<x<<","<<y<<")"<<endl;
			if (x > 2.00 || x < -2.00 || y > 2.00 || y < -2.00){
				noSonar = false;
			}
		}
	}
	// Publish grid map.
	map.setTimestamp(time.toNSec());
	grid_map_msgs::GridMap message;
	GridMapRosConverter::toMessage(map, message);
	gridswarmPublisher.publish(message);
//	ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
	// Wait for next cycle.
	ros::spinOnce();
	rate.sleep();
  }//END OF ROS OK





return 0;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&){
	std_msgs::String msg;
	msg.data = "";
	heartbeatPublisher.publish(msg);
}

void sonarHandlerLeft(const sensor_msgs::Range::ConstPtr& sonarLeft) {
	float simoffsetLeft = 0;
	if(SIMMODE == true){
		simoffsetLeft = ((sonarLeft->range)/cos(pi/6.7)) - (sonarLeft->range); 
	}
	sleft[0] = sonarLeft->range + simoffsetLeft;
}
void sonarHandlerLeft1(const sensor_msgs::Range::ConstPtr& sonarLeft) {
	float simoffsetLeft = 0;
	if(SIMMODE == true){
		simoffsetLeft = ((sonarLeft->range)/cos(pi/6.7)) - (sonarLeft->range); 
	}
	sleft[1] = sonarLeft->range + simoffsetLeft;
}
void sonarHandlerLeft2(const sensor_msgs::Range::ConstPtr& sonarLeft) {
	float simoffsetLeft = 0;
	if(SIMMODE == true){
		simoffsetLeft = ((sonarLeft->range)/cos(pi/6.7)) - (sonarLeft->range); 
	}
	sleft[2] = sonarLeft->range + simoffsetLeft;
}

void sonarHandlerCenter(const sensor_msgs::Range::ConstPtr& sonarCenter) {
	scenter[0]= sonarCenter->range;
}
void sonarHandlerCenter1(const sensor_msgs::Range::ConstPtr& sonarCenter) {
	scenter[1]= sonarCenter->range;
}
void sonarHandlerCenter2(const sensor_msgs::Range::ConstPtr& sonarCenter) {
	scenter[2]= sonarCenter->range;
}

void sonarHandlerRight(const sensor_msgs::Range::ConstPtr& sonarRight) {
	float simoffsetRight = 0;
	if(SIMMODE == true){
		simoffsetRight = ((sonarRight->range)/cos(pi/6.7)) - (sonarRight->range); 
	}
	sright[0] = sonarRight->range + simoffsetRight;
}
void sonarHandlerRight1(const sensor_msgs::Range::ConstPtr& sonarRight) {
	float simoffsetRight = 0;
	if(SIMMODE == true){
		simoffsetRight = ((sonarRight->range)/cos(pi/6.7)) - (sonarRight->range); 
	}
	sright[1] = sonarRight->range + simoffsetRight;
}
void sonarHandlerRight2(const sensor_msgs::Range::ConstPtr& sonarRight) {
	float simoffsetRight = 0;
	if(SIMMODE == true){
		simoffsetRight = ((sonarRight->range)/cos(pi/6.7)) - (sonarRight->range); 
	}
	sright[2] = sonarRight->range + simoffsetRight;
}

void orntnHandler(const std_msgs::Float32& message) {
	double point;
	orntn[0] = message.data;
	if (o0once == true && SIMMODE == false){
		if (orntn[0] <= -3.14 + pi/24 && orntn[0] >= -3.14 - pi/24){
			point = 3.14;
		}else{
			for (point = 3.14; point > -3.14; point -= pi/12){
				//cout <<"0Point:"<<point<<endl;
				if (orntn[0] <= point + pi/24 && orntn[0] >= point - pi/24){
					break;
				}
			}
		}
		x0offset = -1.2 * cos(point);
		y0offset = -1.2 * sin(point);
		o0once = false;
	}
}
void orntnHandler1(const std_msgs::Float32& message) {
	double point;
	orntn[1] = message.data;
	if (o1once == true && SIMMODE == false){
		if (orntn[2] <= -3.14 + pi/24 && orntn[2] >= -3.14 - pi/24){
			point = 3.14;
		}else{
			for (point = 3.14; point > -3.14; point -= pi/12){
				//cout <<"1Point:"<<point<<endl;
				if (orntn[1] <= point + pi/24 && orntn[1] >= point - pi/24){
					break;
				}
			}
		}
		x1offset = -1.2 * cos(point);
		y1offset = -1.2 * sin(point);
		o1once = false;
	}
}
void orntnHandler2(const std_msgs::Float32& message) {
	double point;
	orntn[2] = message.data;
	if (o2once == true && SIMMODE == false){
		if (orntn[2] <= -3.14 + pi/24 && orntn[2] >= -3.14 - pi/24){
			point = 3.14;
		}else{
			for (point = 3.14; point > -3.14; point -= pi/12){
				//cout <<"2Point:"<<point<<endl;
				if (orntn[2] <= point + pi/24 && orntn[2] >= point - pi/24){
					break;
				}
			}
		}
		x2offset = -1.2 * cos(point);
		y2offset = -1.2 * sin(point);
		o2once = false;
	}
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	xpos[0] = message->pose.pose.position.x + x0offset;
	ypos[0] = message->pose.pose.position.y + y0offset;
}
void odometryHandler1(const nav_msgs::Odometry::ConstPtr& message) {
	xpos[1] = message->pose.pose.position.x + x1offset;
	ypos[1] = message->pose.pose.position.y + y1offset;
}
void odometryHandler2(const nav_msgs::Odometry::ConstPtr& message) {
	xpos[2] = message->pose.pose.position.x + x2offset;
	ypos[2] = message->pose.pose.position.y + y2offset;
}

void roverNameHandler(const std_msgs::String& message){
	std::string list= message.data;
	for(int i=0;i<namesArrSize; i++){
		namesArr[i] = "test";
	}
	for(int i=0;i<namesArrSize; i++){
		if(namesArr[i].compare("test") == 0){
			int index = list.find(",");
			namesArr[i] = list.substr(0,index);
			list.erase(0,index+1);
			cout << "GRIDSWARM:namesArray "<<i<<": " << namesArr[i] << endl;
			if (list.empty()){
				i = namesArrSize;
			}
		}
	}
}

void modeHandler(const std_msgs::UInt8::ConstPtr& message) {
	int currentMode = message->data;
	if(currentMode == 2 || currentMode == 3) {
		modeAuto = true;
		
	}
}
