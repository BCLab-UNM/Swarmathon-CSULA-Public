#include <ros/ros.h>

// ROS libraries
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

// ROS messages
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <grid_map_msgs/GridMap.h>

#include <signal.h>

#include <exception>

using namespace std;

float heartbeat_publish_interval = 2;
const float CELLDIVISION = 0.05;
const float ROVERHALF = 0.20;
const float WALL = 1;
const float FOG = -0.5;
const float ROVER = 0.5;
const float DISCOVER = 0.0;

/*----------------MAKE SURE TO TURN FALSE WHEN YOU ARE NOT RUNNING THE SIMULATION----------------*/
/*->->->->->->->->->*/	bool SIMMODE = false;	/*<-<-<-<-<-<-<-<-<-<-<-<-<-<-*/
/*----------------MAKE SURE TO TURN FALSE WHEN YOU ARE NOT RUNNING THE SIMULATION----------------*/

//Publisher
ros::Publisher gridswarmPublisher;
ros::Publisher test;
ros::Publisher heartbeatPublisher;
//Subscriber
ros::Subscriber roverNameSubscriber;
ros::Subscriber sonarLeftSubscriber	,sonarLeftSubscriber1	,sonarLeftSubscriber2	;
ros::Subscriber sonarCenterSubscriber	,sonarCenterSubscriber1	,sonarCenterSubscriber2	;
ros::Subscriber sonarRightSubscriber	,sonarRightSubscriber1	,sonarRightSubscriber2	;
ros::Subscriber odometrySubscriber	,odometrySubscriber1	,odometrySubscriber2	;

//Timer
ros::Timer publish_heartbeat_timer;

std::string publishedName;
//Global
  const double pi = std::acos(-1);
  const int namesArrSize=6;
  string namesArr[namesArrSize] = {"test","test","test","test","test","test"};//"achilles","ajax","aeneas"
  int currentMode = 0;
  int arrCount = 0;
  float sleft[namesArrSize];
  float scenter[namesArrSize];
  float sright[namesArrSize];
  float orntn[namesArrSize];
  float xpos[namesArrSize];
  float ypos[namesArrSize];
  char host[128];
  bool firstgo = true;
  bool o0once = true, o1once = true, o2once = true;
  float x0offset = 0, x1offset = 0, x2offset = 0;
  float y0offset = 0, y1offset = 0, y2offset = 0;
using namespace std;
using namespace grid_map;
using namespace Eigen;

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

void roverNameHandler(const std_msgs::String& message);

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
  void sonarHandlerLeft(const sensor_msgs::Range::ConstPtr& sonarLeft);
  void sonarHandlerCenter(const sensor_msgs::Range::ConstPtr& sonarCenter);
  void sonarHandlerRight(const sensor_msgs::Range::ConstPtr& sonarRight);
void odometryHandler1(const nav_msgs::Odometry::ConstPtr& message);
  void sonarHandlerLeft1(const sensor_msgs::Range::ConstPtr& sonarLeft);
  void sonarHandlerCenter1(const sensor_msgs::Range::ConstPtr& sonarCenter);
  void sonarHandlerRight1(const sensor_msgs::Range::ConstPtr& sonarRight);
void odometryHandler2(const nav_msgs::Odometry::ConstPtr& message);
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

//PUBLISH
  heartbeatPublisher = gNH.advertise<std_msgs::String>((publishedName + "/gridSwarm/heartbeat"), 1,true);
  publish_heartbeat_timer = gNH.createTimer(ros::Duration(heartbeat_publish_interval),publishHeartBeatTimerEventHandler);

  ros::Rate rate(30.0);
  sleep(30);
  ros::spinOnce();
  rate.sleep();


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
			odometrySubscriber = gNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
			sonarLeftSubscriber = gNH.subscribe((publishedName + "/sonarLeft"), 10, sonarHandlerLeft);
			sonarCenterSubscriber = gNH.subscribe((publishedName + "/sonarCenter"), 10, sonarHandlerCenter);
			sonarRightSubscriber = gNH.subscribe((publishedName + "/sonarRight"), 10, sonarHandlerRight);
		}else if(i == 1){
			odometrySubscriber1 = gNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler1);
			sonarLeftSubscriber1 = gNH.subscribe((publishedName + "/sonarLeft"), 10, sonarHandlerLeft1);
			sonarCenterSubscriber1 = gNH.subscribe((publishedName + "/sonarCenter"), 10, sonarHandlerCenter1);
			sonarRightSubscriber1 = gNH.subscribe((publishedName + "/sonarRight"), 10, sonarHandlerRight1);
		}else if(i == 2){
			odometrySubscriber2 = gNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler2);
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
	if (firstgo == true){
	//Populate the Entire Map intially to all FOG
		cout << "Creating the initial FOG" << endl;
		for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
			Position position;
			map.getPosition(*it, position);
			if (map.at("elevation", *it) == FOG || map.at("elevation", *it) == ROVER || firstgo == true){
				map.at("elevation", *it) = FOG;
			}	
		}//END OF ITERATOR
	//Center Mat being Discovered
		cout << "Creating the Center Mat" << endl;
		for (float length = -0.50; length <= 0.50;){
			for(float width = -0.50; width <= 0.50;){
				Vector2d mat(length,width);
				map.atPosition("elevation", mat) = DISCOVER;
				width += CELLDIVISION;
			}
			length += CELLDIVISION;
		}
	}
	for(int count = arrCount; count >= 0; count--){
		//ROVER
		float qx = xpos[count];
		float qy = ypos[count];
		Vector2d q(qx,qy);
		if (map.isInside(q)){
			map.atPosition("elevation", q) = ROVER;
		}
		//CAMERA 0.3m
		for (float length = CELLDIVISION; length <= 0.3;){
			for(float width = -0.15; width <= 0.15;){
				float Cax = (cos(orntn[count]) * length) + (xpos[count] + (sin(orntn[count]) * width));
				float Cay = (sin(orntn[count]) * length) + (ypos[count] + (cos(orntn[count]) * width));
				Vector2d cam(Cax,Cay);
				if (map.isInside(cam)){
					map.atPosition("elevation", cam) = DISCOVER;
				}
				width += CELLDIVISION;
			}
			length += CELLDIVISION;
		}
		//CENTER SONAR
		if (scenter[count] <= 2.8){
			bool overlap = false;
			float cx = (cos(orntn[count]) * scenter[count]) + xpos[count];
			float cy = (sin(orntn[count]) * scenter[count]) + ypos[count];
			Vector2d c(cx,cy);
			if (count == 0){
				for(int inner = 1; inner <= arrCount; inner++){
					float qx = xpos[inner];
					float qy = ypos[inner];
					if (qx <= (cx + ROVERHALF) && qx >= (cx - ROVERHALF) && qy <= (cy + ROVERHALF) && qy >= (cy - ROVERHALF)){
						overlap = true;
					}
				}
			}
			if (map.isInside(c) && overlap == false){
				map.atPosition("elevation", c) = WALL;
			}
		}
		//LEFT SONAR
		if (sleft[count] <= 2.8){
			bool overlap = false;
			float lx = (cos((pi/6)+orntn[count]) * sleft[count]) + xpos[count];
			float ly = (sin((pi/6)+orntn[count]) * sleft[count]) + ypos[count];
			Vector2d l(lx,ly);
			if (count == 0){
				for(int inner = 1; inner <= arrCount; inner++){
					float qx = xpos[inner];
					float qy = ypos[inner];
					if (qx <= (lx + ROVERHALF) && qx >= (lx - ROVERHALF) && qy <= (ly + ROVERHALF) && qy >= (ly - ROVERHALF)){
						overlap = true;
					}
				}
			}
			if (map.isInside(l) && overlap == false){
				map.atPosition("elevation", l) = WALL;
			}
		}
		//RIGHT SONAR
		if (sright[count] <= 2.8){
			bool overlap = false;
			float rx = (cos(-1*(pi/6)+orntn[count]) * sright[count]) + xpos[count];
			float ry = (sin(-1*(pi/6)+orntn[count]) * sright[count]) + ypos[count];
			Vector2d r(rx,ry);
			if (count == 0){
				for(int inner = 1; inner <= arrCount; inner++){
					float qx = xpos[inner];
					float qy = ypos[inner];
					if (qx <= (rx + ROVERHALF) && qx >= (rx - ROVERHALF) && qy <= (ry + ROVERHALF) && qy >= (ry - ROVERHALF)){
						overlap = true;
					}
				}
			}
			if (map.isInside(r) && overlap == false){
				map.atPosition("elevation", r) = WALL;
			}
		}
	}//END OF FOR LOOP
	firstgo = false;

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
		simoffsetLeft = ((sonarLeft->range)/cos(pi/6)) - (sonarLeft->range); 
	}
	sleft[0] = ((float(int(10 * sonarLeft->range)))/10) + simoffsetLeft;
}
void sonarHandlerLeft1(const sensor_msgs::Range::ConstPtr& sonarLeft) {
	float simoffsetLeft = 0;
	if(SIMMODE == true){
		simoffsetLeft = ((sonarLeft->range)/cos(pi/6)) - (sonarLeft->range); 
	}
	sleft[1] = ((float(int(10 * sonarLeft->range)))/10) + simoffsetLeft;
}
void sonarHandlerLeft2(const sensor_msgs::Range::ConstPtr& sonarLeft) {
	float simoffsetLeft = 0;
	if(SIMMODE == true){
		simoffsetLeft = ((sonarLeft->range)/cos(pi/6)) - (sonarLeft->range); 
	}
	sleft[2] = ((float(int(10 * sonarLeft->range)))/10) + simoffsetLeft;
}

void sonarHandlerCenter(const sensor_msgs::Range::ConstPtr& sonarCenter) {
	scenter[0]= ((float(int(10 * sonarCenter->range)))/10);
}
void sonarHandlerCenter1(const sensor_msgs::Range::ConstPtr& sonarCenter) {
	scenter[1]= ((float(int(10 * sonarCenter->range)))/10);
}
void sonarHandlerCenter2(const sensor_msgs::Range::ConstPtr& sonarCenter) {
	scenter[2]= ((float(int(10 * sonarCenter->range)))/10);
}

void sonarHandlerRight(const sensor_msgs::Range::ConstPtr& sonarRight) {
	float simoffsetRight = 0;
	if(SIMMODE == true){
		simoffsetRight = ((sonarRight->range)/cos(pi/6)) - (sonarRight->range); 
	}
	sright[0] = ((float(int(10 * sonarRight->range)))/10) + simoffsetRight;
}
void sonarHandlerRight1(const sensor_msgs::Range::ConstPtr& sonarRight) {
	float simoffsetRight = 0;
	if(SIMMODE == true){
		simoffsetRight = ((sonarRight->range)/cos(pi/6)) - (sonarRight->range); 
	}
	sright[1] = ((float(int(10 * sonarRight->range)))/10) + simoffsetRight;
}
void sonarHandlerRight2(const sensor_msgs::Range::ConstPtr& sonarRight) {
	float simoffsetRight = 0;
	if(SIMMODE == true){
		simoffsetRight = ((sonarRight->range)/cos(pi/6)) - (sonarRight->range); 
	}
	sright[2] = ((float(int(10 * sonarRight->range)))/10) + simoffsetRight;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	string rover = message->header.frame_id;
	rover = rover.substr(0,rover.find("/"));
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	orntn[0] = yaw;
	if (o0once == true){
		x0offset = (-1 - ROVERHALF) * cos(orntn[0]);
		y0offset = (-1 - ROVERHALF) * sin(orntn[0]);
		o0once = false;
	}
	xpos[0] = message->pose.pose.position.x + x0offset;
	ypos[0] = message->pose.pose.position.y + y0offset;
}

void odometryHandler1(const nav_msgs::Odometry::ConstPtr& message) {
	string rover = message->header.frame_id;
	rover = rover.substr(0,rover.find("/"));
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	orntn[1] = yaw;
	if (o1once == true){
		x1offset = (-1 - ROVERHALF) * cos(orntn[1]);
		y1offset = (-1 - ROVERHALF) * sin(orntn[1]);
		o1once = false;
	}
	xpos[1] = message->pose.pose.position.x + x1offset;
	ypos[1] = message->pose.pose.position.y + y1offset;
}
void odometryHandler2(const nav_msgs::Odometry::ConstPtr& message) {
	string rover = message->header.frame_id;
	rover = rover.substr(0,rover.find("/"));
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	orntn[2] = yaw;
	if (o2once == true){
		x2offset = (-1 - ROVERHALF) * cos(orntn[2]);
		y2offset = (-1 - ROVERHALF) * sin(orntn[2]);
		o2once = false;
	}
	xpos[2] = message->pose.pose.position.x + x2offset;
	ypos[2] = message->pose.pose.position.y + y2offset;
}

void roverNameHandler(const std_msgs::String& message){
	std::string list= message.data;
	for(int i=0;i<namesArrSize; i++){
		if(namesArr[i].compare("test") == 0){
			int index = list.find(",");
			namesArr[i] = list.substr(0,index);
			list.erase(0,index+1);
//			cout << "GRIDSWARM:namesArray "<<i<<": " << namesArr[i] << endl;
			if (list.empty()){
				i = namesArrSize;
			}
		}
	}
}
