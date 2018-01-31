#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <sstream>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <unistd.h>
>>>>>>> ed9fd1d9915ab54f22bd62057cc0bca37bd9d551

using namespace std;

float heartbeat_publish_interval = 2;
const float CELLDIVISION = 0.05;
const float WALL = 1;
const float FOG = -0.5;
const float ROVER = 0.5;
const float DISCOVER = 0.0;

/*----------------MAKE SURE TO TURN FALSE WHEN YOU ARE NOT RUNNING THE SIMULATION----------------*/
/*->->->->->->->->->*/	bool SIMMODE = true;	/*<-<-<-<-<-<-<-<-<-<-<-<-<-<-*/
/*----------------MAKE SURE TO TURN FALSE WHEN YOU ARE NOT RUNNING THE SIMULATION----------------*/

//Publisher
ros::Publisher gridswarmPublisher;
//ros::Publisher mainGridPublisher;
ros::Publisher test;
ros::Publisher heartbeatPublisher;
//Subscriber
ros::Subscriber sonarLeftSubscriber;
ros::Subscriber sonarCenterSubscriber;
ros::Subscriber sonarRightSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber roverNameSubscriber;

//Timer
ros::Timer publish_heartbeat_timer;

std::string publishedName;
//Global
  const double pi = std::acos(-1);
  const int namesArrSize=1;
  string namesArr[namesArrSize] = {"test"};//,"test","test","test","test","test"};
  int arrCount = 0;
  float sleft[namesArrSize];
  float scenter[namesArrSize];
  float sright[namesArrSize];
  float orntn[namesArrSize];
  float xpos[namesArrSize];
  float ypos[namesArrSize];
>>>>>>> ed9fd1d9915ab54f22bd62057cc0bca37bd9d551
  char host[128];
  bool firstgo = true;
using namespace std;
using namespace grid_map;
using namespace Eigen;
//using std::string;

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void sonarHandlerLeft(const sensor_msgs::Range::ConstPtr& sonarLeft);
void sonarHandlerCenter(const sensor_msgs::Range::ConstPtr& sonarCenter);
void sonarHandlerRight(const sensor_msgs::Range::ConstPtr& sonarRight);
void roverNameHandler(const std_msgs::String& message);


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
  ros::init(argc, argv, (hostname + "_GRIDSWARM"), ros::init_options::NoSigintHandler);
  ros::NodeHandle gNH;
//PUBLISH
  heartbeatPublisher = gNH.advertise<std_msgs::String>((publishedName + "/gridSwarm/heartbeat"), 1,true);
  publish_heartbeat_timer = gNH.createTimer(ros::Duration(heartbeat_publish_interval),publishHeartBeatTimerEventHandler);
  gridswarmPublisher = gNH.advertise<grid_map_msgs::GridMap>(publishedName + "/grid_map", 1);
//  mainGridPublisher = gNH.advertise<grid_map_msgs::GridMap>("/MAIN_GRID", 1);

//SUBSCRIBER
  roverNameSubscriber = gNH.subscribe(("/roverNames"), 1, roverNameHandler);
  sleep(10);

  for(int i = 0; i < namesArrSize; i++){
	cout << "namesArr[" << i << "] =" << namesArr[i] <<":Start Loop";
	if (namesArr[i] != "test"){
		arrCount = i;
		publishedName = namesArr[i];
		cout << " + Entered Subscriber loop"<< endl;
		odometrySubscriber = gNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
		sonarLeftSubscriber = gNH.subscribe((publishedName + "/sonarLeft"), 10, sonarHandlerLeft);
		sonarCenterSubscriber = gNH.subscribe((publishedName + "/sonarCenter"), 10, sonarHandlerCenter);
		sonarRightSubscriber = gNH.subscribe((publishedName + "/sonarRight"), 10, sonarHandlerRight);
	//	message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(gNH, (publishedName + "/sonarLeft"), 10);
	//	message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(gNH, (publishedName + "/sonarCenter"), 10);
	//	message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(gNH, (publishedName + "/sonarRight"), 10);
	//	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;
	//	message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
	//	sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));
	}//END OF IF STATEMENT
  }//END OF FOR LOOP
  cout << " + Exit Subscriber loop"<< endl;



  ros::Rate rate(30.0);
  int count = 0;
  // Create grid Rover Specific Map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(15.5, 15.5), CELLDIVISION);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));  

  while (ros::ok()) {
	cout << "Enter ROS OK"<<endl;
	// Add data to Rover Specific Grid Map.
	ros::Time time = ros::Time::now();
	for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
		Position position;
		map.getPosition(*it, position);
		if (map.at("elevation", *it) == FOG || map.at("elevation", *it) == ROVER || firstgo == true){
			map.at("elevation", *it) = FOG;
		}
		//ORIGIN
//	Vector2d o(0,0);
//	map.atPosition("elevation", o) = 0;
//		cout << "Entering Grid-Map Data";
		for(count = arrCount; count >= 0; count--){
//			cout << "!!Entered Grid-Map Data: "<< count<< endl;
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
			//CENTER
			//THEORY: MAKE SURE PING HITS MULTIPLE TIMES IN A SECOND BEFORE SAVING.
			if (scenter[count] <= 2.8){
				float cx = (cos(orntn[count]) * scenter[count]) + xpos[count];
				float cy = (sin(orntn[count]) * scenter[count]) + ypos[count];
				Vector2d c(cx,cy);
				if (map.isInside(c)){
					map.atPosition("elevation", c) = WALL;
				}
			}
			//LEFT
			if (sleft[count] <= 2.8){
				float lx = (cos((pi/6)+orntn[count]) * sleft[count]) + xpos[count];
				float ly = (sin((pi/6)+orntn[count]) * sleft[count]) + ypos[count];
				Vector2d l(lx,ly);
				if (map.isInside(l)){
					map.atPosition("elevation", l) = WALL;
				}
			}
			//RIGHT
			if (sright[count] <= 2.8){
				float rx = (cos(-1*(pi/6)+orntn[count]) * sright[count]) + xpos[count];
				float ry = (sin(-1*(pi/6)+orntn[count]) * sright[count]) + ypos[count];
				Vector2d r(rx,ry);
				if (map.isInside(r)){
					map.atPosition("elevation", r) = WALL;
				}
			}
		}
	}//END OF ITERATOR
	firstgo = false;

	// Publish grid map.
	map.setTimestamp(time.toNSec());
	grid_map_msgs::GridMap message;
	//PUBLISH Rover Specific Grid Map
	GridMapRosConverter::toMessage(map, message);
	gridswarmPublisher.publish(message);
	ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
	
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
	cout << "Enter Sonar Subscriver Left" << endl;
	float simoffsetLeft = 0;
	if(SIMMODE == true){
		simoffsetLeft = ((sonarLeft->range)/cos(pi/6)) - (sonarLeft->range); 
	}
	sleft[arrCount]  = ((float(int(10 * sonarLeft->range)))/10) + simoffsetLeft;
}

void sonarHandlerCenter(const sensor_msgs::Range::ConstPtr& sonarCenter) {
	cout << "Enter Sonar Subscriver Center" << endl;
	scenter[arrCount]= ((float(int(10 * sonarCenter->range)))/10);
}

void sonarHandlerRight(const sensor_msgs::Range::ConstPtr& sonarRight) {
	cout << "Enter Sonar Subscriver Right" << endl;
	float simoffsetRight = 0;
	if(SIMMODE == true){
		simoffsetRight = ((sonarRight->range)/cos(pi/6)) - (sonarRight->range); 
	}
	sright[arrCount] = ((float(int(10 * sonarRight->range)))/10) + simoffsetRight;
}
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	cout << "Enter odom Subscriver" << endl;
	tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	orntn[arrCount] = yaw;
	xpos[arrCount] = message->pose.pose.position.x;
	ypos[arrCount] = message->pose.pose.position.y;
}

void roverNameHandler(const std_msgs::String& message){
	for(int i=0;i<namesArrSize; i++){
		if(namesArr[i].compare("test") == 0){
			namesArr[i] = message.data;
			i=7;
		}
		else if(namesArr[i].compare(message.data)==0){
			i=7;
		}
	}
	for(int i=0; i<namesArrSize; i++){ 
		cout << "namesArr[" << i << "] =" << namesArr[i]<< endl;
		// allRoversPublisher.publish(message);
	}
>>>>>>> ed9fd1d9915ab54f22bd62057cc0bca37bd9d551
}
