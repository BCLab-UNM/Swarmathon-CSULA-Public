#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <std_msgs/String.h>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

float heartbeat_publish_interval = 2;

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

//Timer
ros::Timer publish_heartbeat_timer;

std::string publishedName;
//Global
const double pi = std::acos(-1);
float sleft = 0;
float scenter = 0;
float sright = 0;
char host[128];
float orntn = 0;
bool firstgo = true;
//bool largeMapMade = false;
using namespace std;
using namespace grid_map;
using namespace Eigen;

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);
void odometryHandler(const nav_msgs::Odometry::ConstPtr& message);
void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight);

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
//  test = gNH.advertise<std_msgs::String>(publishedName + "/gridtest", 10);
  heartbeatPublisher = gNH.advertise<std_msgs::String>((publishedName + "/gridSwarm/heartbeat"), 1,true);
  publish_heartbeat_timer = gNH.createTimer(ros::Duration(heartbeat_publish_interval),publishHeartBeatTimerEventHandler);
  gridswarmPublisher = gNH.advertise<grid_map_msgs::GridMap>(publishedName + "/grid_map", 1);
//  mainGridPublisher = gNH.advertise<grid_map_msgs::GridMap>("/MAIN_GRID", 1);
//SUBSCRIBER
  odometrySubscriber = gNH.subscribe((publishedName + "/odom/filtered"), 10, odometryHandler);
  message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(gNH, (publishedName + "/sonarLeft"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(gNH, (publishedName + "/sonarCenter"), 10);
  message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(gNH, (publishedName + "/sonarRight"), 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;
  
  message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
  sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

  ros::Rate rate(30.0);
  int count = 0;
//  while (ros::ok()) {
//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world" << count;
//        msg.data = ss.str();
//        
//        ROS_INFO("%s", msg.data.c_str());
//        test.publish(msg);
//        ros::spinOnce();
//        rate.sleep();
//        ++count;
//  }
  // Create grid Rover Specific Map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(6.15, 6.15), 0.05);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));  

  // Create the larger overall map
//  if(largeMapMade == false){
//        GridMap largeMap({"large"});
//        largeMap.setFrameId("largeMap");
//        largeMap.setGeometry(Length(6.15, 6.15), 0.05);
//        ROS_INFO("Created Large Map with size %f x %f m (%i x %i cells).",
//          largeMap.getLength().x(), largeMap.getLength().y(),
//          largeMap.getSize()(0), largeMap.getSize()(1));  
//  }    

  while (ros::ok()) {
	// Add data to Rover Specific Grid Map.
	ros::Time time = ros::Time::now();
	for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
		Position position;
		map.getPosition(*it, position);
		if (map.at("elevation", *it) == -0.5 || firstgo == true){
			map.at("elevation", *it) = -0.5;
		}
		//CENTER
		if (scenter <= 2.7){
			float cy = sin(orntn) * scenter;
			float cx = cos(orntn) * scenter;
			Vector2d c(cx,cy);
			map.atPosition("elevation", c) = 0.5;
		}
		//LEFT
		if (sleft <= 2.7){
			float ly = sin((pi/6)+orntn) * sleft;
			float lx = cos((pi/6)+orntn) * sleft;
			Vector2d l(lx,ly);
			map.atPosition("elevation", l) = 0.5;
		}
		//RIGHT
		if (sright <= 2.7){
			float ry = sin(-1*(pi/6)+orntn) * sright;
			float rx = cos(-1*(pi/6)+orntn) * sright;
			Vector2d r(rx,ry);
			map.atPosition("elevation", r) = 0.5;
		}

	}
	firstgo = false;
//	if(largeMapMade == false){
//		// Add data to Larger Area Grid Map.
//		for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
//			Position position;
//			largeMap.getPosition(*it, position);
//			largeMap.at("large", *it) = -0.5;
//		}
//	}

	// Publish grid map.
	map.setTimestamp(time.toNSec());
	grid_map_msgs::GridMap message;
	//PUBLISH Rover Specific Grid Map
	GridMapRosConverter::toMessage(map, message);
	gridswarmPublisher.publish(message);
//	if(largeMapMade == false)}
//		GridMapRosConverter::toMessage(largemap, message);
//		mainGridPublisher.publish(message);
//	}
	ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
	
	// Wait for next cycle.
	ros::spinOnce();
	rate.sleep();
  }

return 0;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&){
	std_msgs::String msg;
	msg.data = "";
	heartbeatPublisher.publish(msg);
}

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {
  
	sleft  = ((float(int(10 * sonarLeft->range)))/10);
	scenter= ((float(int(10 * sonarCenter->range)))/10);
	sright = ((float(int(10 * sonarRight->range)))/10);
  
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
	orntn = message->pose.pose.orientation.z;
	orntn = pi * orntn;
}
