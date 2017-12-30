#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <std_msgs/String.h>
#include <sstream>


float heartbeat_publish_interval = 2;

//Publisher
ros::Publisher gridswarmPublisher;
ros::Publisher test;
ros::Publisher heartbeatPublisher;
//Subscriber


//Timer
ros::Timer publish_heartbeat_timer;

//Global
sensor_msgs::Range sonarLeft;
sensor_msgs::Range sonarCenter;
sensor_msgs::Range sonarRight;
std::string publishedName;
char host[128];
using namespace std;
using namespace grid_map;

void publishHeartBeatTimerEventHandler(const ros::TimerEvent& event);

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

  test = gNH.advertise<std_msgs::String>(publishedName + "/gridtest", 10);
  heartbeatPublisher = gNH.advertise<std_msgs::String>((publishedName + "/gridSwarm/heartbeat"), 1,true);
  publish_heartbeat_timer = gNH.createTimer(ros::Duration(heartbeat_publish_interval),publishHeartBeatTimerEventHandler);
//  gridswarmPublisher = gNH.advertise<grid_map_msgs::GridMap>("grid_map", 1);



// Create grid map.
//GridMap map({"elevation"});
//map.setFrameId("map");
//map.setGeometry(Length(3.0, 3.0), 0.05);
//ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
//  map.getLength().x(), map.getLength().y(),
//  map.getSize()(0), map.getSize()(1));  
   
    
  ros::Rate loop_rate(30.0);
  int count = 0;
  while (ros::ok()) {
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world" << count;
	msg.data = ss.str();
	
	ROS_INFO("%s", msg.data.c_str());
	test.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
	++count;
//  Add data to grid map.
//  ros::Time time = ros::Time::now();
//  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
//    Position position;
//    map.getPosition(*it, position);
//    map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
//  }
//
//  // Publish grid map.
//  map.setTimestamp(time.toNSec());
//  grid_map_msgs::GridMap message;
//  GridMapRosConverter::toMessage(map, message);
//  gridswarmPublisher.publish(message);
//  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
//  rate.sleep();
  }

return 0;
}

void publishHeartBeatTimerEventHandler(const ros::TimerEvent&){
	std_msgs::String msg;
	msg.data = "";
	heartbeatPublisher.publish(msg);
}
