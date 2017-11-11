#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <std_msgs/String.h>

//Publisher
ros::Publisher gridswarmPublisher;

//Subscriber


//Global
sensor_msgs::Range sonarLeft;
sensor_msgs::Range sonarCenter;
sensor_msgs::Range sonarRight;

char host[128];
using namespace std;
using namespace grid_map;

std::string publishedName;

int main(int argc, char **argv){

  ros::init(argc, argv, ("gridmapswarm"));
  ros::NodeHandle mNH("~");
  gridswarmPublisher = mNH.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

// Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(3.0, 3.0), 0.05);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));  
   
    
  ros::Rate rate(30.0);
  while (mNH.ok()) {

    // Add data to grid map.
    ros::Time time = ros::Time::now();
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    gridswarmPublisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }

return 0;
}
