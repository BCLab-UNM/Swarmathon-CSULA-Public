#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <cmath> 
#include <std_msgs/String.h>
#include <sstream>


using namespace std;


class gridmap {
public:
    
    gridmap();
    virtual ~gridmap();
  
    void makemap(string hostname);
private:


};

