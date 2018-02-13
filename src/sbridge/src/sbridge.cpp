#include "sbridge.h"

sbridge::sbridge(std::string publishedName) {

    ros::NodeHandle sNH;


    driveControlSubscriber = sNH.subscribe((publishedName + "/driveControl"), 10, &sbridge::cmdHandler, this);
//    driveControlSubscriber = sNH.subscribe("/roverName", 10, &sbridge::roverNameCheck, this);

    heartbeatPublisher = sNH.advertise<std_msgs::String>((publishedName + "/sbridge/heartbeat"), 1, false);
    skidsteerPublish = sNH.advertise<geometry_msgs::Twist>((publishedName + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);
    roverNamePublisher = sNH.advertise<std_msgs::String>("/roverNames", 1, true);

//    publishNamePublisher = sNH.advertise<std_msgs::String>("/roverName",1);

    float heartbeat_publish_interval = 2;
    publish_heartbeat_timer = sNH.createTimer(ros::Duration(heartbeat_publish_interval), &sbridge::publishHeartBeatTimerEventHandler, this);


    std_msgs::String nameMsg;
    nameMsg.data=publishedName;
    roverNamePublisher.publish(nameMsg);
    ROS_INFO("constructor");

}
/*
void sbridge::roverNameCheck(const std_msgs::String message) {
	if (message == NULL || message.length() == 0){
		//First Pass Publisher
		//publish first hostname
	}
	else{
		//Publish(message + "," + publishedName);
	}
	// Make sure this method only occurs once per rover
	// By cancel subscriber or boolean skip.	
	
	
}
*/
void sbridge::cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    double left = (message->linear.x);
    double right = (message->angular.z);
    
    float max_turn_rate = 4.5; //radians per second
    float max_linear_velocity = 0.65; // meters per second

    float turn = 0;
    float forward = 0;

    float linearVel = (left + right)/2;
    float angularVel = (right-left)/2;

    turn = angularVel/55;
    forward = linearVel/390;
    if (forward >= 150){
      forward -= (abs(turn)/5);
    }

    if (linearVel >= 0 && forward <= 0)
    {
      forward = 0;
    }
    if (linearVel <= 0 && forward >= 0)
    {
      forward = 0;
    }

    if (fabs(forward) >= max_linear_velocity) {
        forward = forward/fabs(forward) * max_linear_velocity;
    }

    if (fabs(turn) >= max_turn_rate) { //max value needs tuning
        turn = turn/fabs(turn) * max_turn_rate;
    }

    velocity.linear.x = forward,
            velocity.angular.z = turn;
    skidsteerPublish.publish(velocity);
}

void sbridge::publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);

    ROS_INFO("%ds, %dnsec", event.last_real.sec, event.last_real.nsec);
}

sbridge::~sbridge() {
}
