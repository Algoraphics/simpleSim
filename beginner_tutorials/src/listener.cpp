#include "ros/ros.h"
#include "beginner_tutorials/wrenchData.h"

void chatterCallback(const beginner_tutorials::wrenchData::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f %f %f %d]", msg->throttle, msg->brake, msg->steering, msg->gear);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("setWrench", 1000, chatterCallback);
  
  ROS_INFO("I started listening");

  ros::spin();

  return 0;
}
