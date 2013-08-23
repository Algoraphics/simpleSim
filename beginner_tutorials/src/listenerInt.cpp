#include "ros/ros.h"
#include "std_msgs/Int32.h"

void chatterCallback(const beginner_tutorials::Int32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chat", 1000, chatterCallback);
  
  ROS_INFO("I started listening");

  ros::spin();

  return 0;
}