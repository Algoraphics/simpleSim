#include "ros/ros.h"
#include "beginner_tutorials/wrenchData.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wrenchTalker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<beginner_tutorials::wrenchData>("reportWrench", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    beginner_tutorials::wrenchData msg;

    msg.brake = 4.2353;
    msg.throttle = 7.2309;
    msg.steering = 22.8359;

    ROS_INFO("%d %d %d", msg.brake, msg.throttle, msg.steering);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}