#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void tdCallback(const std_msgs::Header::ConstPtr& msg)
{
    double now_time = ros::Time::now().toSec();
    std::cout << now_time - msg->stamp.sec << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("transmit_delay", 1000, tdCallback);
  ros::spin();
  return 0;
}