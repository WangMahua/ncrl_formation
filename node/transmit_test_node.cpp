#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int count = 0;
double sum = 0.0;

void tdCallback(const std_msgs::Header::ConstPtr& msg)
{
    double now_time = ros::Time::now().toSec();
    count++;
    sum += (now_time - msg->stamp.sec);
    std::cout << now_time - msg->stamp.sec <<",\taverage time:" << sum/count<< std::endl;

    if(count==100){
        count = 0;
        sum = 0.0;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("transmit_delay", 1000, tdCallback);
  ros::spin();
  return 0;
}