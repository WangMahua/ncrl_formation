#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Header>("transmit_delay", 10);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    
    std_msgs::Header msg;
    msg.stamp.sec = ros::Time::now().toSec();
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}