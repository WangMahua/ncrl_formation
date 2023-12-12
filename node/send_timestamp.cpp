#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub_1 = n.advertise<std_msgs::Header>("/MAV1/transmit_delay", 10);
  ros::Publisher chatter_pub_2 = n.advertise<std_msgs::Header>("/MAV2/transmit_delay", 10);
  ros::Publisher chatter_pub_3 = n.advertise<std_msgs::Header>("/MAV3/transmit_delay", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    
    std_msgs::Header msg;
    msg.stamp.sec = ros::Time::now().toSec();
    chatter_pub_1.publish(msg);
    chatter_pub_2.publish(msg);
    chatter_pub_3.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}