#ifndef ORIGIN_PUBLISHER_H
#define ORIGIN_PUBLISHER_H

#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/ardupilotmega/mavlink.h>

class OriginPublisher {
public:
    OriginPublisher(ros::NodeHandle& nh, int sysid);
    void run();

private:
    ros::Publisher mavlink_pub;
    double lat;
    double lon;
    double alt;
    mavlink_status_t status;
    int SYSID;

    void send_message(const mavlink_message_t& mavlink_msg);
    void set_global_origin();
    void set_home_position();
};

#endif
