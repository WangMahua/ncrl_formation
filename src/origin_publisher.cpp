#include "origin_publisher.h"

OriginPublisher::OriginPublisher(ros::NodeHandle& nh, int sysid) : lat(0), lon(0), alt(0) {
    mavlink_pub = nh.advertise<mavros_msgs::Mavlink>("mavlink/to", 20);
    SYSID = sysid;
}

void OriginPublisher::send_message(const mavlink_message_t& mavlink_msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);

    mavros_msgs::Mavlink rosmsg;
    rosmsg.len = len;
    rosmsg.seq = 0;
    rosmsg.sysid = mavlink_msg.sysid;
    rosmsg.compid = mavlink_msg.compid;
    rosmsg.msgid = mavlink_msg.msgid;
    rosmsg.payload64.assign(buffer, buffer + len);

    mavlink_pub.publish(rosmsg);

    ROS_INFO("Sent message %d", mavlink_msg.msgid);
}

void OriginPublisher::set_global_origin() {
    mavlink_message_t mavlink_msg;
    mavlink_msg_set_gps_global_origin_pack(SYSID, 1, &mavlink_msg, SYSID, lat, lon, alt, 0);
    send_message(mavlink_msg);
}

void OriginPublisher::set_home_position() {
    mavlink_message_t mavlink_msg;
    float quat[] = {1, 0, 0, 0};
    mavlink_msg_set_home_position_pack(SYSID, 1, &mavlink_msg, SYSID, lat, lon, alt, 0, 0, 1, quat, 0, 0, 1, 0);
    send_message(mavlink_msg);
}

void OriginPublisher::run() {
    while (mavlink_pub.getNumSubscribers() <= 0) {
        ros::spinOnce();
    }

    for (int i = 0; i < 2; ++i) {
        ros::Duration(1.0).sleep();
        set_global_origin();
        set_home_position();
    }
}
