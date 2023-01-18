#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandTOL.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>

#include "flight_control/gps_transform.h"


double q_x = 0;
double q_y = 0;
double q_z = 0;
double q_w = 0;
// double odometry_x = 0;
// double odometry_y = 0;
// double odometry_z = 0;
//
gps_transform gps;

//autopilot ap(gps);


using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void gps_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	double latitude = msg->latitude;
	double longitude = msg->longitude;
	double altitude = msg->altitude;
	//set home point

	ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
	gps.update(latitude,longitude,altitude);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
	q_x = msg->orientation.x;
	q_y = msg->orientation.y;
	q_z = msg->orientation.z;
	q_w = msg->orientation.w;
}

void position_imu_init(Eigen::Quaterniond *q_init, string imu_topic)
{
	ROS_INFO("Wait for leader GPS data ...");
	boost::shared_ptr<sensor_msgs::NavSatFix const> leader_gps_msg;
    leader_gps_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/MAV1/mavros/global_position/global", ros::Duration(30));

    double latitude = leader_gps_msg->latitude;
	double longitude = leader_gps_msg->longitude;
	double altitude = leader_gps_msg->altitude;

    if(gps.is_init() == false)
		gps.set_home_longitude_latitude(latitude,longitude,altitude);

	ROS_INFO("Home position is set to leader position");

	ROS_INFO("Wait for self IMU data ...");
	boost::shared_ptr<sensor_msgs::Imu const> imu_msg;
    imu_msg = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic, ros::Duration(30));
    Eigen::Quaterniond q(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    *q_init = q;
    ROS_INFO("Reseted to current orientation");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_pub");
    ros::NodeHandle nh;

    string pub_pose_topic;
    string gps_global_topic;
    string imu_topic;
    ros::param::get("pub_pose_topic", pub_pose_topic);
    ros::param::get("gps_global_topic", gps_global_topic);
    ros::param::get("imu_topic", imu_topic);
//Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(pub_pose_topic, 10, gps_pos_cb);	//gps position
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, imu_cb );	//odometry orientation

//Publisher
	ros::Publisher ENU_pub = nh.advertise<geometry_msgs::PoseStamped>(pub_pose_topic, 10);

    ros::Rate rate(120);
    
    geometry_msgs::PoseStamped pose;
    Eigen::Quaterniond q_init;
    Eigen::Quaterniond q_new;
    Eigen::Quaterniond p_new;
    position_imu_init(&q_init, imu_topic);
    Eigen::Quaterniond q_imu;

    while(gps.is_init() == false){
		ros::spinOnce();
        rate.sleep();
        ROS_INFO("Self GPS is not ready");
    }

    while (ros::ok()) {

		double now_pos[3];
		gps.get_ENU(now_pos);
		Eigen::Quaterniond q_imu(q_w, q_x, q_y, q_z);
		Eigen::Quaterniond p(0, now_pos[0], now_pos[1], now_pos[2]);
		p_new = q_init.inverse()*p*q_init;

		pose.pose.position.x = p_new.x();
		pose.pose.position.y = p_new.y();
		pose.pose.position.z = p_new.z();
		
		q_new = q_init.inverse()*q_imu;

		pose.pose.orientation.x = q_new.x();
		pose.pose.orientation.y = q_new.y();
		pose.pose.orientation.z = q_new.z();
		pose.pose.orientation.w = q_new.w();

		ENU_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

