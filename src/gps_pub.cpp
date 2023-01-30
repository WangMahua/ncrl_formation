#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Altitude.h>
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
float amsl_alt = 0;
geometry_msgs::Pose gps_pose;
geometry_msgs::Pose gps_pose_init;
float gps_pose_sum_x = 0;
float gps_pose_sum_y = 0;
float gps_pose_sum_z = 0;
int mean_n = 0;

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void gps_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	gps_pose.position = msg->pose.position;
	gps_pose.orientation = msg->pose.orientation;
}

void position_init()
{
	ROS_INFO("Wait for leader GPS data ...");
	boost::shared_ptr<geometry_msgs::PoseStamped const> msg;
    msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/MAV1/mavros/local_position/pose", ros::Duration(30));
	gps_pose_init.position = msg->pose.position;
	ROS_INFO("Home position is set to leader's position");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_pub");
    ros::NodeHandle nh;

    string pub_pose_topic, gps_global_topic;
    ros::param::get("pub_pose_topic", pub_pose_topic);
    ros::param::get("gps_global_topic", gps_global_topic);
//Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PoseStamped>(gps_global_topic, 10, gps_pos_cb);	//gps position
    //ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PoseStamped>("/MAV1/mavros/local_position/pose", 10, gps_pos_cb);	//gps position
//Publisher
	ros::Publisher ENU_pub = nh.advertise<geometry_msgs::PoseStamped>(pub_pose_topic, 10);
	//ros::Publisher ENU_pub = nh.advertise<geometry_msgs::PoseStamped>("/MAV1/mavros/global_position/ENU/pose", 10);

    ros::Rate rate(120);
    
    geometry_msgs::PoseStamped pose;
    double now_pos[3] = {0};
    position_init();
    while (ros::ok()) {

		now_pos[0] = gps_pose.position.x - gps_pose_init.position.x;
		now_pos[1] = gps_pose.position.y - gps_pose_init.position.y;
		now_pos[2] = gps_pose.position.z - gps_pose_init.position.z;

		pose.pose.position.x = now_pos[0];
		pose.pose.position.y = now_pos[1];
		pose.pose.position.z = now_pos[2];
		pose.pose.orientation = gps_pose.orientation;

		ENU_pub.publish(pose);
		std::cout << pose << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

