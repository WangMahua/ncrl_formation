#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CommandHome.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandTOL.h>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Int32.h>

#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>

geometry_msgs::PoseStamped gps_pose;
geometry_msgs::PoseStamped gps_pose_init;
geometry_msgs::PoseStamped initialized_pose;
using namespace std;

void gps_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	gps_pose = *msg;
}

void position_init()
{
	ROS_INFO("Wait for enemy GPS data ...");
	boost::shared_ptr<geometry_msgs::PoseStamped const> msg;
    msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("mavros/local_position/pose", ros::Duration(60));
	gps_pose_init = *msg;
	ROS_INFO("GPS data recieved");
}

void init_cb(const std_msgs::Int32::ConstPtr& msg)
{
	if(msg->data == 1)
		position_init();
	else
		ROS_WARN("Drone is not on land, can't initialize");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_init_backup");
    ros::NodeHandle nh;

    ////////////////////////////// Subscriber /////////////////////////////////

    ros::Subscriber gps_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, gps_pos_cb);
    ros::Subscriber init_sub = nh.subscribe<std_msgs::Int32>("/uav_init", 10, init_cb);

    ////////////////////////////// Publisher /////////////////////////////////

	ros::Publisher local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose_initialized", 10);
    ros::Rate rate(100);

    position_init();
    while (ros::ok())
    {
    	initialized_pose.pose.position.x = gps_pose.pose.position.x - gps_pose_init.pose.position.x;
    	initialized_pose.pose.position.y = gps_pose.pose.position.y - gps_pose_init.pose.position.y;
    	initialized_pose.pose.position.z = gps_pose.pose.position.z - gps_pose_init.pose.position.z;
    	initialized_pose.pose.orientation = gps_pose.pose.orientation;
    	local_pose_pub.publish(initialized_pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
