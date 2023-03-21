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
sensor_msgs::NavSatFix origin_lla;


using namespace std;

/*
void gps_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	gps_pose = *msg;
}
*/

void position_init()
{
	ROS_INFO("Wait for enemy GPS data ...");
	bool service_ready = ros::service::waitForService("mavros/cmd/set_home", 1000);
    if (!service_ready)
    {
	    ROS_ERROR("Failed to find set_home service");
	    return;
    }
	boost::shared_ptr<sensor_msgs::NavSatFix const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/Target/mavros/global_position/global", ros::Duration(60));
	origin_lla = *msg;

	mavros_msgs::CommandHome set_home;
	set_home.request.latitude = origin_lla.latitude;
	set_home.request.longitude = origin_lla.longitude;
	set_home.request.altitude = origin_lla.altitude;
	bool success = ros::service::call("mavros/cmd/set_home", set_home);
	if (success)
	{
	    ROS_INFO("Home position set to latitude: %f, longitude: %f, altitude: %f",
	             set_home.request.latitude, set_home.request.longitude, set_home.request.altitude);
	}
	else 
	{
		ROS_ERROR("Failed to set home position");
		return ;
	}
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
    ros::init(argc, argv, "gps_init");
    ros::NodeHandle nh;

    ////////////////////////////// Subscriber /////////////////////////////////

    //ros::Subscriber gps_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, gps_pos_cb);
    ros::Subscriber init_sub = nh.subscribe<std_msgs::Int32>("/uav_init", 10, init_cb);

    ////////////////////////////// Publisher /////////////////////////////////

	//ros::Publisher local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/initialized_position/pose", 10);
    //ros::Rate rate(100);

    position_init();
    ros::spin();
    /*
    while (ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }
    */
    return 0;
}
