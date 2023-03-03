#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <queue>
#include "Velocity_tracking.h"
#include "Track_cbf.h"
#define gravity 9.806

using namespace std;

void take_off(geometry_msgs::TwistStamped* desired_vel, geometry_msgs::PoseStamped self_pos, float height)
{
    if(abs(height - self_pos.pose.position.z > 0.01))
        desired_vel->twist.linear.z = height - self_pos.pose.position.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_cbf");
    ros::NodeHandle nh, private_nh("~");

    float hz = 120;
    //  publisher
    ros::Publisher track_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/velocity_cbf/track", 2);
    ros::Rate rate(hz);
    //  service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Object

    float target_Gamma, target_trackDistance, MAV_safeDistance;
    ros::param::get("target_gamma", target_Gamma);
    ros::param::get("track", target_trackDistance);
    ros::param::get("MAV_safe_D", MAV_safeDistance);

    Track_CBF cbf(nh, "/vrpn_client_node/MAV1/pose", "/vrpn_client_node/target/pose");
    Virtual_controller vir_acc(1/hz);
    cbf.setCBFparam(0.5, 0.3, 0.6); // track_distance, safe_distance, gamma
    vir_acc.setVirtualInputParam(1);

    geometry_msgs::TwistStamped desired_vel;
    geometry_msgs::TwistStamped desired_vel_raw;
    desired_vel.twist.linear.x = desired_vel.twist.linear.y = desired_vel.twist.linear.z = 0;
    desired_vel_raw.twist.linear.x = desired_vel_raw.twist.linear.y = desired_vel_raw.twist.linear.z = 0;

    while(ros::ok())
    {
        cbf.QPsolve_vel(desired_vel_raw , &desired_vel);
        cout << desired_vel << endl;

        track_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}