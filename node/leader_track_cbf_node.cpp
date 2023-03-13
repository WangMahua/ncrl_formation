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

int mode = 0;
int kill_all_drone = 0;
int takeOff_height = 1;
mavros_msgs::State current_state;

void takeOff(geometry_msgs::TwistStamped* desired_vel, geometry_msgs::PoseStamped self_pos, float height)
{
    if(abs(height - self_pos.pose.position.z > 0.01))
    {
        desired_vel->twist.linear.x = -self_pos.pose.position.x;
        desired_vel->twist.linear.y = 1 - self_pos.pose.position.y;
        desired_vel->twist.linear.z = height - self_pos.pose.position.z;
    }
}

void land(geometry_msgs::TwistStamped* desired_vel, geometry_msgs::PoseStamped self_pos)
{
    if(abs(self_pos.pose.position.z > 0.01))
    {
        desired_vel->twist.linear.x = 0;
        desired_vel->twist.linear.y = 0;
        desired_vel->twist.linear.z = -0.15;
    }
    else
        desired_vel->twist.linear.z = 0;
}

void stop(geometry_msgs::TwistStamped* desired_vel)
{
    desired_vel->twist.linear.x = 0;
    desired_vel->twist.linear.y = 0;
    desired_vel->twist.linear.z = 0;
}

void kill_cb(const std_msgs::Int32 msg)
{
    kill_all_drone = msg.data;
}

void mode_cb(const std_msgs::Int32 msg)
{
    mode = msg.data;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
    current_state = *msg;
}

void bound_yaw(double* yaw)
{
    if(*yaw>M_PI)
        *yaw = *yaw - 2*M_PI;
    else if(*yaw<-M_PI)
        *yaw = *yaw + 2*M_PI;
}

void follow_yaw(geometry_msgs::TwistStamped& desired_vel, double desired_yaw, double yaw)
{
    double err_yaw, u_yaw;
    float KPyaw = 1;
    err_yaw = desired_yaw - yaw;
    bound_yaw(&err_yaw);
    u_yaw = KPyaw*err_yaw;
    desired_vel.twist.angular.z = u_yaw;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_cbf");
    ros::NodeHandle nh;

    float hz = 120;

    //  subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
    ros::Subscriber uav_killer_sub = nh.subscribe<std_msgs::Int32>("/uav_kill", 10, kill_cb);
    ros::Subscriber uav_mode = nh.subscribe<std_msgs::Int32>("/uav_mode", 10, mode_cb);

    //  publisher
    ros::Publisher track_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/velocity_cbf/track", 2);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 2);
    ros::Rate rate(hz);
    //  service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Object

    float cbf_Gamma, trackDistance, safeDistance;
    string uav_pose_topic;
    ros::param::get("cbf_gamma", cbf_Gamma);
    ros::param::get("track_D", trackDistance);
    ros::param::get("safe_D", safeDistance);
    ros::param::get("sub_topic", uav_pose_topic);

    Track_CBF cbf(nh, uav_pose_topic, "/vrpn_client_node/MAV1/pose");
    //Virtual_controller vir_acc(1/hz);

    cbf.setCBFparam(trackDistance, safeDistance, cbf_Gamma); // track_distance, safe_distance, gamma
    //vir_acc.setVirtualInputParam(1);

    geometry_msgs::TwistStamped desired_vel;
    geometry_msgs::TwistStamped desired_vel_raw;
    desired_vel.twist.linear.x = desired_vel.twist.linear.y = desired_vel.twist.linear.z = 0;
    desired_vel_raw.twist.linear.x = desired_vel_raw.twist.linear.y = desired_vel_raw.twist.linear.z = 0;

    /////////////////////////////////////// start progress /////////////////////////////////////////

    ROS_INFO("Wait for pose and desired input init");
    while (ros::ok() && (!cbf.selfPose_init)) 
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose init %d",cbf.selfPose_init);
    }
    ROS_INFO("pose initialized");
    
    ROS_INFO("Wait for FCU connection");
    while (ros::ok() && !current_state.connected) 
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for FCU");
    }
    ROS_INFO("FCU connected");

    ROS_INFO("Wait for UAV all start signal");
    while (ros::ok())
    {
        if(mode != 0)
            break;
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for UAV all start signal");
    }
    ROS_INFO("get UAV all start signal");

    //send a few velocity setpoints before starting
    for(int i = 0; ros::ok() && i < 20; i++)
    {
        local_vel_pub.publish(desired_vel);
        rate.sleep();
        
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        ROS_INFO("Offboard enabled");

    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        ROS_INFO("Vehicle armed");

    while (ros::ok()) 
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))) 
        {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else 
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))) 
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success) 
                    ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            }
        }
    
        //keyboard control
        if(kill_all_drone == 1)
        {
            ROS_WARN("velocity_cbf_kill!");
            offb_set_mode.request.custom_mode = "STABILIZED";
            set_mode_client.call(offb_set_mode);
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
        }

        if(( ros::Time::now() - cbf.getSelfPose().header.stamp)<ros::Duration(0.5))
        {
            switch(mode)
            {
                case 1:
                    takeOff(&desired_vel, cbf.getSelfPose(), takeOff_height);
                    break;
                case 2:
                    land(&desired_vel, cbf.getSelfPose());
                    break;
                case 3:
                    if(cbf.QPsolve_vel(desired_vel_raw , &desired_vel, 1) != 0)
                        desired_vel = desired_vel_raw;
                    break;
                case 4:
                    stop(&desired_vel);
                    break;
            }
            if(cbf.QPsolve_vel(desired_vel, &desired_vel, 0) != 0)
                desired_vel = desired_vel_raw;
            //  ROS_INFO("cbf input:vx: %f vy: %f \n",desired_vel.twist.linear.x,desired_vel.twist.linear.y); 
        }
        else
            desired_vel = desired_vel_raw;
        
        follow_yaw(desired_vel, M_PI/2, cbf.getSelfYaw());
        local_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}