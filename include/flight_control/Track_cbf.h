#pragma once

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
#define gravity 9.806

using namespace std;

class Track_CBF
{
private:
    geometry_msgs::PoseStamped target_pos;
    geometry_msgs::PoseStamped self_pos;
    double roll;
    double pitch;
    double yaw;
    ros::Subscriber target_pos_sub;
    ros::Subscriber self_pos_sub;
    float distance_track;
    float distance_safe;
    float gamma;

public:
    Track_CBF();
    Track_CBF(ros::NodeHandle nh, string target_pos_topic, string self_pos_topic);
    void target_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void self_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setCBFparam(float dis_t, float dis_s, float gamma);
    float getTrackDistance();
    float getSafeDistance();
    float getGamma();
    geometry_msgs::PoseStamped getTargetPose();
    geometry_msgs::PoseStamped getSelfPose();
    double getSelfYaw();

    int QPsolve_vel_track(geometry_msgs::TwistStamped desired_vel_raw, geometry_msgs::TwistStamped* desired_vel);
    int QPsolve_vel_avoid(geometry_msgs::TwistStamped desired_vel_raw, geometry_msgs::TwistStamped* desired_vel);


    bool selfPose_init;
};

