#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

class Virtual_controller
{
private:
    float kp_acc;
    float delta_t;
    struct ACC
    {
        float x;
        float y;
        float z;
    }acc;
    geometry_msgs::TwistStamped desired_vel;
    geometry_msgs::TwistStamped current_vel;
public:
    Virtual_controller();
    Virtual_controller(float dt);
    void setVirtualInputParam(float kp);
    void computeAcc();
    void computeVel();
    void setVel(geometry_msgs::TwistStamped d_vel, geometry_msgs::TwistStamped c_vel);
};