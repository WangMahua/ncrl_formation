#include "Velocity_tracking.h"

Virtual_controller::Virtual_controller(float dt)
{
    delta_t = dt;
}

void Virtual_controller::computeAcc()
{
    acc.x = kp_acc*(desired_vel.twist.linear.x - current_vel.twist.linear.x);
    acc.y = kp_acc*(desired_vel.twist.linear.y - current_vel.twist.linear.y);
    acc.z = kp_acc*(desired_vel.twist.linear.z - current_vel.twist.linear.z);
}
void Virtual_controller::computeVel()
{
    computeAcc();
    current_vel.twist.linear.x = current_vel.twist.linear.x + acc.x*delta_t;
    current_vel.twist.linear.y = current_vel.twist.linear.y + acc.y*delta_t;
    current_vel.twist.linear.z = current_vel.twist.linear.z + acc.z*delta_t;
}
void Virtual_controller::setVirtualInputParam(float kp){kp_acc = kp;}

void Virtual_controller::setVel(geometry_msgs::TwistStamped d_vel, geometry_msgs::TwistStamped c_vel)
{
    desired_vel = d_vel;
    current_vel = c_vel;
}

/*
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_speed_generation");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    Virtual_controller controller(nh);

    while(ros::ok())
    {
        car.computeAcc();
        car.computeVel();
        ros::spinOnce();

        rate.sleep();
    }
    
    return 0;
}
*/