#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <std_msgs/Int32.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <queue>

#define gravity 9.806

using namespace std;

bool desired_input_init = false;
bool pose_init = false;

//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
double KPyaw = 1;

double roll = 0, pitch = 0, yaw = 0;

// var for desired_pose
geometry_msgs::PoseStamped desired_pose;
double desired_yaw = 0.0;
int kill_all_drone = 0;
int start_all_drone = 0;
int takeoff_all_drone = 0;
int gs_state = 0;
// var for desired_velocity
geometry_msgs::TwistStamped desired_vel_raw;
geometry_msgs::TwistStamped desired_vel_vor;
geometry_msgs::TwistStamped desired_vel;        //output
geometry_msgs::TwistStamped desired_vel_init;        //output

// var for obstacle
//sgeometry_msgs::PoseStamped obstacle_pose;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;

void bound_yaw(double* yaw){
        if(*yaw>M_PI)
            *yaw = *yaw - 2*M_PI;
        else if(*yaw<-M_PI)
            *yaw = *yaw + 2*M_PI;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void host_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    if(pose_init == false){
        pose_init = true;
    }
    host_mocap = *msg;

    //transfer quartenion to roll, pitch, yaw
    tf::Quaternion Q(
        host_mocap.pose.orientation.x,
        host_mocap.pose.orientation.y,
        host_mocap.pose.orientation.z,
        host_mocap.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}

void desired_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    desired_pose = *msg;
    
    if(desired_input_init == false){
        desired_input_init = true;
    }
    tf::Quaternion Q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
    desired_yaw = tf::getYaw(Q);
    // desired_yaw = 0/180*M_PI;
    bound_yaw(&desired_yaw);
}

void desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    //store odometry into global variable
    desired_vel_raw = *msg;

}

void desired_vor_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    //store odometry into global variable
    desired_vel_vor = *msg;

}


void land(geometry_msgs::PoseStamped desired_pose,double desired_yaw, geometry_msgs::TwistStamped *desired_vel, geometry_msgs::PoseStamped uav_pose)
{
    double err_x, err_y, err_z, err_yaw;
    double ux,uy,uz,uyaw;
    //compute error: desired - measurement

    if(desired_pose.pose.position.z < uav_pose.pose.position.z - 0.2){
        desired_pose.pose.position.z = uav_pose.pose.position.z - 0.2;
    }

    err_x = desired_pose.pose.position.x - uav_pose.pose.position.x;
    err_y = desired_pose.pose.position.y - uav_pose.pose.position.y;
    err_z = desired_pose.pose.position.z - uav_pose.pose.position.z;
    err_yaw = desired_yaw - yaw;

    bound_yaw( &err_yaw );

    //ROS_INFO("err: %.3f,%.3f,%.3f,%.3f", err_x, err_y, err_z, err_yaw/M_PI*180);

    ux = KPx*err_x;
    uy = KPy*err_y;
    uz = KPz*err_z;
    uyaw = KPyaw*err_yaw;

    //set max&min for control input
    /*
    if(ux<=-1.5 ||ux>=1.5)
    {
      ux = 1.5*ux/abs(ux);
    }
    if(uy<=-1.5 ||uy>=1.5)
    {
      uy = 1.5*uy/abs(uy);
    }
    */
    if(uz<=-0.4 ||uz>=0.4)
    {
      uz = 0.4*uz/abs(uz);
    }
    //output control input
    desired_vel->twist.linear.x = ux;
    desired_vel->twist.linear.y = uy;
    desired_vel->twist.linear.z = uz;
    desired_vel->twist.angular.z = uyaw;
}

void follow(geometry_msgs::PoseStamped desired_pose,double desired_yaw, geometry_msgs::TwistStamped *desired_vel, geometry_msgs::PoseStamped uav_pose)
{
    double err_x, err_y, err_z, err_yaw;
    double ux,uy,uz,uyaw;
    //compute error: desired - measurement
    err_x = desired_pose.pose.position.x - uav_pose.pose.position.x;
    err_y = desired_pose.pose.position.y - uav_pose.pose.position.y;
    err_z = desired_pose.pose.position.z - uav_pose.pose.position.z;
    err_yaw = desired_yaw - yaw;

    bound_yaw( &err_yaw );

    ROS_INFO("err: %.3f,%.3f,%.3f,%.3f", err_x, err_y, err_z, err_yaw/M_PI*180);

    ux = KPx*err_x;
    uy = KPy*err_y;
    uz = KPz*err_z;
    uyaw = KPyaw*err_yaw;

    //set max&min for control input
    /*
    if(ux<=-1.5 ||ux>=1.5)
    {
      ux = 1.5*ux/abs(ux);
    }
    if(uy<=-1.5 ||uy>=1.5)
    {
      uy = 1.5*uy/abs(uy);
    }
    */
    if(uz<=-0.4 ||uz>=0.4)
    {
      uz = 0.4*uz/abs(uz);
    }
    //output control input
    desired_vel->twist.linear.x = ux;
    desired_vel->twist.linear.y = uy;
    desired_vel->twist.linear.z = uz;
    desired_vel->twist.angular.z = uyaw;
}

void get_vor_vel(geometry_msgs::TwistStamped *desired_vel)
{
    //output control input
    desired_vel->twist.linear.x = desired_vel_vor.twist.linear.x ;
    desired_vel->twist.linear.y = desired_vel_vor.twist.linear.y ;
    desired_vel->twist.linear.z = desired_vel_vor.twist.linear.z ;
    desired_vel->twist.angular.z = desired_vel_vor.twist.angular.z ;
}

void follow_yaw(geometry_msgs::TwistStamped& desired_vel, double desired_yaw)
{
	double err_yaw, u_yaw;
	err_yaw = desired_yaw - yaw;
	bound_yaw( &err_yaw );
	u_yaw = KPyaw*err_yaw;
	desired_vel.twist.angular.z = u_yaw;
}



void start_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    start_all_drone = msg.data;
}
void kill_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    kill_all_drone = msg.data;
}
void takeoff_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    takeoff_all_drone = msg.data;
}

void gs_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    gs_state = msg.data;
}

int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "uav_controller");
    ros::NodeHandle nh,private_nh("~");

    int UAV_ID;
    ros::param::get("UAV_ID", UAV_ID);

    string use_input_s;
    if(private_nh.getParam("use_input", use_input_s) == false) {
       ROS_FATAL("No use_input is assigned.");
       //exit(0);
       use_input_s = "position";
    }   
    std::cout<< use_input_s << "\n";
    //    subscriber    //
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, host_pose_cb);

    ros::Subscriber desired_vor_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("cmd_vel_from_vor", 10, desired_vor_vel_cb);
    ros::Subscriber desired_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("desired_pose", 10, desired_pose_cb);
    ros::Subscriber desired_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("desired_velocity_raw", 10, desired_vel_cb);
    
    ros::Subscriber uav_start_sub = nh.subscribe<std_msgs::Int32>("/uav_start", 10, start_cb);
    ros::Subscriber uav_killer_sub = nh.subscribe<std_msgs::Int32>("/uav_kill", 10, kill_cb);
    ros::Subscriber uav_takeoff_sub = nh.subscribe<std_msgs::Int32>("/uav_takeoff", 10, takeoff_cb);



    ros::Subscriber gs_sub = nh.subscribe<std_msgs::Int32>("/GS_state", 10, gs_cb);

    // publisher
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 2);

    // publisher
    ros::Publisher test_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("test_output", 2);

    // service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // ros::Rate rate(100);
    ros::Rate rate(200);


    float obstacle_Gamma, obstacle_SafeDistance, MAV_Gamma, MAV_SafeDistance;
	bool cbf_mode;
    float hover_x,hover_y;
    ros::param::get("obs_gamma", obstacle_Gamma);
    ros::param::get("obs_safe_D", obstacle_SafeDistance);
    ros::param::get("MAV_gamma", MAV_Gamma);
    ros::param::get("MAV_safe_D", MAV_SafeDistance);
	ros::param::get("cbf", cbf_mode);
	ros::param::get("hover_x", hover_x);
	ros::param::get("hover_y", hover_y);

    ROS_INFO("Wait for pose and desired input init");
    while (ros::ok() && (!pose_init)) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose and desired input init, %d",pose_init);
    }
    ROS_INFO("pose initialized");
    
    ROS_INFO("Wait for FCU connection");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for FCU");
    }
    ROS_INFO("FCU connected");

    ROS_INFO("Wait for setting origin and home position...");
    string mavlink_topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/mavlink/to");
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(mavlink_topic, ros::Duration(10.0));
    ROS_INFO("Message received or timeout reached. Continuing execution.");
    sleep(2);

    ROS_INFO("Wait for UAV all takeoff signal");
    while (ros::ok()) {
        if(takeoff_all_drone == 1){
            break;
        }
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for UAV all takeoff signal");
    }
    ROS_INFO("get UAV all takeoff signal");
    
    //send a few velocity setpoints before starting
    for(int i = 0; ros::ok() && i < 20; i++){
        local_vel_pub.publish(desired_vel);
        rate.sleep();
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("GUIDED enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }

    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 0.5;
    if(takeoff_cl.call(srv_takeoff))
    {
        ROS_INFO("srv_takeoff send success %d", srv_takeoff.response.success);
    }
    else
    {
        ROS_ERROR("Takeoff failed");
	return 0;
    }
	
    sleep(10);

    ROS_INFO("Wait for UAV all start signal");
    while (ros::ok()) {
        if(start_all_drone == 1){
            break;
        }
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for UAV all start signal");
    }
    ROS_INFO("get UAV all start signal");


    while (ros::ok()) {

        // make sure velocity has been published 
        desired_vel_raw = desired_vel_init;

        //keyboard control
        if(kill_all_drone == 1){
            ROS_WARN("velocity_cbf_kill!");
            offb_set_mode.request.custom_mode = "LAND";
            set_mode_client.call(offb_set_mode);
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
            sleep(3);
            return 0;
        }

        if(gs_state==2){ // hover 
            follow(desired_pose,desired_yaw, &desired_vel_raw, host_mocap);
            std::cout << "---" << std::endl;
            std::cout << "desired pose : " << std::endl;
            std::cout << desired_pose.pose.position.x << std::endl;
            std::cout << desired_pose.pose.position.y << std::endl;
            std::cout << desired_pose.pose.position.z << std::endl;
            follow_yaw(desired_vel, M_PI/2);
        }

        if(gs_state==3){ // vor
            get_vor_vel(&desired_vel_raw);
        }


        std::cout << "--- new ---" << std::endl;
        std::cout << "ground station : " << gs_state << std::endl;


        std::cout << "---" << std::endl;
        std::cout << "now pose : " << std::endl;
        std::cout << host_mocap.pose.position.x << std::endl;
        std::cout << host_mocap.pose.position.y << std::endl;
        std::cout << host_mocap.pose.position.z << std::endl;

        // std::cout << "---" << std::endl;
        // std::cout << "desired velocity : " << std::endl;
        // std::cout << desired_vel_raw.twist.linear.x << std::endl;
        // std::cout << desired_vel_raw.twist.linear.y << std::endl;
        // std::cout << desired_vel_raw.twist.linear.z << std::endl;

        // local_vel_pub.publish(desired_vel);
        test_vel_pub.publish(desired_vel_raw);
        local_vel_pub.publish(desired_vel_raw);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



