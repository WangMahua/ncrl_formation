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
double desired_yaw = 0;
int kill_all_drone = 0;
int start_all_drone = 0;
int takeoff_all_drone = 0;
int gs_state = 0;
// var for desired_velocity
geometry_msgs::TwistStamped desired_vel_raw;
geometry_msgs::TwistStamped desired_vel;        //output
geometry_msgs::TwistStamped desired_vel_init;        //output

// var for obstacle
//sgeometry_msgs::PoseStamped obstacle_pose;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;

class CBF_object
{
private:
    geometry_msgs::PoseStamped pose;
    ros::Subscriber pose_sub;
    bool exist;
    float safeDistance;
    float gamma;
    int id;
    queue<geometry_msgs::PoseStamped> pose_queue;
public:
    CBF_object(ros::NodeHandle nh, string subTopic, float safe_D, float gm, int ID);
    CBF_object(ros::NodeHandle nh, string subTopic);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped getPose();
    bool getExist();
    float getSafeDistance();
    float getGamma();

    static int self_id;
    static int delay_step;
};

int CBF_object::self_id = 0;
int CBF_object::delay_step = 0;

CBF_object::CBF_object(ros::NodeHandle nh, string subTopic, float safe_D, float gm, int ID)
{
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(subTopic, 10, &CBF_object::pose_cb, this);
    exist = false;
    safeDistance = safe_D;
    gamma = gm;
    id = ID;
}
CBF_object::CBF_object(ros::NodeHandle nh, string subTopic)
{
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(subTopic, 10, &CBF_object::pose_cb, this);
    exist = false;
}

void CBF_object::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    exist = true;
    if(id != self_id)
    {
	pose_queue.push(*msg);
	if(pose_queue.size() >= delay_step)
	{	
	    pose = pose_queue.front();
	    pose_queue = queue<geometry_msgs::PoseStamped>();
	}
    }
    else
	pose = *msg;
}

geometry_msgs::PoseStamped CBF_object::getPose(){return pose;}

bool CBF_object::getExist(){return exist;}

float CBF_object::getSafeDistance(){return safeDistance;}

float CBF_object::getGamma(){return gamma;}


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
    bound_yaw(&desired_yaw);
}

void desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    //store odometry into global variable
    desired_vel_raw = *msg;
    
    if(desired_input_init == false){
        desired_input_init = true;
    }
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

void takeoff(geometry_msgs::PoseStamped desired_pose,double desired_yaw, geometry_msgs::TwistStamped *desired_vel, geometry_msgs::PoseStamped uav_pose)
{
    double err_x, err_y, err_z, err_yaw;
    double ux,uy,uz,uyaw;
    //compute error: desired - measurement

    if(desired_pose.pose.position.z > uav_pose.pose.position.z + 0.2){
        desired_pose.pose.position.z = uav_pose.pose.position.z + 0.2;
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


void follow_yaw(geometry_msgs::TwistStamped& desired_vel, double desired_yaw)
{
	double err_yaw, u_yaw;
	err_yaw = desired_yaw - yaw;
	bound_yaw( &err_yaw );
	u_yaw = KPyaw*err_yaw;
	desired_vel.twist.angular.z = u_yaw;
}

int velocity_cbf(geometry_msgs::TwistStamped desired_vel_raw,geometry_msgs::TwistStamped* desired_vel, CBF_object cbO[]){

            Eigen::SparseMatrix<double> hessian_Matrix;
            Eigen::VectorXd gradient;
            Eigen::SparseMatrix<double> linearMatrix;
            Eigen::VectorXd lowerBound;
            Eigen::VectorXd upperBound;

            hessian_Matrix.resize(2,2);
            hessian_Matrix.insert(0,0) = 1;
            hessian_Matrix.insert(1,0) = 0;
            hessian_Matrix.insert(0,1) = 0;
            hessian_Matrix.insert(1,1) = 1;

            gradient.resize(2);
            gradient << - desired_vel_raw.twist.linear.x , - desired_vel_raw.twist.linear.y;
	   
	    int cbf_num = 0;
	    for(int i = 0; i < 5; i++)
	    {
	        if(cbO[i].getExist() == true)
		      cbf_num++;
	    }
	   ////////////////////// cbf constraints ///////////////////////// 
            upperBound.resize(cbf_num-1);
            lowerBound.resize(cbf_num-1);
            linearMatrix.resize(cbf_num-1,2);
            int j = 0;
            for(int i = 0; i < 5; i++)
            {
                if(i != CBF_object::self_id && cbO[i].getExist() == true)
                {
                    linearMatrix.insert(j,0) = 2*(cbO[i].getPose().pose.position.x - host_mocap.pose.position.x );
                    linearMatrix.insert(j,1) = 2*(cbO[i].getPose().pose.position.y - host_mocap.pose.position.y );
            	    upperBound(j) = cbO[i].getGamma()*(pow((cbO[i].getPose().pose.position.x - host_mocap.pose.position.x ),2)+
           			 pow((cbO[i].getPose().pose.position.y - host_mocap.pose.position.y ),2)-
            			 pow(cbO[i].getSafeDistance(),2));
		    lowerBound(j) = -OsqpEigen::INFTY;

		    j++;
                }   
            }

            OsqpEigen::Solver solver;
            solver.settings()->setWarmStart(true);
            solver.settings()->setVerbosity(false);

            solver.data()->setNumberOfVariables(2);
            solver.data()->setNumberOfConstraints(cbf_num-1);

            if(!solver.data()->setHessianMatrix(hessian_Matrix)) return 1;
            if(!solver.data()->setGradient(gradient)) return 1;
            if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
            if(!solver.data()->setLowerBound(lowerBound)) return 1;
            if(!solver.data()->setUpperBound(upperBound)) return 1;
            if(!solver.initSolver()) return 1;
            if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

            Eigen::VectorXd QPSolution;
            QPSolution = solver.getSolution();
            *desired_vel = desired_vel_raw;
            desired_vel->twist.linear.x = QPSolution(0);
            desired_vel->twist.linear.y = QPSolution(1);

            return 0;
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
    ros::param::get("delay_step", CBF_object::delay_step);

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
    ros::Rate rate(100);

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

    // CBF_object::self_id = UAV_ID;
    // CBF_object cbO[5] = {CBF_object(nh, "/vrpn_client_node/obstacle/pose",obstacle_SafeDistance, obstacle_Gamma, 0),
    //                      CBF_object(nh, "/MAV1/mavros/local_position/pose", MAV_SafeDistance, MAV_Gamma, 1),
    //                      CBF_object(nh, "/MAV2/mavros/local_position/pose", MAV_SafeDistance, MAV_Gamma, 2),
    //                      CBF_object(nh, "/MAV3/mavros/local_position/pose", MAV_SafeDistance, MAV_Gamma, 3),
    //                      CBF_object(nh, "/MAV4/mavros/local_position/pose", MAV_SafeDistance, MAV_Gamma, 4)};

    ROS_INFO("Wait for pose and desired input init");
    while (ros::ok() && (!desired_input_init || !pose_init)) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose and desired input init %d,%d",desired_input_init,pose_init);
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
    srv_takeoff.request.altitude = 0.7;
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
        if (current_state.mode != "GUIDED" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    
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
        //ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z, desired_yaw/M_PI*180);
        //follow desired_pose
        // if(use_input_s == "position"){
        //     follow(desired_pose,desired_yaw, &desired_vel_raw, host_mocap);
        // }
        
        // //avoid collicsion
        // //ROS_INFO("origin input:vx: %f vy: %f \n",desired_vel.twist.linear.x,desired_vel.twist.linear.y); 
         
        // //is_obstacle_exit
		// if(cbf_mode){
		// 	if(( ros::Time::now() - cbO[0].getPose().header.stamp)<ros::Duration(0.5)){
		// 		if(velocity_cbf( desired_vel_raw , &desired_vel, cbO)!=0){
		// 			desired_vel = desired_vel_raw;
		// 		}
		// 		//  ROS_INFO("cbf input:vx: %f vy: %f \n",desired_vel.twist.linear.x,desired_vel.twist.linear.y); 

		// 	}
		// 	else{
		// 		desired_vel = desired_vel_raw;
		// 	}
		// }else{
		// 	desired_vel = desired_vel_raw;
		// }

        // if(gs_state==1){ // takeoff 
        //     takeoff(desired_pose,desired_yaw, &desired_vel_raw, host_mocap);
        // }
        if(gs_state==2){ // hover 
            follow(desired_pose,desired_yaw, &desired_vel_raw, host_mocap);
        }
        if(gs_state==3){

        }

        follow_yaw(desired_vel, M_PI/2);

        std::cout << "---" << std::endl;
        std::cout << desired_vel.twist.linear.x << std::endl;
        std::cout << desired_vel.twist.linear.y << std::endl;
        std::cout << desired_vel.twist.linear.z << std::endl;

        // local_vel_pub.publish(desired_vel);
        test_vel_pub.publish(desired_vel);
        local_vel_pub.publish(desired_vel_init);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



