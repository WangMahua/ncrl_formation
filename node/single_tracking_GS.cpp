#include "ros/ros.h"
#include "std_msgs/String.h"
#include "getch.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <math.h>



enum {
	DISARM,
	HOVERING,
    TAKEOFF,
    LAND,
	TRACKING,
}LeaderMode;

int leader_mode;
int mode;
int kill_all_drone = 0;
int start_all_drone = 0;
std_msgs::Int32 kill_msg, mode_msg;
geometry_msgs::PoseStamped uav_pose;

void start_takeoff(){
	if(leader_mode == TAKEOFF || uav_pose.pose.position.z>0.1 ){
		ROS_WARN("already takeoff");
	}
	else{
		leader_mode = TAKEOFF;
		ROS_INFO("start takeoff");
		mode = 1;
	}
}

void start_land(){
	if(leader_mode == LAND || uav_pose.pose.position.z <= 0.01 ){
		ROS_WARN("already landing or it's on the land");
	}
	else{
		leader_mode = LAND;
		ROS_INFO("start landing");
		mode = 2;
	}
}

void start_track()
{
	if(leader_mode == TRACKING){
		ROS_WARN("already tracking");
	}
	else{
		leader_mode = TRACKING;
		ROS_INFO("start tracking");
		mode = 3;
	}
}

void stop()
{
	if(leader_mode == HOVERING){
		ROS_WARN("already hovering");
	}
	else{
		leader_mode = HOVERING;
		ROS_INFO("stop moving, hovering");
		mode = 4;
	}
}

void uav_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	uav_pose = *msg;
}

int main(int argc, char **argv)
{
  mode = 0;
  ros::init(argc, argv, "leader_pose_publisher");

  ros::NodeHandle nh;
  ros::Publisher uav_killer_pub = nh.advertise<std_msgs::Int32>("/uav_kill", 10);
  ros::Publisher mode_pub = nh.advertise<std_msgs::Int32>("/uav_mode", 10);
  ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose", 10, uav_pose_cb);

  ros::Rate loop_rate(100);

ROS_INFO("(t):takeoff\n (l):land\n (r):track_red_point\n (p):stop MAV\n (k):kill_all_drone\n");
  while (ros::ok())
  {
        //keyboard control
        int c = getch();
        //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
                case 116:    // (t) takeoff
                    start_takeoff();
                    break;
                case 108:    // (l) land
                    start_land();
                    break;
				case 114:    // (t) start_track_red_point
                   	start_track();
                    break;	
                case 112:    // (p) stop_trajectory_following
                	stop();
                	break;
                case 107:    // (k) uav_kill
					kill_all_drone = 1;
					ROS_WARN("kill all drone");
					break;
			}
        }
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */	

	kill_msg.data=kill_all_drone;
	mode_msg.data = mode;

	uav_killer_pub.publish(kill_msg);
	mode_pub.publish(mode_msg);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
