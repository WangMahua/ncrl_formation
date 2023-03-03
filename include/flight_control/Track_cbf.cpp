#include "Track_cbf.h"

Track_CBF::Track_CBF(ros::NodeHandle nh, string self_pos_topic, string target_pos_topic)
{
    target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(target_pos_topic, 10, &Track_CBF::target_pose_cb, this);
    self_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(self_pos_topic, 10, &Track_CBF::target_pose_cb, this);
    distance_safe = distance_track = gamma = 0.5;
    pose_init = false;
}

void Track_CBF::target_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    target_pos = *msg;
}

void Track_CBF::self_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!pose_init)
        pose_init = true;
    self_pos = *msg;
}

void Track_CBF::setCBFparam(float dis_t, float dis_s, float gm)
{
    distance_track = dis_t;
    distance_safe = dis_s;
    gamma = gm;
}

float Track_CBF::getTrackDistance(){ return distance_track;}
float Track_CBF::getSafeDistance(){ return distance_safe;}
float Track_CBF::getGamma(){ return gamma;}
geometry_msgs::PoseStamped Track_CBF::getTargetPose(){ return target_pos;}

int Track_CBF::QPsolve_vel(geometry_msgs::TwistStamped desired_vel_raw, geometry_msgs::TwistStamped* desired_vel)
{
    Eigen::SparseMatrix<double> hessian_Matrix;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    hessian_Matrix.resize(3, 3);
    hessian_Matrix.insert(0, 0) = 1;
    hessian_Matrix.insert(1, 0) = 0;
    hessian_Matrix.insert(2, 0) = 0;
    hessian_Matrix.insert(0, 1) = 0;
    hessian_Matrix.insert(1, 1) = 1;
    hessian_Matrix.insert(2, 1) = 0;
    hessian_Matrix.insert(0, 2) = 0;
    hessian_Matrix.insert(1, 2) = 0;
    hessian_Matrix.insert(2, 2) = 1;

    gradient.resize(3);
    gradient << -desired_vel_raw.twist.linear.x , -desired_vel_raw.twist.linear.y, -desired_vel_raw.twist.linear.z;

    upperBound.resize(1);
    lowerBound.resize(1);
    linearMatrix.resize(1, 3);

    linearMatrix.insert(0, 0) = 2*(self_pos.pose.position.x - target_pos.pose.position.x);
    linearMatrix.insert(0, 1) = 2*(self_pos.pose.position.y - target_pos.pose.position.y);
    linearMatrix.insert(0, 2) = 2*(self_pos.pose.position.z - target_pos.pose.position.z);
    upperBound(0) = gamma*(pow(distance_track, 2)
                                -pow(self_pos.pose.position.x - target_pos.pose.position.x, 2)
                                -pow(self_pos.pose.position.y - target_pos.pose.position.y, 2)
                                -pow(self_pos.pose.position.z - target_pos.pose.position.z, 2));
    lowerBound(0) = -gamma*( pow(self_pos.pose.position.x - target_pos.pose.position.x, 2)
                            +pow(self_pos.pose.position.y - target_pos.pose.position.y, 2)
                            +pow(self_pos.pose.position.z - target_pos.pose.position.z, 2)
                            -pow(distance_safe, 2));


    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(3);
    solver.data()->setNumberOfConstraints(1);

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
    desired_vel->twist.linear.z = QPSolution(2);
    if(desired_vel->twist.linear.x > 100 || desired_vel->twist.linear.y > 100 || desired_vel->twist.linear.z > 100)
        desired_vel->twist.linear.x = desired_vel->twist.linear.y = desired_vel->twist.linear.z = 0;
    return 0;
}