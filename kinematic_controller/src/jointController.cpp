#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>


std::vector<double> q_tar(7,0.0);
std::vector<double> q_fbk(7,0.0);
std_msgs::Float64MultiArray q_cmd_msg;
double joint_limit, linear_limit, tolerance, dt, rate, k, a;
bool joint_received = false;


void get_q_fbk(const sensor_msgs::JointState &joint_state_msg) {
    q_fbk = joint_state_msg.position;
    joint_received = true;
}

void predict_q_cmd(){
    Eigen::Map<Eigen::VectorXd> q_tar_vec(q_tar.data(), 7);
    Eigen::Map<Eigen::VectorXd> q_fbk_vec(q_fbk.data(), 7);

    //potential field calculation
    Eigen::VectorXd q_dot_ref = k * (q_tar_vec - q_fbk_vec);

    //joint normalization
    double norm = q_dot_ref.norm();
    if(norm > joint_limit) q_dot_ref *= (joint_limit/norm);

    //integration
    Eigen::VectorXd q_cmd = q_fbk_vec + (q_dot_ref * dt) * a;
    std::vector<double> q_cmd_vec(q_cmd.data(), q_cmd.data() + q_cmd.size());
    q_cmd_msg.data = q_cmd_vec;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "joint_planner");

    ros::NodeHandle nodeHandle;
    nodeHandle.getParam("/gen3/target/joint/positions",q_tar);
    nodeHandle.getParam("/gen3/joint/max_velocity",joint_limit);
    nodeHandle.getParam("/gen3/linear/max_velocity",linear_limit);
    nodeHandle.getParam("/gen3/target/joint/tolerance",tolerance);
    nodeHandle.getParam("/publish_rate",rate);
    nodeHandle.getParam("/dt",dt);
    nodeHandle.getParam("/joint_k",k);
    nodeHandle.getParam("/intg_constant",a);
    ros::Rate loopRate(rate);

    //get current joint position
    ros::Subscriber q_fbk_subscriber;
    q_fbk_subscriber = nodeHandle.subscribe("/gen3/joint_states", 1, get_q_fbk) ;

    //publisher for desired joint position
    ros::Publisher q_cmd_publisher;
    q_cmd_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("gen3/joint_group_position_controller/command", 1) ;

    while(ros::ok()){
        
        while(!joint_received) ros::spinOnce();

        //check if target is reached
        double distance = 0.0;
        for (int i=0; i < 7;++i) distance += pow(q_fbk[i] - q_tar[i], 2);
        distance = sqrt(distance);
        ROS_INFO_STREAM("distance: " << distance);
        if (distance < tolerance) q_cmd_msg.data = q_fbk;
        else predict_q_cmd();
        q_cmd_publisher.publish(q_cmd_msg);
        joint_received = false;
        loopRate.sleep();
    }

    return 0;
}

