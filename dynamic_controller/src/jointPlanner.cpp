#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>


std::vector<double> target(7,0.0);
std::vector<double> current_pose(7,0.0);
std::vector<double> current_velocity(7,0.0);
std::vector<double> current_effort(7,0.0);
std_msgs::Float64MultiArray q_cmd_msg;
double joint_limit, linear_limit, angular_limit, a, b, tolerance, damping, stiffness, k, rate, dt;
Eigen::MatrixXd D, K;
std::string urdf_file;
bool received_joint = false;


void get_joint_state(const sensor_msgs::JointState &joint_state_msg) {
    current_pose = joint_state_msg.position;
    current_velocity = joint_state_msg.velocity;
    current_effort = joint_state_msg.effort;
    received_joint = true;
}

void predict_q_cmd(){
    Eigen::Map<Eigen::VectorXd> q_tar(target.data(), 7);
    Eigen::Map<Eigen::VectorXd> q_cur(current_pose.data(), 7);
    Eigen::Map<Eigen::VectorXd> q_dot_cur(current_velocity.data(), 7);

    //potential field
    Eigen::VectorXd q_dot_ref = k * (q_tar - q_cur);
    if (abs(q_dot_ref.norm()) > 1.2) q_dot_ref *= 1.2/q_dot_ref.norm();

    //integration
    Eigen::VectorXd q_ref = q_cur + q_dot_ref * dt * a;

    //differentiation
    Eigen::VectorXd q_ddot_ref = (q_dot_ref - q_dot_cur) / (dt * b + 0.001);

    //compute torque
    Eigen::VectorXd q_ddot_cmd = q_ddot_ref + D * (q_dot_ref - q_dot_cur) + K * (q_ref - q_cur);
    pinocchio::Model model;													
	pinocchio::urdf::buildModel(urdf_file, model, false);	
	pinocchio::Data data(model); 
    pinocchio::computeAllTerms(model, data, q_cur, q_dot_cur) ;

    Eigen::VectorXd torque =  data.M * q_ddot_cmd + data.nle;

   
    // std::cout << "q_cur: " << q_cur << std::endl;
    // std::cout << "q_ref: " << q_ref << std::endl;
    // std::cout << "q_dot_ref: " << q_dot_ref << std::endl;
    // std::cout << "q_ddot_ref: " << q_ddot_ref << std::endl;
    // std::cout << "Ma+h (torque): " << torque << std::endl;
    std::vector<double> q_cmd(torque.data(), torque.data() + torque.size());
    q_cmd_msg.data = q_cmd;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "joint_planner");

    ros::NodeHandle nodeHandle;

    // read target joint position
    nodeHandle.getParam("/publish_rate",rate);
    nodeHandle.getParam("/dt",dt);
    nodeHandle.getParam("/gen3/target/joint/positions",target);
    nodeHandle.getParam("/gen3/joint/max_velocity",joint_limit);
    nodeHandle.getParam("/gen3/linear/max_velocity",linear_limit);
    nodeHandle.getParam("/gen3/angular/max_velocity",angular_limit);
    nodeHandle.getParam("/gen3/target/joint/tolerance",tolerance);
    nodeHandle.getParam("/damping",damping);
    nodeHandle.getParam("/stiffness",stiffness);
    nodeHandle.getParam("/potential_field_constant",k);
    nodeHandle.getParam("/gen3/urdf_file_name",urdf_file);
    nodeHandle.getParam("/a",a);
    nodeHandle.getParam("/b",b);
    D = Eigen::MatrixXd::Identity(7, 7) * damping;
    K = Eigen::MatrixXd::Identity(7, 7) * stiffness;

    ros::Rate loopRate(rate);

    //get current joint position
    ros::Subscriber joint_state_subscriber;
    joint_state_subscriber = nodeHandle.subscribe("/gen3/joint_states", 1, get_joint_state) ;

    //publisher for desired joint position
    ros::Publisher q_cmd_publisher;
    q_cmd_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/planner/q_cmd", 1) ;

    
    while(ros::ok()){
        
        while(!received_joint) ros::spinOnce();

        //check if target is reached
        double distance = 0.0;
        for (int i=0; i < 7;++i){
            distance += pow(current_pose[i] - target[i], 2);
        } 
        distance = sqrt(distance);
        ROS_INFO_STREAM("distance: " << distance);
        predict_q_cmd();
        q_cmd_publisher.publish(q_cmd_msg);

        loopRate.sleep();
    }
    received_joint = false;
    return 0;
}

