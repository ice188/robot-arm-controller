#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ros/package.h>

std::vector<double> q(7,0.0);
std::vector<double> q_dot(7,0.0);
std::vector<double> q_tar(7,0.0);
std::vector<double> q_effort(7,0.0);
std::vector<double> x_tar(3,0.0);
std::vector<double> x_cur(3,0.0);
std::vector<double> x_dot_cur(3,0.0);
std::vector<double> x_ref(3,0.0);
std::vector<double> x_dot_ref(3,0.0);
std::vector<double> x_ddot_ref(3,0.0);
std::vector<double> x_ddot_cmd(3,0.0);

double rate, dt, joint_limit, k0, stiffness, damping, stiffness0, damping0;
std_msgs::Float64MultiArray torque_cmd_msg;
std::string urdf_file;
Eigen::MatrixXd D, K, D0, K0;
bool redundancy;
bool received_reference = false;
bool staying = false;
bool received_joint = false;

void get_joint_state(const sensor_msgs::JointState &joint_state_msg){
    q = joint_state_msg.position;
    q_dot = joint_state_msg.velocity;
    q_effort = joint_state_msg.effort;
    received_joint = true;
}

void get_q_tar(const sensor_msgs::JointState &msg){
    q_tar = msg.position;
    staying = true;
}

void get_reference(const sensor_msgs::JointState &msg){
    staying = false;
    x_ref = msg.position;
    x_dot_ref = msg.velocity;
    x_ddot_ref = msg.effort;
    received_reference = true;
}

void compute_xddot_cmd(){
    Eigen::VectorXd x_ref_vec = Eigen::Map<Eigen::VectorXd>(x_ref.data(), 3);
    Eigen::VectorXd x_dot_ref_vec = Eigen::Map<Eigen::VectorXd>(x_dot_ref.data(), 3);
    Eigen::VectorXd x_ddot_ref_vec = Eigen::Map<Eigen::VectorXd>(x_ddot_ref.data(), 3);
    Eigen::Map<Eigen::VectorXd> x_cur_vec(x_cur.data(), 3);
    Eigen::Map<Eigen::VectorXd> x_dot_cur_vec(x_dot_cur.data(), 3);

    Eigen::VectorXd x_ddot = x_ddot_ref_vec + D * (x_dot_ref_vec - x_dot_cur_vec) + K * (x_ref_vec - x_cur_vec);
    x_ddot_cmd.assign(x_ddot.data(), x_ddot.data() + x_ddot.size());
}

void forward_kinematics(){
	pinocchio::Model model;													
	pinocchio::urdf::buildModel(urdf_file, model, false);	
	pinocchio::Data data(model); 	
    Eigen::Map<Eigen::VectorXd> q_vec(q.data(), 7);
    Eigen::Map<Eigen::VectorXd> q_dot_vec(q_dot.data(), 7);								
    pinocchio::forwardKinematics(model, data, q_vec, q_dot_vec);

    //get x_cur from q_cur
    Eigen::Vector3d x_vec = data.oMi[7].translation();	
    x_cur.assign(x_vec.data(), x_vec.data() + x_vec.size());

    //get x_dot from q_dot 
    pinocchio::computeAllTerms(model, data, q_vec, q_dot_vec) ;
    Eigen::MatrixXd J(6,7);
    pinocchio::getJointJacobian(model, data, 7, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J) ;
    J = J.topRows(3);
    Eigen::Vector3d x_dot_vec =  J * q_dot_vec;
    x_dot_cur.assign(x_dot_vec.data(), x_dot_vec.data() + x_dot_vec.size());
}

void inverse_dynamics(){
    pinocchio::Model model;													
    pinocchio::urdf::buildModel(urdf_file, model, false);	
    pinocchio::Data data(model); 
    Eigen::Map<Eigen::VectorXd> q_vec(q.data(), 7);
    Eigen::Map<Eigen::VectorXd> q_dot_vec(q_dot.data(), 7);
    Eigen::Map<Eigen::VectorXd> x_ddot_cmd_vec(x_ddot_cmd.data(), 3);
    pinocchio::computeAllTerms(model, data, q_vec, q_dot_vec) ;
    Eigen::MatrixXd J(6,7);
    Eigen::MatrixXd J_dot(6,7);
    pinocchio::getJointJacobian(model, data, 7, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J) ;
    pinocchio::getJointJacobianTimeVariation(model, data, 7, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_dot) ;
    J = J.topRows(3);
    
    J_dot = J_dot.topRows(3);

    
    Eigen::MatrixXd J_transpose_pseudo = (J * J.transpose()).inverse() * J;
    Eigen::MatrixXd J_pseudo = J_transpose_pseudo.transpose();

    // Eigen::MatrixXd J_pseudo = J.completeOrthogonalDecomposition().pseudoInverse();
    // Eigen::MatrixXd J_transpose_pseudo = J.transpose().completeOrthogonalDecomposition().pseudoInverse();
    // std::cout << "J: " << J << std::endl;
    // std::cout << "J.transpose(): " << J.transpose() << std::endl;
    // std::cout << "J.transpose() * J: " << J.transpose() * J << std::endl;
    // std::cout << "(J.transpose() * J).inverse(): " << (J.transpose() * J).inverse() << std::endl;
    // std::cout << "J_dot: " << J_dot << std::endl;
    // std::cout << "J_transpose_pseudo: " << J_transpose_pseudo << std::endl;
    // std::cout << "J_pseudo: " << J_pseudo << std::endl;

    Eigen::MatrixXd lambda = J_transpose_pseudo * data.M * J_pseudo;
    Eigen::MatrixXd eta = J_transpose_pseudo * data.nle - lambda * J_dot * q_dot_vec;
    Eigen::MatrixXd F_cmd = lambda * x_ddot_cmd_vec + eta;
    Eigen::VectorXd torque = J.transpose() * F_cmd;

    //redundancy
    if (redundancy){
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(7, 7) - J.transpose() * (J * data.M.inverse() * J.transpose()).inverse() * J * data.M.inverse();
        Eigen::VectorXd null_target = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd q_dot_ref = k0 * (null_target - q_vec);
        if (abs(q_dot_ref.norm()) > 1.2) q_dot_ref *= 1.2/q_dot_ref.norm();
        Eigen::VectorXd q_ref = q_vec + q_dot_ref * dt;
        //Eigen::VectorXd q_ddot_ref = (q_dot_ref - q_dot_vec) / (dt * b0 + 0.001);

        Eigen::VectorXd q_ddot_cmd = K0 * (q_ref - q_vec) - D0 * q_dot_vec;

        Eigen::VectorXd torque0 =  data.M * q_ddot_cmd + data.nle;
        torque += P * torque0;
    }
    std::vector<double> torque_cmd(torque.data(), torque.data() + torque.size());
    torque_cmd_msg.data = torque_cmd;
}

void stay(){
    Eigen::Map<Eigen::VectorXd> q_tar_vec(q_tar.data(), 7);
    Eigen::Map<Eigen::VectorXd> q_cur_vec(q.data(), 7);
    Eigen::Map<Eigen::VectorXd> q_dot_cur(q_dot.data(), 7);

    Eigen::VectorXd q_dot_ref = 5 * (q_tar_vec - q_cur_vec);
    Eigen::VectorXd q_ref = q_cur_vec + q_dot_ref * dt * 20;
    Eigen::VectorXd q_ddot_ref = (q_dot_ref - q_dot_cur) / (dt * 100 + 0.001);

    //compute torque
    Eigen::MatrixXd Ds = Eigen::MatrixXd::Identity(7, 7) * 3.0;
    Eigen::MatrixXd Ks = Eigen::MatrixXd::Identity(7, 7) * 3.0;
    Eigen::VectorXd q_ddot_cmd = q_ddot_ref + Ds * (q_dot_ref - q_dot_cur) + Ks * (q_ref - q_cur_vec);
    pinocchio::Model model;													
	pinocchio::urdf::buildModel(urdf_file, model, false);	
	pinocchio::Data data(model); 
    pinocchio::computeAllTerms(model, data, q_cur_vec, q_dot_cur) ;
    Eigen::VectorXd torque =  data.M * q_ddot_cmd + data.nle;

    std::vector<double> torque_cmd(torque.data(), torque.data() + torque.size());
    torque_cmd_msg.data = torque_cmd;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "task_controller");

    ros::NodeHandle nodeHandle;

    nodeHandle.getParam("/publish_rate",rate);
    ros::Rate loopRate(rate);
    nodeHandle.getParam("/dt",dt);
    nodeHandle.getParam("/gen3/joint/max_velocity",joint_limit);
    nodeHandle.getParam("/gen3/urdf_file_name",urdf_file);
    nodeHandle.getParam("/damping",damping);
    nodeHandle.getParam("/stiffness",stiffness);
    nodeHandle.getParam("/damping0",damping0);
    nodeHandle.getParam("/stiffness0",stiffness0);
    nodeHandle.getParam("/k0",k0);
    nodeHandle.getParam("/redundancy",redundancy);
    D = Eigen::MatrixXd::Identity(3, 3) * damping;
    K = Eigen::MatrixXd::Identity(3, 3) * stiffness;
    D0 = Eigen::MatrixXd::Identity(7, 7) * damping0;
    K0 = Eigen::MatrixXd::Identity(7, 7) * stiffness0;

    //subscribers and publishers
    ros::Subscriber joint_subscriber = nodeHandle.subscribe("/gen3/joint_states", 1, get_joint_state) ;
    ros::Subscriber q_tar_subscriber = nodeHandle.subscribe("/q_tar", 1, get_q_tar) ;
    ros::Subscriber x_ddot_cmd_subscriber = nodeHandle.subscribe("/planner/reference", 1, get_reference) ;
    ros::Publisher torque_cmd_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command", 1) ;
    
    while(ros::ok()){
        
        // if (staying) {
        //     stay();
        // }
        while (!received_reference || !received_joint) ros::spinOnce();
        forward_kinematics();
        compute_xddot_cmd();
        inverse_dynamics();
    
        if (torque_cmd_msg.data.size() != 0) 
            torque_cmd_publisher.publish(torque_cmd_msg);
        received_reference = false;
        received_joint = false;
        loopRate.sleep();
    }

    return 0;
}
