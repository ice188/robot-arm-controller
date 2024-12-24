#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

#include <highlevel_msgs/MoveTo.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>

Eigen::VectorXd q(7);
Eigen::VectorXd q_dot(7);
Eigen::VectorXd x_tar(3);
double T;
bool initialize = true;

bool state_received;

void get_q(const sensor_msgs::JointState &msg){

    for (int i=0; i<7; ++i) {
        q[i] = msg.position[i];
        q_dot[i] = msg.velocity[i];
    }
    state_received = true;    
    
}

bool start_planning( highlevel_msgs::MoveTo::Request  &req,
				 	 highlevel_msgs::MoveTo::Response &res) {

    x_tar[0] = req.x;
    x_tar[1] = req.y;
    x_tar[2] = req.z;
    T = req.T;

    initialize = true;
    res.success = true;
    return true;

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "task_controller");
    ros::NodeHandle nodeHandle;
    ros::Subscriber joint_subscriber = nodeHandle.subscribe("/gen3/joint_states", 1, get_q) ;
    ros::ServiceServer move_service = nodeHandle.advertiseService("/pose_planner/move_to", start_planning);
    ros::Publisher q_cmd_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("gen3/joint_group_position_controller/command", 1) ;

    //vars
    std::string urdf_file;
    double rate, linear_limit, joint_limit;
    double k, k0, a;
    bool redundancy, normalize;

    double t_ini, t;
    Eigen::VectorXd x(3);
    Eigen::VectorXd x_ini(3);
    Eigen::VectorXd cubic_twist(3);
    Eigen::VectorXd cubic_pose(3);
    
    Eigen::MatrixXd J(3,7);
    Eigen::MatrixXd J6(6,7);
    Eigen::MatrixXd J_pinv, N;
    Eigen::VectorXd q_dot_cmd(7);
    Eigen::VectorXd q_cmd(7);
    Eigen::VectorXd q_def(7);
    q_def.setZero();
    std_msgs::Float64MultiArray q_cmd_msg;
    q_cmd_msg.data.resize(7);

    //parameter
    nodeHandle.getParam("/publish_rate",rate);
    ros::Rate loopRate(rate);
    nodeHandle.getParam("/gen3/urdf_file_name", urdf_file);
    nodeHandle.getParam("/potential_constant", k);
    nodeHandle.getParam("/intg_constant2", a);
    nodeHandle.getParam("/redundancy_constant", k0);
    nodeHandle.getParam("/redundancy", redundancy);
    nodeHandle.getParam("/normalize", normalize);
    nodeHandle.getParam("/gen3/linear/max_velocity",linear_limit);
    nodeHandle.getParam("/gen3/joint/max_velocity",joint_limit);

    //pinocchio
    pinocchio::Model model;													
	pinocchio::urdf::buildModel(urdf_file, model, false);	
	pinocchio::Data data(model); 	

    //start default
    std::vector<double> x_target(3,0.0);
    nodeHandle.getParam("/gen3/target/hand/position",x_target);
    for(int i=0;i<3;++i) x_tar[i] = x_target[i];
    T = 5.0;
    
    while(ros::ok()){

        while (!state_received) ros::spinOnce();

        pinocchio::forwardKinematics(model, data, q, q_dot);
        x = data.oMi[7].translation();	

        if(initialize){
            x_ini = x;
            t_ini = ros::Time::now().toSec();
            initialize = false;
        }

        double distance = 0.0;
        for (int i=0; i<3;++i) distance += pow(x[i] - x_tar[i], 2);
        distance = sqrt(distance);
        ROS_INFO_STREAM("distance: " << distance);

        t = ros::Time::now().toSec() - t_ini;
        ROS_INFO_STREAM("time: " << t);

        if (t>=T){

            cubic_twist = k * (x_tar - x);

        }else{
            double c = (3 * pow(t,2)) / (pow(T,2)) - (2 * pow(t,3)) / (pow(T,3));
            
            cubic_pose = x_ini + c * (x_tar - x_ini);
            cubic_twist = k * (cubic_pose - x);
        }

        if (normalize)
            if(cubic_twist.norm() > linear_limit) cubic_twist *= (linear_limit/cubic_twist.norm());

        //controller
        
        pinocchio::computeJointJacobians(model, data, q) ;

        pinocchio::getJointJacobian(model, data, 7, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J6) ;
        
        J = J6.topRows(3);

        J_pinv = J.transpose() * ((J * J.transpose()).inverse());

        q_dot_cmd = J_pinv * cubic_twist;

        if (normalize)

            if(q_dot_cmd.norm() > joint_limit) q_dot_cmd *= (joint_limit/q_dot_cmd.norm());

        if (redundancy){

            N = Eigen::MatrixXd::Identity(7, 7) - J_pinv * J;
            q_dot_cmd += (N * k0 * (q_def - q));
            
        }
        q_cmd = q + (q_dot_cmd * (1.0/rate)) * a;

        for (int i=0;i<7;++i)
            q_cmd_msg.data[i] = q_cmd[i];

        q_cmd_publisher.publish(q_cmd_msg);

        state_received = false;
        loopRate.sleep();

    }
    return 0;
}