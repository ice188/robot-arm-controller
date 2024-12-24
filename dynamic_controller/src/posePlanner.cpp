#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <highlevel_msgs/MoveTo.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ros/package.h>

std::vector<double> x_ini(3,0.0);
std::vector<double> x_tar(3,0.0);
std::vector<double> x_cur(3,0.0);
std::vector<double> x_dot_cur(3,0.0);
std::vector<double> q_cur(7,0.0);
std::vector<double> q_dot(7,0.0);
std::vector<double> q_tar(7,0.0);
std::vector<double> q_effort(7,0.0);

ros::ServiceServer move_service;
sensor_msgs::JointState ref_msg;
sensor_msgs::JointState q_tar_msg;
std_msgs::Float64MultiArray torque_cmd_msg;
bool get_ini = true;
bool target_set = false;
double T, t_start, t, linear_limit, rate, stopping_tolerance;
std::string urdf_file;
bool received_joint = false;


void get_joint_state(const sensor_msgs::JointState &joint_state_msg){
    q_cur = joint_state_msg.position;
    q_dot = joint_state_msg.velocity;
    q_effort = joint_state_msg.effort;
    received_joint = true;
}


void forward_kinematics(){
	pinocchio::Model model;													
	pinocchio::urdf::buildModel(urdf_file, model, false);	
	pinocchio::Data data(model); 	
    Eigen::Map<Eigen::VectorXd> q_vec(q_cur.data(), 7);		
    Eigen::Map<Eigen::VectorXd> q_dot_vec(q_dot.data(), 7);						
    pinocchio::forwardKinematics(model, data, q_vec, q_dot_vec);

    //get x_cur from q_cur
    Eigen::Vector3d x_vec = data.oMi[7].translation();	
    x_cur.assign(x_vec.data(), x_vec.data() + x_vec.size());
}

bool start_planning( highlevel_msgs::MoveTo::Request  &req,
				 	 highlevel_msgs::MoveTo::Response &res) {
    if (req.z<=0 || req.T<=0) {
        res.success = false;
        return false;
    }
    //set target
    x_tar[0] = req.x;
    x_tar[1] = req.y;
    x_tar[2] = req.z;
    T = req.T;
    t_start = ros::Time::now().toSec();
    get_ini = true;

    target_set = false;
    res.success = true;
    return true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "task_planner");

    ros::NodeHandle nodeHandle;

    //subscriber and publisher
    ros::Subscriber joint_subscriber = nodeHandle.subscribe("/gen3/joint_states", 1, get_joint_state) ;
    ros::Publisher planner_publisher = nodeHandle.advertise<sensor_msgs::JointState>("/planner/reference", 1) ;
    ros::Publisher q_tar_publisher = nodeHandle.advertise<sensor_msgs::JointState>("/q_tar", 1) ;

    // service
    move_service = nodeHandle.advertiseService("/pose_planner/move_to", start_planning);

    // paramters
    nodeHandle.getParam("/publish_rate",rate);
    nodeHandle.getParam("/gen3/linear/max_velocity",linear_limit);
    nodeHandle.getParam("/gen3/urdf_file_name",urdf_file);
    nodeHandle.getParam("/stopping_tolerance",stopping_tolerance);

    ros::Rate loopRate(rate);

    //initialize target
    nodeHandle.getParam("/gen3/target/hand/position",x_tar);
    T = 5.0;
    t_start = ros::Time::now().toSec();
    
    while(ros::ok()){
        while (!received_joint) ros::spinOnce();
        forward_kinematics(); //get x_cur
        if(get_ini){
            x_ini = x_cur;
            get_ini = false;
        }
        double distance = 0.0;
        for (int i=0; i < 3;++i) distance += pow(x_cur[i] - x_tar[i], 2);
        distance = sqrt(distance);
        ROS_INFO_STREAM("distance: " << distance);

        t = ros::Time::now().toSec() - t_start;
        std::cout << "time: " << t << std::endl;
        if(t>=T || distance < stopping_tolerance){
            std::cout << "pausing" << std::endl;
            ref_msg.position = x_cur;
            std::vector<double> zeros(3,0.0);
            ref_msg.velocity = zeros;
            ref_msg.effort = zeros;
            planner_publisher.publish(ref_msg);
        }else{
            double c1 = (3 * pow(t,2)) / (pow(T,2)) - (2 * pow(t,3)) / (pow(T,3));
            double c2 = (6 * t) / pow(T,2) - (6 * pow(t,2)) / (pow(T,3));
            double c3 = (6) / pow(T,2) - (12 * t) / pow(T,3);
            Eigen::Map<Eigen::VectorXd> x_tar_vec(x_tar.data(), 3);
            Eigen::Map<Eigen::VectorXd> x_cur_vec(x_cur.data(), 3);
            Eigen::Map<Eigen::VectorXd> x_ini_vec(x_ini.data(), 3);
            
            Eigen::VectorXd x_cub = x_ini_vec + c1 * (x_tar_vec - x_ini_vec) ;
            Eigen::VectorXd x_dot_cub = c2 * (x_tar_vec - x_ini_vec);
            Eigen::VectorXd x_ddot_cub = c3 * (x_tar_vec - x_ini_vec);

            std::vector<double> position_vector(x_cub.data(), x_cub.data() + x_cub.size());
            std::vector<double> velocity_vector(x_dot_cub.data(), x_dot_cub.data() + x_dot_cub.size());
            std::vector<double> effort_vector(x_ddot_cub.data(), x_ddot_cub.data() + x_ddot_cub.size());
        
            ref_msg.position = position_vector;
            ref_msg.velocity = velocity_vector;
            ref_msg.effort = effort_vector;
            planner_publisher.publish(ref_msg);
       }
        received_joint = false;
        loopRate.sleep();
    }
    return 0;
}
