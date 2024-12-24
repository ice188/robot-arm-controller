#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <actionlib/server/simple_action_server.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <ros/ros.h>

Eigen::VectorXd q(7);
Eigen::VectorXd q_dot(7);
bool state_received;

class PlannerServer{
    public:
        pinocchio::SE3 pose_cur, pose_ref;
        Eigen::VectorXd x_ini;
        Eigen::VectorXd x_ref;
        Eigen::VectorXd x_dot_ref;
        Eigen::VectorXd cubic_twist;
        Eigen::VectorXd cubic_pose;
        Eigen::Quaterniond ori_ini, ori_tar, ori_ref, ori_cur, ori_cubic;
        Eigen::VectorXd pose_err_vec_local;
        Eigen::VectorXd pose_err_vec_world;
        double t = 0.0;
        double t_ini;
        double T_ = 0.0;
        geometry_msgs::Pose pose_msg;
        geometry_msgs::Twist twist_msg;
        pinocchio::Model model_;
        pinocchio::Data data_;

        PlannerServer(ros::NodeHandle &node_handle_, std::string urdf) : 
            node_handle_(node_handle_), 
            loop_rate_(500),
            initialize_(true),
            planner_action_server_(node_handle_, "/gen3/action_planner/pose", boost::bind(&PlannerServer::planner_callback, this, _1), false),
            cubic_twist(3),
            cubic_pose(3),
            x_ini(3),
            x_ref(3),
            x_dot_ref(6),
            pose_err_vec_local(6),
            pose_err_vec_world(6) {

            pose_publisher_ = node_handle_.advertise<geometry_msgs::Pose>("/gen3/reference/pose", 1) ;
            twist_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/gen3/reference/twist", 1) ;
            q_cmd_publisher_ = node_handle_.advertise<std_msgs::Float64MultiArray>("gen3/joint_group_position_controller/command", 1) ;

            pinocchio::urdf::buildModel(urdf, model_, false);	
            pinocchio::Data data(model_);
            data_ = data;
            planner_action_server_.start();
            
        }

        void planner_callback(const highlevel_msgs::PoseCommandGoalConstPtr& goal){

            ROS_INFO_STREAM("\n ---------------------------\n" << "Server received goal, executing \n---------------------------");
            pose_tar_.translation() << goal->x, goal->y, goal->z;
            pose_tar_.rotation() << pinocchio::rpy::rpyToMatrix(goal->roll, goal->pitch, goal->yaw);
            T_ = goal->T;
            t_ini = ros::Time::now().toSec();
            feedback_.time_elapsed = ros::Time::now().toSec() - t_ini;
            initialize_ = true;

            while (feedback_.time_elapsed <T_) {
                loop_rate_.sleep();
            }
    
            planner_action_server_.setSucceeded();
            ROS_INFO_STREAM("\n ---------------------------\n" << "Server finished goal, returning to client \n---------------------------");
        }

        void update(){
            pinocchio::forwardKinematics(model_, data_, q, q_dot);
            pose_cur = data_.oMi[7];

            if (initialize_) {
                x_ini = pose_cur.translation();
                pinocchio::quaternion::assignQuaternion(ori_ini, pose_cur.rotation());
                pinocchio::quaternion::assignQuaternion(ori_tar, pose_tar_.rotation());
                
                initialize_ = false;
            }

            // feedback
            feedback_.time_elapsed = ros::Time::now().toSec() - t_ini;
            ROS_INFO_STREAM("time: " << feedback_.time_elapsed);

            double distance_tra = 0.0;
            for (int i=0; i<3;++i) distance_tra += pow(pose_cur.translation()[i] - pose_tar_.translation()[i],2);
            distance_tra = sqrt(distance_tra);
            pinocchio::quaternion::assignQuaternion(ori_cur, pose_cur.rotation());
            feedback_.distance_translation = distance_tra;
            feedback_.distance_orientation = pinocchio::quaternion::angleBetweenQuaternions(ori_cur,ori_tar);
            planner_action_server_.publishFeedback(feedback_);

            ROS_INFO_STREAM("distance tra: " << distance_tra);
            ROS_INFO_STREAM("angle ori: " << feedback_.distance_orientation);
            
        
            double c = (3 * pow(feedback_.time_elapsed,2)) / (pow(T_,2)) - (2 * pow(feedback_.time_elapsed,3)) / (pow(T_,3));
            double c2 = (6 * feedback_.time_elapsed) / (pow(T_,2)) - (6 * pow(feedback_.time_elapsed,2)) / (pow(T_,3));

            cubic_pose = x_ini + c * (pose_tar_.translation() - x_ini);
            cubic_twist = c2 * (pose_tar_.translation() - x_ini);

            ori_cubic = ori_ini.slerp(c, ori_tar);
            pose_ref.rotation() = ori_ref.toRotationMatrix();
            pose_err_vec_local = pinocchio::log6(pose_cur.inverse() * pose_ref).toVector();
            pose_err_vec_world.tail<3>() = pose_cur.rotation() * pose_err_vec_local.tail<3>();


            //publish

            pose_msg.position.x = cubic_pose[0];
            pose_msg.position.y = cubic_pose[1];
            pose_msg.position.z = cubic_pose[2];
            pose_msg.orientation.x = ori_cubic.x();
            pose_msg.orientation.y = ori_cubic.y();
            pose_msg.orientation.z = ori_cubic.z();
            pose_msg.orientation.w = ori_cubic.w();

            twist_msg.linear.x = cubic_twist[0];
            twist_msg.linear.y = cubic_twist[1];
            twist_msg.linear.z = cubic_twist[2];
            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;

            // publish pose and twist

            pose_publisher_.publish(pose_msg);
            twist_publisher_.publish(twist_msg);

            if (feedback_.time_elapsed>=T_){

                //translation

                for (int i=0;i<3;++i) x_dot_ref[i] = 5.0 * (pose_tar_.translation()[i] - pose_cur.translation()[i]);

                //orientation

                ori_ref = ori_tar;
                pose_ref.rotation() = ori_ref.toRotationMatrix();
                pose_err_vec_local = pinocchio::log6(pose_cur.inverse() * pose_ref).toVector();
                pose_err_vec_world.tail<3>() = pose_cur.rotation() * pose_err_vec_local.tail<3>();
                for (int i=3;i<6;++i) x_dot_ref[i] = 5.0 * pose_err_vec_world[i];

            }else{

                // translation

                double c = (3 * pow(feedback_.time_elapsed,2)) / (pow(T_,2)) - (2 * pow(feedback_.time_elapsed,3)) / (pow(T_,3));
                x_ref = x_ini + c * (pose_tar_.translation() - x_ini);
                for (int i=0; i<3; ++i) x_dot_ref[i] = 5.0 * (x_ref[i] - pose_cur.translation()[i]);
                
                // orientation

                ori_ref = ori_ini.slerp(c, ori_tar);
                pose_ref.rotation() = ori_ref.toRotationMatrix();
                pose_err_vec_local = pinocchio::log6(pose_cur.inverse() * pose_ref).toVector();
                pose_err_vec_world.tail<3>() = pose_cur.rotation() * pose_err_vec_local.tail<3>();

                for (int i=3;i<6;++i) x_dot_ref[i] = 5.0 * pose_err_vec_world[i];
            }

            // controller
            
            Eigen::MatrixXd J(6,7);
            Eigen::MatrixXd J_pinv, N;
            Eigen::VectorXd q_dot_cmd(7);
            Eigen::VectorXd q_cmd(7);
            Eigen::VectorXd q_def(7);
            q_def.setZero();
            std_msgs::Float64MultiArray q_cmd_msg;
            q_cmd_msg.data.resize(7);
            
            pinocchio::computeJointJacobians(model_, data_, q) ;
            pinocchio::getJointJacobian(model_, data_, 7, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J) ;
            J_pinv = J.transpose() * ((J * J.transpose()).inverse());
            q_dot_cmd = J_pinv * x_dot_ref;

            // redundancy
            N = Eigen::MatrixXd::Identity(7, 7) - J_pinv * J;
            q_dot_cmd += (N * 5.0 * (q_def - q));
            
            q_cmd = q + (q_dot_cmd * 1.0/500) * 30.0;
            for (int i=0;i<7;++i) q_cmd_msg.data[i] = q_cmd[i];
        

            q_cmd_publisher_.publish(q_cmd_msg);
            loop_rate_.sleep();

        }  
        private:
            ros::NodeHandle node_handle_;
            ros::Publisher pose_publisher_, twist_publisher_, q_cmd_publisher_;
            highlevel_msgs::PoseCommandFeedback feedback_;
            pinocchio::SE3 pose_tar_;
            ros::Rate loop_rate_;
            bool initialize_;
            actionlib::SimpleActionServer<highlevel_msgs::PoseCommandAction> planner_action_server_;
};

void get_q(const sensor_msgs::JointState &joint_msg){
    for (int i=0; i<7; ++i) {
        q[i] = joint_msg.position[i];
        q_dot[i] = joint_msg.velocity[i];
    }
    state_received = true;    
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "planner_server");
	
    //vars
    ros::NodeHandle nodeHandle;
	ros::Rate loopRate(500);
    std::string urdf_file;

    // paramters
    nodeHandle.getParam("/gen3/urdf_file_name", urdf_file);

    // subscriber & publisher
    ros::Subscriber joint_subscriber = nodeHandle.subscribe("/gen3/joint_states", 1, get_q) ;

    while (!state_received) ros::spinOnce();

    // server
    PlannerServer planner_action_server(nodeHandle, urdf_file) ;

	while (ros::ok()) {
		while (!state_received) ros::spinOnce();
        if (planner_action_server.T_ != 0.0) planner_action_server.update();
        state_received = false;
		loopRate.sleep();
	}
	return 0;
}

