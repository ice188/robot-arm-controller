#include <actionlib/client/simple_action_client.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "client") ;
	ros::NodeHandle node_handle ;
    double n_targets = 3;
    node_handle.getParam("action_list/number_of_targets",n_targets);
    
    std::vector<double> target_translation(3,0.0);
    std::vector<double> target_orientation(3,0.0);
    double target_time;

    Eigen::MatrixXd targets = Eigen::MatrixXd::Zero(n_targets,7);
    
    highlevel_msgs::PoseCommandGoal pose_goal;

    actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> pose_client("/gen3/action_planner/pose",true) ;

    // get targets 

    for (int i=0;i<n_targets;++i){
        std::string prefix = "/action_list/action_" + std::to_string(i);
        node_handle.getParam(prefix+"/translation", target_translation);
        node_handle.getParam(prefix+"/orientation", target_orientation);
        node_handle.getParam(prefix+"/duration", target_time);
        
        pose_goal.x = target_translation[0];
        pose_goal.y = target_translation[1];
        pose_goal.z = target_translation[2];
        pose_goal.roll = target_orientation[0];
        pose_goal.pitch = target_orientation[1];
        pose_goal.yaw = target_orientation[2];
        pose_goal.T = target_time;
        pose_client.waitForServer();
        pose_client.sendGoal(pose_goal);
        ROS_INFO_STREAM("\n ---------------------------\n" << "Client published goal " << i << ", waiting for 30s \n ---------------------------");
        pose_client.waitForResult(ros::Duration(30.0));
    }

    ros::shutdown();
	return 0;
}


