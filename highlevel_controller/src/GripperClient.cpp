#include <actionlib/client/simple_action_client.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <string.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "client") ;
	ros::NodeHandle node_handle ;
    double n_targets = 3;
    
    
    node_handle.getParam("action_list/number_of_targets",n_targets);

    std::vector<double> target_translation(3,0.0);
    std::vector<double> target_orientation(3,0.0);
    double target_time;
    
    highlevel_msgs::PoseCommandGoal pose_goal;

    actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> pose_client("/gen3/action_planner/pose",true) ;

    // a8 
    std::string type;
    double position;
    double effort;
    double T;

    control_msgs::GripperCommandGoal gripper_goal;
    control_msgs::GripperCommand gripper_command;

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client("/gen3/finger_group_action_controller/gripper_cmd",true) ;

    // get targets 

    for (int i=0;i<n_targets;++i){

        std::string prefix = "/action_list/action_" + std::to_string(i);

        node_handle.getParam(prefix+"/action_type", type);

        if (type.compare("pose") == 0){
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
            pose_client.waitForResult(ros::Duration(30.0));

        }else if(type.compare("gripper") == 0){
            node_handle.getParam(prefix+"/position", position);
            node_handle.getParam(prefix+"/effort", effort);
            node_handle.getParam(prefix+"/duration", T);

            gripper_command.position = position;
            gripper_command.max_effort = effort;
            gripper_goal.command = gripper_command;
            gripper_client.waitForServer();
            gripper_client.sendGoal(gripper_goal);
            pose_client.waitForResult(ros::Duration(30.0));
        }
        
        
    }

    ros::shutdown();
	return 0;
}


