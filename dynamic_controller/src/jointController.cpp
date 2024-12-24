#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

std_msgs::Float64MultiArray q_cmd;
double rate;
bool received_q_cmd=false;

void get_q_cmd(const std_msgs::Float64MultiArray &q_cmd_msg){
    q_cmd = q_cmd_msg;
    received_q_cmd=true;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joint_controller");

    ros::NodeHandle nodeHandle;

    nodeHandle.getParam("/publish_rate",rate);
    ros::Rate loopRate(rate);

    ros::Publisher q_cmd_publisher;
    ros::Subscriber q_cmd_subscriber;
    
    q_cmd_subscriber = nodeHandle.subscribe("/planner/q_cmd", 1, get_q_cmd) ;
    q_cmd_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("gen3/joint_group_effort_controller/command", 1) ;

    while(ros::ok()){

        //get desired joint torque from planner
        while (!received_q_cmd) ros::spinOnce();

        //send joint torque to robot
        q_cmd_publisher.publish(q_cmd);
        loopRate.sleep();
    }
    received_q_cmd = false;
    return 0;
}

