#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_velocity;

void keyboard_input_callback ( const std_msgs::String& msg) {

    geometry_msgs::Twist velocity_msg;
    //ROS_INFO_STREAM("message passed to callback: " << msg.data.c_str());
    if (strcmp(msg.data.c_str(),"i")==0) {
	//ROS_INFO_STREAM("setting velocity for i...");
        velocity_msg.linear.x = 0.5;
        velocity_msg.angular.z = 0;
    } else if (strcmp(msg.data.c_str(),"u")==0) {
	//ROS_INFO_STREAM("setting velocity for u...");
        velocity_msg.linear.x = 0.5;
        velocity_msg.angular.z = 0.5;
    } else if (strcmp(msg.data.c_str(),"o")==0) {
	//ROS_INFO_STREAM("setting velocity for o...");
        velocity_msg.linear.x = 0.5;
        velocity_msg.angular.z = -0.5;
    } else {
	//ROS_INFO_STREAM("no input");
        velocity_msg.linear.x = 0;
        velocity_msg.angular.z = 0;
    }
    pub_velocity.publish(velocity_msg) ;

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "node1") ;

    ros::NodeHandle node_handle;

    ros::Rate loopRate(100);    
    
    pub_velocity = node_handle.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel",1);

    ros::Subscriber subscriber = node_handle.subscribe("teleop/cmd", 1, keyboard_input_callback) ;


    while ( ros::ok() ) {

        ros::spinOnce() ;

        loopRate.sleep() ;
    }

    return 0 ;
}



