#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool planner_done = false;
bool start_moving = false;

geometry_msgs::Pose current_pose;
geometry_msgs::Pose desired_pose;
geometry_msgs::Twist desired_twist;
geometry_msgs::Twist current_twist;

geometry_msgs::Twist heading_steering_speed;
double max_linear_velocity, max_angular_velocity;
double v, w;

void pose_callback(const geometry_msgs::Pose &pose_msg){
    desired_pose = pose_msg;
    
}
void twist_callback(const geometry_msgs::Twist &twist_msg){
    desired_twist = twist_msg;
    
    start_moving = true;
}
void done_callback(const std_msgs::Bool &done_msg){
    planner_done = done_msg.data;
}
void feedback_callback(const geometry_msgs::Pose &feedback_pose_msg){
    current_pose = feedback_pose_msg;
}
void current_twist_callback(const geometry_msgs::Twist &twist_msg){
    current_twist = twist_msg;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "controller");

    ros::NodeHandle node_handle;

    ros::Rate loopRate(500);

    ros::Subscriber pose_subscriber = node_handle.subscribe("/planner/pose", 1, pose_callback) ;
    ros::Subscriber twist_subscriber = node_handle.subscribe("/planner/twist", 1, twist_callback) ;
    ros::Subscriber done_subscriber = node_handle.subscribe("/planner/done", 1, done_callback) ;
    ros::Subscriber feedback_subscriber = node_handle.subscribe("/husky_velocity_controller/feedback/pose", 1, feedback_callback) ;
    ros::Subscriber current_twist_subscriber = node_handle.subscribe("/husky_velocity_controller/feedback/twist", 1, current_twist_callback);
    ros::Publisher cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
    
    node_handle.getParam("/husky_velocity_controller/linear/x/max_velocity",max_linear_velocity);
    node_handle.getParam("/husky_velocity_controller/angular/z/max_velocity",max_angular_velocity);

    tf2::Quaternion current_orientation;
    double x_r, y_r;
    Eigen::Matrix2f J;
    Eigen::Matrix2f R_sb;
    double roll, pitch, current_yaw;

    while(ros::ok()){

        ros::spinOnce(); 
        if (!start_moving) continue;

        if (!planner_done ){

            // tf2::fromMsg(current_pose.orientation, current_orientation);

            // tf2::Matrix3x3 m(current_orientation);

            // m.getRPY(roll, pitch, yaw);
            // double phi = atan2(desired_twist.linear.y, desired_twist.linear.x)-yaw;
            // v = sqrt(pow(desired_twist.linear.x, 2) + pow(desired_twist.linear.y, 2));
            
            // while (phi < -M_PI) {
            //     phi += 2 * M_PI;
            // } 
            // while (phi > M_PI) {
            //     phi -= 2 * M_PI;
            // }
            // w = phi;


            // if (v > max_linear_velocity) {
            //     v =  v/abs(v) * max_linear_velocity;
            // } 
            
            // if (w> max_angular_velocity){
            //     w = w/abs(w) * max_angular_velocity;
            // }


            // get heading angle in radians

            tf2::fromMsg(current_pose.orientation, current_orientation);
            tf2::Matrix3x3 m(current_orientation);
            m.getRPY(roll, pitch, current_yaw);

            //calculate difference in pose in local frame

            Eigen::Vector2f world_difference(desired_twist.linear.x, desired_twist.linear.y);

            R_sb << cos(current_yaw), -sin(current_yaw) , sin(current_yaw) , cos(current_yaw);
            auto local_difference = R_sb.inverse() * world_difference;
            x_r = local_difference.x();
            y_r = local_difference.y();

            // calculate heading and steering speed through inverse kinematics

            //double phi = atan2(desired_twist.linear.y, desired_twist.linear.x);
            J << cos(current_yaw), -sin(current_yaw) * x_r - cos(current_yaw) * y_r, 
                  sin(current_yaw), cos(current_yaw) * x_r - sin(current_yaw) * y_r;

            Eigen::Vector2f velocities(desired_twist.linear.x, desired_twist.linear.y);
            Eigen::Vector2f heading_steering = J.inverse() * velocities;

            v = heading_steering.x();
            w = heading_steering.y();

            while (w < -M_PI) {
                w += 2 * M_PI;
            }
            while (w > M_PI) {
                w -= 2 * M_PI;
            }
            std::cout << "x_r, y_r:"<< x_r << " ," << y_r << std::endl;
            std::cout << "J:"<< J << std::endl;
            std::cout << "J_inverse:"<< J.inverse() << std::endl;
            std::cout << "desired velocity:"<< velocities << std::endl;
            std::cout << "heading and steering: " << v << ", " << w << std::endl;
            
            if (abs(v) > max_linear_velocity) {
                v = v / abs(v) * max_linear_velocity;
            } 
            
            if (abs(w) > max_angular_velocity){
                w = w / abs(w) * max_angular_velocity;
            }

            heading_steering_speed.linear.x = v;
            heading_steering_speed.angular.z = w;

            cmd_vel_publisher.publish(heading_steering_speed);

            std::cout << "current pose: (" << current_pose.position.x << ", " << current_pose.position.y << ")" << std::endl;
            std::cout << "desired pose: (" << desired_pose.position.x << ", " <<  desired_pose.position.y << ")" << std::endl;
            std::cout << "current_yaw: " << current_yaw * 180 / M_PI << std::endl;
            std::cout << "desired angle: " << atan2(desired_twist.linear.y, desired_twist.linear.x) * 180 / M_PI << std::endl;
            std::cout << "normalized heading and steering: " << v << " " << w << std::endl;

        }else{
            heading_steering_speed.linear.x = 0;
            heading_steering_speed.angular.z = 0;
            cmd_vel_publisher.publish(heading_steering_speed);
        }
       // ROS_INFO_STREAM("heading: " << heading_steering_speed.linear.x << ", steering: "<< heading_steering_speed.angular.z);
        loopRate.sleep();
    }

    return 0;
}


