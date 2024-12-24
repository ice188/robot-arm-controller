#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>
#include <math.h>

double current_x;
double current_y;
Eigen::Vector2f predicted_velocity;
double dt = 1.0/500;
bool params_read = false;

bool start_planning = false;
ros::Publisher pose_publisher;
ros::Publisher twist_publisher;
geometry_msgs::Pose new_pose;
geometry_msgs::Twist new_twist;
ros::Publisher done_publisher;

class PFPlanner {
    public:
        PFPlanner(ros::NodeHandle &nodeHandle);
        double target_x_;
        double target_y_;
        double k_att_;
        double max_linear_velocity_;
        double max_angular_velocity_;

        void readParams();

    private:
        ros::NodeHandle &node_handle_ ;
        ros::Subscriber pose_subscriber_;
        ros::ServiceServer start_service_;

        
};

//*************** part 1 **********************
void PFPlanner::readParams() {
    if (params_read) return;
    node_handle_.getParam("/target/x",target_x_);
    node_handle_.getParam("/target/y",target_y_);
    node_handle_.getParam("/planner/k_att",k_att_);
    node_handle_.getParam("/husky_velocity_controller/linear/x/max_velocity",max_linear_velocity_);
    node_handle_.getParam("/husky_velocity_controller/angular/z/max_velocity",max_angular_velocity_);

    params_read = true;
}

//*************** part 2 **********************
void get_xy_velocity(PFPlanner &planner) {
    Eigen::Vector2f x_c(current_x, current_y);
    Eigen::Vector2f x_f(planner.target_x_, planner.target_y_);

    Eigen::Vector2f F_att = planner.k_att_ * (x_f - x_c);
    predicted_velocity = F_att;
    double norm = predicted_velocity.norm();
    
    if (norm > planner.max_linear_velocity_) {    
        predicted_velocity /= norm;
        predicted_velocity *= planner.max_linear_velocity_;   
    }
    
}

void get_pose (const geometry_msgs::Pose &current_pose){
    current_x = current_pose.position.x;
    current_y = current_pose.position.y;
}


//*************** part 3 **********************
bool publish_pose_twist ( std_srvs::Trigger::Request  &req,
                          std_srvs::Trigger::Response &res) {

    res.success = true;
    res.message = "Start publishing desired twist and pose";
    start_planning = true;
    return res.success;
}

//*************** part 3 END ******************


PFPlanner::PFPlanner(ros::NodeHandle &nodeHandle) : node_handle_(nodeHandle) {
    
    pose_subscriber_ = node_handle_.subscribe("/husky_velocity_controller/feedback/pose", 1, get_pose) ;
    start_service_ = node_handle_.advertiseService("/planner/start", publish_pose_twist);
    pose_publisher = node_handle_.advertise<geometry_msgs::Pose>("/planner/pose", 1) ;
    twist_publisher = node_handle_.advertise<geometry_msgs::Twist>("/planner/twist", 1) ;
    done_publisher = node_handle_.advertise<std_msgs::Bool>("/planner/done", 1) ;
    
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "planner");

    ros::NodeHandle nodeHandle;

    ros::Rate loopRate(500);

    PFPlanner planner(nodeHandle);

    std_msgs::Bool target_reached;
    target_reached.data = false;

    while(ros::ok()){

        //get current position
        ros::spinOnce();
        if (!start_planning) continue; //wait till service starts
        
        planner.readParams();
        
        get_xy_velocity(planner);
        new_twist.linear.x = predicted_velocity(0);
        new_twist.linear.y = predicted_velocity(1);
        
        new_pose.position.x = current_x + predicted_velocity(0) * dt;
        new_pose.position.y = current_y + predicted_velocity(1) * dt;

        pose_publisher.publish(new_pose);
        twist_publisher.publish(new_twist);


        double distance = sqrt(pow(current_x - planner.target_x_, 2) + pow(current_y - planner.target_y_, 2));
        if (distance < 0.1) {
            target_reached.data = true;
        } else {
            target_reached.data = false;
        }
        
        ROS_INFO_STREAM("distance: " << distance);

        done_publisher.publish(target_reached);

        loopRate.sleep();
    }

    return 0;
}


