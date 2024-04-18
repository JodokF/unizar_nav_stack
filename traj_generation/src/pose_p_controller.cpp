// proportional controller lololol

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// Global variables
geometry_msgs::Point position_setpoint;
geometry_msgs::Point current_position;

// Callback for position setpoint
void positionSetpointCallback(const geometry_msgs::Point::ConstPtr& msg) {
    position_setpoint = *msg;
}

// Callback for position feedback
void positionFeedbackCallback(const geometry_msgs::Point::ConstPtr& msg) {
    current_position = *msg;
}

// Proportional controller function
void proportionalController() {
    // Calculate position error
    geometry_msgs::Point position_error;
    position_error.x = position_setpoint.x - current_position.x;
    position_error.y = position_setpoint.y - current_position.y;
    position_error.z = position_setpoint.z - current_position.z;

    // Proportional gain
    double Kp = 0.1; // Adjust as needed

    // Compute velocity command (proportional control)
    geometry_msgs::Point velocity_command;
    velocity_command.x = Kp * position_error.x;
    velocity_command.y = Kp * position_error.y;
    velocity_command.z = Kp * position_error.z;

    // Publish velocity command
    // (Publish the velocity command to control the drone's motion)
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "proportional_controller_node");
    ros::NodeHandle nh;

    // Subscribe to position setpoint topic
    ros::Subscriber position_setpoint_sub = nh.subscribe("position_setpoint_topic", 1, positionSetpointCallback);

    // Subscribe to position feedback topic
    ros::Subscriber position_feedback_sub = nh.subscribe("position_feedback_topic", 1, positionFeedbackCallback);

    // Set up publisher for velocity command
    // ros::Publisher velocity_command_pub = nh.advertise<geometry_msgs::Point>("velocity_command_topic", 1);

    // Rate for the control loop
    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        // Call proportional controller function
        proportionalController();

        // Publish any velocity command if needed
        // velocity_command_pub.publish(velocity_command);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
