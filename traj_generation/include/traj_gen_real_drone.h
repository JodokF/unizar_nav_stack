#include <ros/ros.h>
#include <stdio.h> 
#include <iostream>
#include <sstream>
#include <fstream>

#include <chrono>
#include <cmath>
#include <queue>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <hector_uav_msgs/EnableMotors.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_planning_msgs/PlannerService.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/State.h>
#include <iomanip> 

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class poly_traj_plan{
    private:

        bool waypoints_received, odom_received, goal_recieved, planner_service_called;
        int nmbr_of_states, curr_state, waypoint_cntr, marker_cntr;
        
        ros::Subscriber plan_sub, pose_sub, goal_sub;
        ros::Publisher vel_pub, mav_traj_markers_pub, marker_pub, pose_pub, vel_pub_2_real_drone;
        std::string odom_topic, default_odom_topic, goal_topic, default_goal_topic, planner_service;

        mav_trajectory_generation::Trajectory trajectory;
        mav_trajectory_generation::Vertex::Vector vertices;
        mav_planning_msgs::PlannerService request;

        geometry_msgs::PoseArray vxblx_waypoints;
        // std::vector<geometry_msgs::Pose> trajectory;
        geometry_msgs::PoseStamped odom_info;
        geometry_msgs::PoseStamped goal;
        geometry_msgs::Twist vel_msg;
        hector_uav_msgs::EnableMotors motor_enable_service_msg;
        std_srvs::Empty path_plan_req;
        ros::ServiceClient motor_enable_service; 
        ros::ServiceClient path_plan_client;


        // functions:
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void planCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void drawMAVTrajectoryMarkers();
        void drawMarkerArray(geometry_msgs::PoseArray waypoints, int color, int offset);
        double goalDistance(geometry_msgs::Pose pose, geometry_msgs::Point goal);
        double get_yaw_from_quat(const geometry_msgs::Quaternion );
        geometry_msgs::Pose calculateMidpoint(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2);

        

    public:

        poly_traj_plan(ros::NodeHandle& nh);
        mav_msgs::EigenTrajectoryPoint::Vector traj_states;
        double takeoff_altitude, sampling_interval, vel_threshold;
     //~poly_traj_plan();
        int takeoff();
        bool generate_trajectory();
        int run_navigation_node();
        void send_vel_commands();


}; 

class real_drone_connection{
    private:
        ros::Subscriber current_pose_sub, planned_vel_sub, state_sub;
        ros::Publisher real_vel_pub;
        ros::ServiceClient arming_client, set_cmd_vel_frame, set_mode_client;

        geometry_msgs::Twist vel_msg;

        
        void state_cb(const mavros_msgs::State::ConstPtr& msg);

        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
        


    public:

    real_drone_connection(ros::NodeHandle& nh);
    int establish_drone_connection();
    //bool send_vel_commands(const mav_msgs::EigenTrajectoryPoint&, double);
    bool send_vel_commands();
    geometry_msgs::PoseStamped current_pose;
    mavros_msgs::State current_state;
    geometry_msgs::Twist planned_vel;
    geometry_msgs::Twist cmd_vel;




};


