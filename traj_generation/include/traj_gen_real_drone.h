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

// #include <hector_uav_msgs/EnableMotors.h>

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

// for the camera tf frame transformation:
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class poly_traj_plan{
    private:

        bool waypoints_received, odom_received, goal_recieved, planner_service_called, tracking_camera;
        int nmbr_of_states, curr_state, waypoint_cntr, marker_cntr;
        
        ros::Subscriber plan_sub, pose_sub_geomtry_msg_pose, pose_sub_nav_msg_odom, goal_sub;
        ros::Publisher vel_pub, mav_traj_markers_pub, marker_pub, pose_pub;
        std::string pose_nav_msg_odom_msg_topic, default_odom_topic, goal_topic, default_goal_topic, planner_service;

        mav_trajectory_generation::Trajectory trajectory;
        mav_trajectory_generation::Vertex::Vector vertices;
        mav_planning_msgs::PlannerService request;

        geometry_msgs::PoseArray vxblx_waypoints;
        // std::vector<geometry_msgs::Pose> trajectory;
        nav_msgs::Odometry odom_info;
        geometry_msgs::PoseStamped odom_info_geo_msg;
        geometry_msgs::PoseStamped goal;
        geometry_msgs::PoseStamped cmd_pose;
        geometry_msgs::Twist cmd_vel;
        
        std_srvs::Empty path_plan_req;
        ros::ServiceClient motor_enable_service; 
        ros::ServiceClient path_plan_client;

        
        // for the camera tf frame transformation:
        geometry_msgs::TransformStamped tf_odom_to_camera;
        geometry_msgs::PoseStamped temp_pose;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener; 
        std::string target_frame, source_frame;


        double altitude_factor, x_offset;


        // functions:
        void poseCallback_nav_msg_odom(const nav_msgs::Odometry::ConstPtr &msg);
        void poseCallback_geomtry_msg_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void planCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void drawMAVTrajectoryMarkers();
        void drawMarkerArray(std::vector<nav_msgs::Odometry> waypoints, int color, int offset);
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



