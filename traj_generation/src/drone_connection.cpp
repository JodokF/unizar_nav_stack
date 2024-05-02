#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <iomanip> // for std::fixed and std::setprecision
#include <tf/tf.h> // Include the necessary header
#include <tf/transform_datatypes.h>


class drone_connection
{
    private:
        void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
        void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg);
        void pose_cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        double calc_euc_dist(geometry_msgs::Pose pose, geometry_msgs::Pose goal);
        
        ros::Subscriber state_sub;
        ros::Subscriber vel_cmd_sub;
        ros::Subscriber pose_cmd_sub;
        ros::Publisher pose_pub;
        ros::Publisher error_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Publisher cmd_vel_unstmpd_pub;
        ros::Subscriber pose_sub;
        ros::ServiceClient set_cmd_vel_frame;
        
        geometry_msgs::PoseStamped start_pose;

        mavros_msgs::SetMavFrame set_frame_msg;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;

        mavros_msgs::State current_mav_state;
        nav_msgs::Odometry curr_pose;
        geometry_msgs::Twist vel_cmd_in, vel_cmd_send;
        geometry_msgs::PoseStamped pose_cmd_in;
        geometry_msgs::Pose error_pose;
        ros::Time ros_time_last, ros_time_now;
        ros::Duration passed_time;

        double k_x, k_y, k_z;

        double time_last;
        double vel_calc_x, vel_calc_y, vel_calc_z;


    public:
        bool vel_cmd_received, pose_cmd_received;
        int take_off();
        drone_connection(ros::NodeHandle& nh);
        ~drone_connection();
};

drone_connection::drone_connection(ros::NodeHandle& nh)
{

    /* --- Getting CMDS --- */
    vel_cmd_sub = nh.subscribe<geometry_msgs::Twist>
        ("/vel_cmd_2_drone", 10, &drone_connection::vel_cmd_cb, this);
    pose_cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/pose_cmd_2_drone", 10, &drone_connection::pose_cmd_cb, this);
    pose_sub = nh.subscribe<nav_msgs::Odometry>
        ("/mavros/odometry/out", 10, &drone_connection::pose_cb,this);        
    
    /* --- Sending CMDS to Drone --- */
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);
    cmd_vel_unstmpd_pub = nh.advertise<geometry_msgs::Twist>
        ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    /* --- MAVROS STUFF --- */
    state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, &drone_connection::mavros_state_cb, this);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("/mavros/set_mode");
    set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
        ("/mavros/setpoint_velocity/mav_frame");
    
    /* --- Other STUFF --- */
    error_pub = nh.advertise<geometry_msgs::Pose>
        ("/error_setpoint_real", 10);

    start_pose.pose.position.x = 0; // -1
    start_pose.pose.position.y = 0; // 2
    start_pose.pose.position.z = 2.1; // 1.2
    start_pose.pose.orientation.x = 0;
    start_pose.pose.orientation.y = 0;
    start_pose.pose.orientation.z = 0.383; // = 45 degree in z
    start_pose.pose.orientation.w = 0.924; // = 45 degree in z
   
    vel_cmd_received = false;
    pose_cmd_received = false;
    vel_calc_x = 0;
    vel_calc_y = 0;
    vel_calc_z = 0;
    k_x = 0.08;
    k_y = 0.3;
    k_z = 0.3;
    

    ros_time_last = ros::Time::now(); 

}

drone_connection::~drone_connection()
{
}

void drone_connection::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_mav_state = *msg;
}

void drone_connection::pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    curr_pose = *msg;

    if (pose_cmd_received == true)
    {
        passed_time = ros::Time::now() - ros_time_last; // = 0.052 sec = aprx. 20 hz

        vel_calc_x = k_x * (error_pose.position.x / passed_time.toSec());
        vel_calc_y = k_y * (error_pose.position.y / passed_time.toSec());
        vel_calc_z = k_z * (error_pose.position.z / passed_time.toSec());


        std::cout << "calculated vel.: "<< vel_calc_x << std::endl;
        std::cout << "real vel.:       "<< vel_cmd_in.linear.z << "\n---\n";
        

        vel_cmd_send.linear.x = vel_cmd_in.linear.x; // vel_calc_x;
        vel_cmd_send.linear.y = vel_cmd_in.linear.y; // vel_calc_y;
        vel_cmd_send.linear.z = vel_calc_z; // vel_cmd_in.linear.z; // vel_calc_z;
        vel_cmd_send.angular.x = vel_cmd_in.angular.x;
        vel_cmd_send.angular.y = vel_cmd_in.angular.y;
        vel_cmd_send.angular.z = vel_cmd_in.angular.z;
        
        
        ros_time_last = ros::Time::now();

        cmd_vel_unstmpd_pub.publish(vel_cmd_send);
    }

}

void drone_connection::pose_cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_cmd_in = *msg;
    pose_cmd_received = true;
    
    std::cout << std::fixed << std::showpoint;
    std::cout << std::setprecision(3);

    error_pose.position.x = pose_cmd_in.pose.position.x - curr_pose.pose.pose.position.x;
    error_pose.position.y = pose_cmd_in.pose.position.y - curr_pose.pose.pose.position.y;
    error_pose.position.z = pose_cmd_in.pose.position.z - curr_pose.pose.pose.position.z;
    error_pose.orientation.x = pose_cmd_in.pose.orientation.x - curr_pose.pose.pose.orientation.x;
    error_pose.orientation.y = pose_cmd_in.pose.orientation.y - curr_pose.pose.pose.orientation.y;
    error_pose.orientation.z = pose_cmd_in.pose.orientation.z - curr_pose.pose.pose.orientation.z;
    error_pose.orientation.w = pose_cmd_in.pose.orientation.w - curr_pose.pose.pose.orientation.w; 

    error_pub.publish(error_pose);


    std::cout << "Error x, y, z: "  << error_pose.position.x << ", " 
                                    << error_pose.position.y << ", "
                                    << error_pose.position.z << "\n";

    //pose_cmd.header.frame_id.at(5);
    //std::cout << pose_cmd.header;
    
    //pose_pub.publish(*msg);
    
}

void drone_connection::vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg){
    vel_cmd_in = *msg;
    vel_cmd_received = true;
    
}

double drone_connection::calc_euc_dist(geometry_msgs::Pose pose, geometry_msgs::Pose goal){
    double x_diff = goal.position.x - pose.position.x, y_diff = goal.position.y - pose.position.y, z_diff = goal.position.z - pose.position.z;
    double euc_dis = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc_dis;
}

int drone_connection::take_off()
{
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate50(20.0);
    ros_time_last = ros::Time::now();

    // wait for FCU connection
    while(ros::ok() && !current_mav_state.connected){

        ros::spinOnce();
        rate50.sleep();
    }

    // Set the frame in which the velocties are  interpreted by the drone
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_LOCAL_NED;  // FRAME_LOCAL_NED = world, FRAME_BODY_NED = drone
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    std::cout << "\nMavframe set: "<< set_cmd_vel_frame.call(set_frame_msg) << std::endl;

    
    //send a few setpoints before starting to establish the offboard connection
    for(int i = 50; ros::ok() && i > 0; --i){
        pose_pub.publish(start_pose);
        ros::spinOnce();
        rate50.sleep();
    }

    ros::Time last_request = ros::Time::now();

    bool start_hight_reached = false;
    bool take_off_detected = false;
    while(ros::ok()){
        if( current_mav_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {    
            if( !current_mav_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(curr_pose.pose.pose.position.z >= start_pose.pose.position.z - 0.11 && start_hight_reached == false) {
            std::cout << "Start hight reached.\n";
            start_hight_reached = true;
        }
        if(curr_pose.pose.pose.position.z >= 0.06 && take_off_detected == false) {
            std::cout << "Time for Takeoff: "<< ros::Time::now() - ros_time_last << " sec. \n";
            take_off_detected = true;
        }
        


        pose_pub.publish(start_pose);

        if(vel_cmd_sub.getNumPublishers() > 0 && vel_cmd_received == true){
            ROS_INFO("Traj. vel. publisher detected.");
            return 0;
        }
        
        ros::spinOnce();
        rate50.sleep();
    }
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_gen");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner ich_spinne(1);
    ich_spinne.start();

    drone_connection drone(std::ref(node_handle));

    drone.take_off();

    ros::Rate rate20(20);

    ROS_INFO("Publishing velocitiess now...");
    while(ros::ok())
    {                               
        //ros::spinOnce();
        rate20.sleep();

    }

    return 0;
}