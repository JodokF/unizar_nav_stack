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
#include "std_msgs/Float64.h"


class drone_connection{

    private:
        void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
        void sim_pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void real_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg);
        void pose_cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        double calc_euc_dist(geometry_msgs::Pose pose, geometry_msgs::Pose goal);
        double get_yaw_difference(double from, double to);

        
        ros::Subscriber state_sub;
        ros::Subscriber vel_cmd_sub;
        ros::Subscriber pose_cmd_sub;
        ros::Subscriber pose_sub_sim;
        ros::Subscriber pose_sub_real;
        ros::Publisher pose_pub;
        ros::Publisher error_pub;
        ros::Publisher cmd_vel_unstmpd_pub;

    /**************** PID Stuff: ****************/
        
        ros::Publisher x_pid_setp_pub, x_pid_state_pub;
        ros::Publisher y_pid_setp_pub, y_pid_state_pub;        
        ros::Publisher z_pid_setp_pub, z_pid_state_pub; 
        ros::Publisher yaw_pid_setp_pub, yaw_pid_state_pub; 

        ros::Subscriber x_pid_out_sub, y_pid_out_sub, z_pid_out_sub, yaw_pid_out_sub;

        void x_pid_cb(const std_msgs::Float64::ConstPtr& msg);
        void y_pid_cb(const std_msgs::Float64::ConstPtr& msg);
        void z_pid_cb(const std_msgs::Float64::ConstPtr& msg);
        void yaw_pid_cb(const std_msgs::Float64::ConstPtr& msg);

        std_msgs::Float64 x_setp, x_state;
        std_msgs::Float64 y_setp, y_state;
        std_msgs::Float64 z_setp, z_state;
        std_msgs::Float64 yaw_setp, yaw_state;

        double x_pid_out, y_pid_out, z_pid_out, yaw_pid_out;

    /**************** PID Stuff: END ****************/


        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        
        ros::ServiceClient set_cmd_vel_frame;
        
        mavros_msgs::SetMavFrame set_frame_msg;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;

        mavros_msgs::State current_mav_state;

        nav_msgs::Odometry curr_pose_sim;
        geometry_msgs::PoseStamped curr_pose_real;
        geometry_msgs::PoseStamped curr_pose;

        geometry_msgs::Twist vel_cmd_in, vel_cmd_send;
        geometry_msgs::PoseStamped pose_cmd_in;
        geometry_msgs::Pose error_pose, takeoff;
        ros::Time ros_time_last, ros_time_now;
        ros::Duration passed_time;
        
        double time_last, vel_threshold_lin, vel_threshold_ang; 
        int sim_or_real; // 0 = default, 1 = simulation, 2 = real drone
        double vel_calc_x, vel_calc_y, vel_calc_z;
        bool vel_anomalie_detected;
        int pid_debug_sync_cntr;


    public:
        geometry_msgs::PoseStamped start_pose;
        bool vel_cmd_received, pose_cmd_received, new_pose_received, use_cntrl;
        void calc_cntrl_vel();
        void pub_to_ros_pid();
        void calc_error();
        void send_vel_cmds_to_drone();
        int establish_connection_and_take_off();
        drone_connection(ros::NodeHandle& nh);
        ~drone_connection();
};

drone_connection::drone_connection(ros::NodeHandle& nh)
{

    /* --- Parameter from launch file---*/
    //   declare the variables to read from param
    //   Read the take-off pose
    nh.getParam("/drone_connection_node/x_takeoff", takeoff.position.x);
    nh.getParam("/drone_connection_node/y_takeoff", takeoff.position.y);
    nh.getParam("/drone_connection_node/z_takeoff", takeoff.position.z);
    nh.getParam("/drone_connection_node/R_takeoff", takeoff.orientation.x); // saved as RPY not Quaternion
    nh.getParam("/drone_connection_node/P_takeoff", takeoff.orientation.y); // saved as RPY not Quaternion
    nh.getParam("/drone_connection_node/Y_takeoff", takeoff.orientation.z); // saved as RPY not Quaternion
    nh.getParam("/drone_connection_node/use_cntrl", use_cntrl); // determine if controller should be used or not

    /* --- Getting CMDS --- */
    vel_cmd_sub = nh.subscribe<geometry_msgs::Twist>
        ("/vel_cmd_2_drone", 10, &drone_connection::vel_cmd_cb, this);
    pose_cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/pose_cmd_2_drone", 10, &drone_connection::pose_cmd_cb, this); // 19.2 Hz
    pose_sub_sim = nh.subscribe<nav_msgs::Odometry>
        ("/mavros/odometry/out", 10, &drone_connection::sim_pose_cb, this);  // 19.2 Hz
    pose_sub_real = nh.subscribe<geometry_msgs::PoseStamped>
        ("/optitrack/pose", 10, &drone_connection::real_pose_cb, this);  
    
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
        ("/error_setpoint_and_real", 10);
    
    /* --- ROS PID Controller --- */        

    x_pid_setp_pub = nh.advertise<std_msgs::Float64>
        ("/x_pos/setpoint", 10);
    x_pid_state_pub = nh.advertise<std_msgs::Float64>
        ("/x_pos/state", 10);
    x_pid_out_sub = nh.subscribe<std_msgs::Float64>
        ("/x_pos/control_effort", 10, &drone_connection::x_pid_cb, this);

    y_pid_setp_pub = nh.advertise<std_msgs::Float64>
        ("/y_pos/setpoint", 10);
    y_pid_state_pub = nh.advertise<std_msgs::Float64>
        ("/y_pos/state", 10);
    y_pid_out_sub = nh.subscribe<std_msgs::Float64>
        ("/y_pos/control_effort", 10, &drone_connection::y_pid_cb, this);

    z_pid_setp_pub = nh.advertise<std_msgs::Float64>
        ("/z_pos/setpoint", 10);
    z_pid_state_pub = nh.advertise<std_msgs::Float64>
        ("/z_pos/state", 10);
    z_pid_out_sub = nh.subscribe<std_msgs::Float64>
        ("/z_pos/control_effort", 10, &drone_connection::z_pid_cb, this);

    yaw_pid_setp_pub = nh.advertise<std_msgs::Float64>
        ("/yaw/setpoint", 10);
    yaw_pid_state_pub = nh.advertise<std_msgs::Float64>
        ("/yaw/state", 10);
    yaw_pid_out_sub = nh.subscribe<std_msgs::Float64>
        ("/yaw/control_effort", 10, &drone_connection::yaw_pid_cb, this);

    pid_debug_sync_cntr = 0;

    /* --- ROS PID Controller --- */  
     

    start_pose.pose.position.x = takeoff.position.x; // -1
    start_pose.pose.position.y = takeoff.position.y; // 2
    start_pose.pose.position.z = takeoff.position.z; // 1.2

    tf2::Quaternion quaternion;
    quaternion.setRPY(takeoff.orientation.x, takeoff.orientation.y, takeoff.orientation.z);

    start_pose.pose.orientation.x = quaternion.getX();
    start_pose.pose.orientation.y = quaternion.getY();
    start_pose.pose.orientation.z = quaternion.getZ();
    start_pose.pose.orientation.w = quaternion.getW(); 
   
    vel_cmd_received = false;
    pose_cmd_received = false;
    vel_anomalie_detected = false;
    new_pose_received = false;
    // vel_calc_x = 0;
    // vel_calc_y = 0;
    // vel_calc_z = 0;

    vel_threshold_lin = 0.45;       // 0.4 m/s
    vel_threshold_ang = M_PI / 3 ; //= 1.047 rad per second = 60 degree / s
    sim_or_real = 0;

    ros_time_last = ros::Time::now(); 

}

drone_connection::~drone_connection()
{
    // Todo
}


/////////////////////////////// CALLBACKS ///////////////////////////////
// TODO: make them more slick and sick

void drone_connection::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_mav_state = *msg;
}

void drone_connection::sim_pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    curr_pose_sim = *msg;
    
    // conversion to geometry_msgs:
    curr_pose.pose = curr_pose_sim.pose.pose;
    curr_pose.header = curr_pose_sim.header;

    new_pose_received = true;
}

void drone_connection::real_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pose_real = *msg;
    curr_pose = curr_pose_real;
    new_pose_received = true;

}

void drone_connection::pose_cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_cmd_in = *msg;
    pose_cmd_received = true;
        
}

void drone_connection::vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg){
    vel_cmd_in = *msg;
    vel_cmd_send = vel_cmd_in;  // idk probably 'vel_cmd_in' could also be used without copying it to vel_cmd_send
                                // but since it gets changed in the controller the original velocity command is preserved
    vel_cmd_received = true;

}

void drone_connection::x_pid_cb(const std_msgs::Float64::ConstPtr& msg){
    std::cout << pid_debug_sync_cntr << " - B\n";
    x_pid_out = msg->data;
} 

void drone_connection::y_pid_cb(const std_msgs::Float64::ConstPtr& msg){
    y_pid_out = msg->data;
}

void drone_connection::z_pid_cb(const std_msgs::Float64::ConstPtr& msg){
    z_pid_out = msg->data;
}

void drone_connection::yaw_pid_cb(const std_msgs::Float64::ConstPtr& msg){
    yaw_pid_out = msg->data;
}


/////////////////////////////// CALLBACKS END ///////////////////////////////

double drone_connection::get_yaw_difference(double from, double to)
{
    double difference = to - from; 
    while (difference < -M_PI) difference += M_PI;
    while (difference >  M_PI) difference -= M_PI;
    return difference;
}

void drone_connection::pub_to_ros_pid(){
    
    std::cout << pid_debug_sync_cntr << " - A\n";

    x_setp.data = pose_cmd_in.pose.position.x; // to transforme it to a std_msgs::Float64
    x_state.data = curr_pose.pose.position.x;
    x_pid_setp_pub.publish(x_setp);
    x_pid_state_pub.publish(x_state);

    y_setp.data = pose_cmd_in.pose.position.y; // to transforme it to a std_msgs::Float64
    y_state.data = curr_pose.pose.position.y;
    y_pid_setp_pub.publish(y_setp);
    y_pid_state_pub.publish(y_state);

    z_setp.data = pose_cmd_in.pose.position.z; // to transforme it to a std_msgs::Float64
    z_state.data = curr_pose.pose.position.z;
    z_pid_setp_pub.publish(z_setp);
    z_pid_state_pub.publish(z_state);

    // yaw stuff:
    //double from = tf::getYaw(curr_pose.pose.orientation);
    //double to = tf::getYaw(pose_cmd_in.pose.orientation);
    yaw_setp.data = 0;
    yaw_state.data = get_yaw_difference(tf::getYaw(curr_pose.pose.orientation), tf::getYaw(pose_cmd_in.pose.orientation));
    yaw_pid_setp_pub.publish(yaw_setp);
    yaw_pid_state_pub.publish(yaw_state);
    
    
}


void drone_connection::calc_cntrl_vel(){

        
        passed_time = ros::Time::now() - ros_time_last; // = 0.052 sec = aprx. 20 hz
        ros_time_last = ros::Time::now();

        // calc k controller values:
        std::cout << pid_debug_sync_cntr << " - C\n";
        pid_debug_sync_cntr++;

        // vel_calc_x = x_pid_out / passed_time.toSec();
        // vel_calc_y = y_pid_out / passed_time.toSec();
        // vel_calc_z = z_pid_out / passed_time.toSec();

        //std::cout << "calculated vel.: "<< vel_calc_x << std::endl;
        //std::cout << "real vel.:       "<< vel_cmd_in.linear.z << "\n---\n";
        
        vel_cmd_send.linear.x = x_pid_out / passed_time.toSec(); // vel_calc_x; 
        vel_cmd_send.linear.y = y_pid_out / passed_time.toSec(); // vel_calc_y;
        vel_cmd_send.linear.z = z_pid_out / passed_time.toSec(); // vel_calc_z; 
        vel_cmd_send.angular.z = yaw_pid_out / passed_time.toSec();
        
}

 void drone_connection::calc_error() {

    error_pose.position.x = pose_cmd_in.pose.position.x - curr_pose.pose.position.x;
    error_pose.position.y = pose_cmd_in.pose.position.y - curr_pose.pose.position.y;
    error_pose.position.z = pose_cmd_in.pose.position.z - curr_pose.pose.position.z;
    /*
    error_pose.orientation.x = pose_cmd_in.pose.orientation.x - curr_pose.pose.orientation.x;
    error_pose.orientation.y = pose_cmd_in.pose.orientation.y - curr_pose.pose.orientation.y;
    error_pose.orientation.z = pose_cmd_in.pose.orientation.z - curr_pose.pose.orientation.z;
    error_pose.orientation.w = pose_cmd_in.pose.orientation.w - curr_pose.pose.orientation.w; 
    */
    
    
    // to record rosbag from error
    error_pub.publish(error_pose);

    /*
    std::cout << std::fixed << std::showpoint << std::setprecision(3);
    std::cout << "Error x, y, z: "  << error_pose.position.x << ", " 
                                     << error_pose.position.y << ", "
                                     << error_pose.position.z << "\n";
    */

 }
 
 void drone_connection::send_vel_cmds_to_drone(){

  
    std::cout << std::fixed << std::showpoint << std::setprecision(3);

        std::cout << "---\nVelocities:\nCalc::"   << vel_cmd_send.linear.x << " x; " 
                                                << vel_cmd_send.linear.y << " y; " 
                                                << vel_cmd_send.linear.z << " z;\n";//\nAng: ";
                                                //<< vel_cmd_send.angular.x << " roll; "
                                                //<< vel_cmd_send.angular.y << " pitch; "
                                                //<< vel_cmd_send.angular.z << " yaw;\n";
        std::cout << "Org: "  << vel_cmd_in.linear.x << " x; " 
                                            << vel_cmd_in.linear.y << " y; " 
                                            << vel_cmd_in.linear.z << " z;\n---\n";//\nAng: ";


    

    // if the calculatet velocity is under the vel_threshold -> publish it
    // if not -> change all velocities to zero and publish it
    if (vel_cmd_send.linear.x >  vel_threshold_lin ||
        vel_cmd_send.linear.x < -vel_threshold_lin ||
        vel_cmd_send.linear.y >  vel_threshold_lin ||
        vel_cmd_send.linear.y < -vel_threshold_lin ||
        vel_cmd_send.linear.z >  vel_threshold_lin ||
        vel_cmd_send.linear.z < -vel_threshold_lin ||
        vel_cmd_send.angular.x >  vel_threshold_ang ||
        vel_cmd_send.angular.x < -vel_threshold_ang ||
        vel_cmd_send.angular.y >  vel_threshold_ang || // actually this should be always zero, right?
        vel_cmd_send.angular.y < -vel_threshold_ang || // actually this should be always zero, right?
        vel_cmd_send.angular.z >  vel_threshold_ang || // actually this should be always zero, right?
        vel_cmd_send.angular.z < -vel_threshold_ang || // actually this should be always zero, right?
        vel_anomalie_detected == true)
    {
            ROS_ERROR("Velocitie anomalie detected! Sending Velocity ZERO!\n");
            vel_anomalie_detected = true;
            vel_cmd_send.linear.x = 0.0;
            vel_cmd_send.linear.y = 0.0;
            vel_cmd_send.linear.z = 0.0;
            vel_cmd_send.angular.x = 0.0;
            vel_cmd_send.angular.y = 0.0;
            vel_cmd_send.angular.z = 0.0;
    }
    
    cmd_vel_unstmpd_pub.publish(vel_cmd_send);

}

double drone_connection::calc_euc_dist(geometry_msgs::Pose pose, geometry_msgs::Pose goal){
    double x_diff = goal.position.x - pose.position.x, y_diff = goal.position.y - pose.position.y, z_diff = goal.position.z - pose.position.z;
    double euc_dis = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
    return euc_dis;
}

int drone_connection::establish_connection_and_take_off()
{
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate20(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_mav_state.connected){

        ros::spinOnce();
        rate20.sleep();
    }

    // Set the frame in which the velocties are  interpreted by the drone
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_LOCAL_NED;  // FRAME_LOCAL_NED = world, FRAME_BODY_NED = drone
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    std::cout << "\nMavframe set: ";
    if(set_cmd_vel_frame.call(set_frame_msg) == 1) std::cout << " true" << std::endl;
    else std::cout << " false" << std::endl;

    
    //send a few setpoints before starting to establish the offboard connection
    for(int i = 50; ros::ok() && i > 0; --i){
        pose_pub.publish(start_pose);
        ros::spinOnce();
        rate20.sleep();
    }

    ros::Time last_request = ros::Time::now();

    bool start_hight_reached = false;
    bool offb_entered = false;  
    bool armed_once = false;

    while(ros::ok()){
        if(  current_mav_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)) &&
             offb_entered == false){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                offb_entered = true; // to ensure that the drone does not enter in the offb mode again after taking the commando with the remote control
            }
            last_request = ros::Time::now();
        } else {    
            if( !current_mav_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0)) &&
                armed_once == false ){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    armed_once = true;
                }
                last_request = ros::Time::now();
            }
        }

        if(curr_pose.pose.position.z >= start_pose.pose.position.z - 0.1 && start_hight_reached == false) {
            ROS_INFO("Start hight reached.");
            start_hight_reached = true;
        }
        
        pose_pub.publish(start_pose);

        // safety first
        if(vel_cmd_sub.getNumPublishers() > 0 && vel_cmd_received == true){
            ROS_INFO("Trajectory velocity publisher detected.");
            return 0;
        }
        if(pose_cmd_sub.getNumPublishers() > 0 && pose_cmd_received == true){
            ROS_INFO("Trajectory pose publisher detected.");
            return 0;
        }
        
        ros::spinOnce();
        rate20.sleep();
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
    std::cout << "Start pose: \n" << drone.start_pose.pose;
    std::cout << "Using controller: " << drone.use_cntrl;

    
    // i think i can delete this check again bc. I wont need it...
    /*
    try {drone.get_sim_or_real();}
    catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    */

    drone.establish_connection_and_take_off();

    ros::Rate rate40(40);

    ROS_INFO("Publishing velocitiess now...");
    // check if commands are recived for flying the trajectory
    if (drone.pose_cmd_received == true || drone.vel_cmd_received == true){
        if(drone.use_cntrl == true){ // check if controller should be used
            while(ros::ok()){     
                    
                drone.pub_to_ros_pid();
                drone.calc_cntrl_vel();
                drone.send_vel_cmds_to_drone();

                //ros::spinOnce();
                rate40.sleep();

            }
        }
        else if(drone.use_cntrl == false){
            while(ros::ok()){  
                drone.send_vel_cmds_to_drone();
                rate40.sleep();
            }
        }

    }

    return 0;
}