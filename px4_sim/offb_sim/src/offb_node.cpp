
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



// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>

/*
geometry_msgs::Quaternion RPY_to_quat(double roll, double pitch, double yaw) {
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}
*/
double roll, pitch, yaw;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

nav_msgs::Odometry curr_pose;
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    curr_pose = *msg;
}

geometry_msgs::PoseStamped curr_fused_pose;
void fused_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_fused_pose = *msg;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10); 
    ros::Publisher cmd_vel_unstamped_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("/gazebo/ground_truth/pose", 10, pose_cb);
    ros::Subscriber fused_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, fused_pose_cb);
    ros::ServiceClient set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("/mavros/setpoint_velocity/mav_frame");
            
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

   
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1.5;
    pose.pose.position.y = 1.5;
    pose.pose.position.z = 1.5;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0; // 0.383; // = 45 degree in z
    pose.pose.orientation.w = 0; //0.924; // = 45 degree in z
  
    geometry_msgs::Twist cmd_vel_unstamped;
    cmd_vel_unstamped.linear.x = 0;
    cmd_vel_unstamped.linear.y = 0;
    cmd_vel_unstamped.linear.z = 0;
    cmd_vel_unstamped.angular.x = 0;
    cmd_vel_unstamped.angular.y = 0;
    cmd_vel_unstamped.angular.z = 1; 
  

    //Stablish the cmd_vel frame
    mavros_msgs::SetMavFrame set_frame_msg;
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_BODY_NED;  // FRAME_LOCAL_NED = Global
    std::cout << "\nMavframe set: "<< set_cmd_vel_frame.call(set_frame_msg) << std::endl;
    

    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int cntr = 0;
    bool hight_check = false;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {    
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // to exit the while loop after the desire hight is reched + a little wait time 
        if(curr_pose.pose.pose.position.z >= pose.pose.position.z-0.1) hight_check = true;
        if(hight_check == true) cntr++;
        if(cntr == 1) std::cout << "Waiting...\n";    
        if(cntr > 200) break;

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    

    std::cout << "Publishing velocitiess now...\n";
    while(ros::ok())
    {
        cmd_vel_unstamped_pub.publish(cmd_vel_unstamped);

                                                
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}