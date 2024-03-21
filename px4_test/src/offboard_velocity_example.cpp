#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/State.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    std::cout << "Current State: " << current_state.mode << "\n";
}

//--------------------
//-------MAIN---------
int main(int argc, char **argv)
{
    //ROS INIT
    ros::init(argc, argv, "offb_node_loop");
    //node handle
    ros::NodeHandle nh;


    //---------SUBSCRIBERS-----------
    //MAVROS STATE SUBS
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //---------END OFSUBSCRIBERS-----------



    //---------PUBLISHERS-----------
    //MAVROS SETPOINT (LOCAL REFEFERENCE)
    ros::Publisher cmd_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
        ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    //---------END OF PUBLISHERS-----------



    //---------SERVICE CLIENTS-----------
    //MAVROS FRAME
    ros::ServiceClient set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("/mavros/setpoint_velocity/mav_frame");
    //MAVROS ARMING
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //MAVROS SET MODE
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
     //---------END OF SERVICE CLIENTS-----------



    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_(tf_buffer_, nh);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = -0.2; //5.0/180 * M_PI;
    
    
    //------------SEND INITIAL 100 POINTS TO CHANGE TO OFFBOARD------------------
    

    for(int i = 100; ros::ok() && i > 0; --i){
        
        //Publish CMD_POS
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }
    //------------END OF SEND INITIAL 100 POINTS TO CHANGE TO OFFBOARD------------------

    
    
    // --------ARMING AND OFFBOARD --------------
    //REQUEST OFFBOARD FLIGHT MODE
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //REQUEST ARM DRONE
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //LOOP UNTIL POSITION AND  ARMED
    while(ros::ok() && !current_state.armed && current_state.mode != "POSITION"){
        ROS_INFO("Arm manually and enable position mode before going offboard");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    bool offboard = false;

    //LOOP UNTIL OFFBOARD
    do{
        ROS_INFO("Trying to go offboard");
        //Pass offboard
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            offboard = true;
        }else{
           
            if(!current_state.armed){
                ROS_INFO("NOT ARMED, ARM FIRST");
            }
        }
    }while(ros::ok() && !offboard);
    // --------END OF ARMING AND OFFBOARD --------------


    std::cout << "Pulbishing Velocity... \n";
    while(ros::ok()){
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
