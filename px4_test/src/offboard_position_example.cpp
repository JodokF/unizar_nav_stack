#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMavFrame.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher cmd_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::ServiceClient set_cmd_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("/mavros/setpoint_position/mav_frame");

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_(tf_buffer_, nh);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ros::Rate wait_rate(1.0);
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pos_localframe, cmd_pos;
    pos_localframe.pose.position.x = 1;
    pos_localframe.pose.position.y = 0;
    pos_localframe.pose.position.z = 1;
    pos_localframe.pose.orientation.x = 0;
    pos_localframe.pose.orientation.y = 0;
    pos_localframe.pose.orientation.z = 0;
    pos_localframe.pose.orientation.w = 1;
    pos_localframe.header.frame_id = "base_link";
/* 
    //Stablish the cmd frame
    mavros_msgs::SetMavFrame set_frame_msg;
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_BODY_FRD;
    set_cmd_frame.call(set_frame_msg);
      */

    wait_rate.sleep();
    geometry_msgs::TransformStamped transform_msg;
    bool tf_recieved = false;
    while(!tf_recieved){
        try{
            transform_msg = tf_buffer_.lookupTransform("odom_ned", "base_link", ros::Time(0));
            tf2::doTransform(pos_localframe, cmd_pos, transform_msg);
            cmd_pos.header.frame_id = "odom_ned";
            tf_recieved = true;
        } catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }  
    


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_pos_pub.publish(cmd_pos);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    


    while(ros::ok() && !current_state.armed && current_state.mode != "POSITION"){
        ROS_INFO("Arm manually and enable position mode before going offboard");
        ros::spinOnce();
        wait_rate.sleep();
    }
    bool offboard = false;
    do{
        ROS_INFO("Trying to go offboard");
        //Pass offboard
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            offboard = true;
        }else{
            ROS_INFO("NOT ABLE TO PASS OFFBOARD");
            if(!current_state.armed){
                ROS_INFO("NOT ARMED, ARM FIRST");
            }
        }
    }while(ros::ok() && !offboard);

    while(ros::ok()){
        cmd_pos_pub.publish(cmd_pos);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
