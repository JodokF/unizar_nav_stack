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
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg);
        ros::Subscriber state_sub;
        ros::Subscriber vel_sub;
        ros::Publisher local_pos_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Publisher cmd_vel_unstmpd_pub;
        ros::Subscriber pose_sub;
        ros::ServiceClient set_cmd_vel_frame;
        
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::Twist cmd_vel_zero;

        mavros_msgs::SetMavFrame set_frame_msg;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;

        mavros_msgs::State current_state;
        nav_msgs::Odometry curr_pose;
        geometry_msgs::Twist vel_cmd_import;

    public:
        bool vel_break;
        int take_off();
        drone_connection(ros::NodeHandle& nh);
        ~drone_connection();
};

drone_connection::drone_connection(ros::NodeHandle& nh)
{
    state_sub = nh.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, &drone_connection::state_cb, this);
    vel_sub = nh.subscribe<geometry_msgs::Twist>
        ("/vel_cmd_2_drone", 10, &drone_connection::vel_cmd_cb, this);
    pose_sub = nh.subscribe<nav_msgs::Odometry>
        ("/gazebo/ground_truth/pose", 10, &drone_connection::pose_cb,this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("/mavros/set_mode");
    cmd_vel_unstmpd_pub = nh.advertise<geometry_msgs::Twist>
        ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
        ("/mavros/setpoint_velocity/mav_frame");

    start_pose.pose.position.x = -1;
    start_pose.pose.position.y = 2;
    start_pose.pose.position.z = 1.2;
    start_pose.pose.orientation.x = 0;
    start_pose.pose.orientation.y = 0;
    start_pose.pose.orientation.z = 0; // 0.383; // = 45 degree in z
    start_pose.pose.orientation.w = 0; //0.924; // = 45 degree in z
  
    cmd_vel_zero.linear.x = 0;
    cmd_vel_zero.linear.y = 0;
    cmd_vel_zero.linear.z = 0;
    cmd_vel_zero.angular.x = 0;
    cmd_vel_zero.angular.y = 0;
    cmd_vel_zero.angular.z = 0; 

    vel_break = false;

}

drone_connection::~drone_connection()
{
}


void drone_connection::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void drone_connection::pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    curr_pose = *msg;
}

void drone_connection::vel_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg){
    //vel_cmd_import = *msg;
    // vel_break = true;
    cmd_vel_unstmpd_pub.publish(*msg);
    
}

int drone_connection::take_off()
{
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate100(100.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){

        ros::spinOnce();
        rate100.sleep();
    }

    //Stablish the cmd_vel frame

    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_LOCAL_NED;  // FRAME_LOCAL_NED = world, FRAME_BODY_NED = drone
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    std::cout << "\nMavframe set: "<< set_cmd_vel_frame.call(set_frame_msg) << std::endl;
    

    //send a few setpoints before starting to establish the offboard connection
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub.publish(start_pose);
        ros::spinOnce();
        rate100.sleep();
    }

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

        local_pos_pub.publish(start_pose);
        if(vel_sub.getNumPublishers() > 0 || vel_break == true){
            ROS_INFO("Traj. vel. publisher detected.");
            return 0;
        }
        
        ros::spinOnce();
        rate100.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_gen");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner ich_spinne(1);
    ich_spinne.start();

    drone_connection drone(std::ref(node_handle));

    drone.take_off();

    ros::Rate rate1(1);

    std::cout << "Publishing velocitiess now...\n";
    while(ros::ok())
    {                               
        //ros::spinOnce();
        rate1.sleep();

    }

    return 0;
}