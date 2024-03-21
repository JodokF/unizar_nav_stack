#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/State.h>
#include <iomanip> 

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class real_drone_connection{
    private:
        ros::Subscriber current_pose_sub, planned_vel_sub, state_sub;
        ros::ServiceClient arming_client, set_cmd_vel_frame, set_mode_client;


        void state_cb(const mavros_msgs::State::ConstPtr& msg){
            current_state = *msg;
            std::cout << "--- Curr. State: " << current_state.mode << "\n";
        };

        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
            current_pose = *msg;
        };

        void vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
            planned_vel = *msg;
        };


    public:

    real_drone_connection(ros::NodeHandle& nh);
    int establish_drone_connection();
    geometry_msgs::PoseStamped current_pose;
    mavros_msgs::State current_state;
    geometry_msgs::Twist planned_vel;
    geometry_msgs::Twist cmd_vel;

    ros::Publisher cmd_vel_pub;


};

real_drone_connection::real_drone_connection(ros::NodeHandle& nh){

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    planned_vel_sub = nh.subscribe<geometry_msgs::Twist>
            ("/vel2real_drone", 10, vel_cb);

    current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/optitrack/pose",10, pose_cb);

    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    set_cmd_vel_frame = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("/mavros/setpoint_velocity/mav_frame");

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("mavros/setpoint_velocity/cmd_vel", 10);

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0; //5.0/180 * M_PI;


}

void real_drone_connection::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    std::cout << "--- Curr. State: " << current_state.mode << "\n";
};

void real_drone_connection::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
};

void real_drone_connection::vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
    planned_vel = *msg;
};

int real_drone_connection::establish_drone_connection()
{
    
    //tf2_ros::Buffer tf_buffer_;
    //tf2_ros::TransformListener tf_listener_(tf_buffer_, nh);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    //Stablish the cmd_vel frame
    mavros_msgs::SetMavFrame set_frame_msg;
    set_frame_msg.request.mav_frame = set_frame_msg.request.FRAME_BODY_NED;
    set_cmd_vel_frame.call(set_frame_msg);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }

    // --------ARMING AND OFFBOARD --------------
    //REQUEST OFFBOARD FLIGHT MODE
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //REQUEST ARM DRONE
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    //LOOP UNTIL POSITION AND ARMED
    ros::Rate wait_rate(1.0);
    
    while(ros::ok() && !current_state.armed && current_state.mode != "POSITION"){
        ROS_INFO("Arm manually and enable position mode before going offboard");
        ros::spinOnce();
        wait_rate.sleep();
    }
    
    bool offboard = false;

    //LOOP UNTIL OFFBOARD
    do{
        ROS_INFO("Trying to go offboard");
        //Pass offboard
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            offboard = true;
        }else{
            if(!current_state.armed){
                ROS_INFO("NOT ARMED, ARM FIRST");
            }
        }
    }while(ros::ok() && !offboard);
    // --------END OF ARMING AND OFFBOARD --------------


}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    //ros::AsyncSpinner ich_spinne(1);
    //ich_spinne.start();
    //ros::NodeHandle node_handle("~");
    ros::NodeHandle nh;
    // poly_traj_plan ptp(std::ref(node_handle));
    real_drone_connection drone(std::ref(nh));
    
    drone.establish_drone_connection();
    
    ros::Rate rate(20.0);
    while(ros::ok()){
        drone.cmd_vel = drone.planned_vel;
        drone.cmd_vel_pub.publish(drone.cmd_vel);
       
        /************     Debugg Info     ***********/
        std::cout << std::fixed << std::setprecision(2);
        /*std::cout   << "Current Pose(x, y, z): ("
                    << current_pose.pose.position.x << ", " 
                    << current_pose.pose.position.y << ", " 
                    << current_pose.pose.position.z << ")\n " ;*/
        /************     Debugg Info     ***********/

        // std::cout << "Velocity published\n";

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

