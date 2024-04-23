
// not like this i think

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


class p_controller
{
    private:
        void setpoint_cb(const geometry_msgs::Twist::ConstPtr& msg);
        void feedback_cp(const nav_msgs::Odometry::ConstPtr& msg);
             
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Twist current_setpoint;
        nav_msgs::Odometry feedback_pose;

        // geometry_msgs::PoseStamped start_pose;

        // ros::Time ros_time_last;

        double k_gain_x, k_gain_y, k_gain_z;

    public:
        ros::Subscriber setpoint_sub, feedback_sub;
        ros::Publisher cmd_vel_unstmpd_pub;
        p_controller(ros::NodeHandle& nh);
        ~p_controller();

        void p_cntrl();

};

p_controller::p_controller(ros::NodeHandle& nh)
{
    setpoint_sub = nh.subscribe<geometry_msgs::Twist>
        ("/cmd_vel_setpoint", 10, &p_controller::setpoint_cb, this);

    feedback_sub = nh.subscribe<nav_msgs::Odometry>
        ("/mavros/odometry/out", 10, &p_controller::feedback_cp, this);
        
    cmd_vel_unstmpd_pub = nh.advertise<geometry_msgs::Twist>
        ("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    /*
    start_pose.pose.position.x = -1;
    start_pose.pose.position.y = 2;
    start_pose.pose.position.z = 1.2;
    start_pose.pose.orientation.x = 0;
    start_pose.pose.orientation.y = 0;
    start_pose.pose.orientation.z = 0; // 0.383; // = 45 degree in z
    start_pose.pose.orientation.w = 0; //0.924; // = 45 degree in z
    */

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0; 

    k_gain_x = 0.8;
    k_gain_y = 0.8;

    k_gain_z = 1.65;

    // ros_time_last = ros::Time::now(); 

}

p_controller::~p_controller()
{
    // to-do
}

void p_controller::feedback_cp(const nav_msgs::Odometry::ConstPtr& msg){
    feedback_pose = *msg;

}

void p_controller::setpoint_cb(const geometry_msgs::Twist::ConstPtr& msg){
        // To print publishing frequenz:
    // std::cout << std::fixed << std::showpoint;
    // std::cout << std::setprecision(3);
    // std::cout << "Publ. time: " << ros::Time::now() - ros_time_last << " - ";
    // ros_time_last = ros::Time::now(); 
    
    current_setpoint = *msg;


}

void p_controller::p_cntrl(){
    
    std::cout << std::fixed << std::showpoint;
    std::cout << std::setprecision(3); 

    cmd_vel.linear.x =  current_setpoint.linear.x + k_gain_x * (current_setpoint.linear.x - feedback_pose.twist.twist.linear.x);
    cmd_vel.linear.y =  current_setpoint.linear.y + k_gain_y * (current_setpoint.linear.y - feedback_pose.twist.twist.linear.y);
    cmd_vel.linear.z =  current_setpoint.linear.z + k_gain_z * (current_setpoint.linear.z - feedback_pose.twist.twist.linear.z);
    cmd_vel.angular.x = current_setpoint.angular.x;
    cmd_vel.angular.y = current_setpoint.angular.y;
    cmd_vel.angular.z = current_setpoint.angular.z;

    // std::cout << "Sp. - Fb. x:  " << current_setpoint.linear.x << " - " << feedback_pose.twist.twist.linear.x << std::endl;
    // std::cout << "Sp. - Fb. y:  " << current_setpoint.linear.y << " - " << feedback_pose.twist.twist.linear.y << std::endl;
    // std::cout << "Sp. - Fb. z:  " << current_setpoint.linear.z << " - " << feedback_pose.twist.twist.linear.z << std::endl;
    std::cout << "Pub. Vel. (x, y, z) k: (" << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ", " << cmd_vel.linear.z << ") " << k_gain_z << "\n";
    std::cout << "---\n";
    cmd_vel_unstmpd_pub.publish(cmd_vel);


}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "cntrlr");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner ich_spinne(1);
    ich_spinne.start();

    p_controller cntrl(std::ref(node_handle));


    ros::Rate rate100(100);

    ROS_INFO("Publishing velocitiess now...");
    while( cntrl.setpoint_sub.getNumPublishers() == 0)
    {
        rate100.sleep();
    }

    while(ros::ok())
    {                           

        cntrl.p_cntrl();
        ros::spinOnce();
        rate100.sleep();

    }

    return 0;
}