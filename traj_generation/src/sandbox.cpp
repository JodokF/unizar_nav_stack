#include <ros/ros.h>
#include <tf/transform_datatypes.h> // For quaternion to yaw conversion
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>



int main(int argc, char** argv) {
    ros::init(argc, argv, "quaternion_to_yaw");

    geometry_msgs::PoseStamped pose;

    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0.5;
    pose.pose.orientation.w = 0.8660254;
    
    std::cout << "Yaw in degree: " << (180 * tf::getYaw(pose.pose.orientation))/M_PI << std::endl;
    


    return 0;
}


