#include <ros/ros.h>
#include <std_msgs/Int32.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "connection_checker");
    ros::NodeHandle nh;

    ros::Publisher int_publisher = nh.advertise<std_msgs::Int32>("/zzz_connection_checker", 10);


    // Debug to check if nodes are connected:
    ros::Rate loop_rate(1); 
    int count = 0;
    std_msgs::Int32 msg;

    while (ros::ok()) {

        msg.data = count;
        int_publisher.publish(msg);
        ROS_INFO("Published: %d", msg.data);
        count++;
        loop_rate.sleep();

    }
    
}