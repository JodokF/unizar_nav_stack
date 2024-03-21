
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <iostream>


geometry_msgs::Twist v;


bool initPublish = false;
int i=0;


void velCallback(geometry_msgs::Twist msg){//Cada T=100 ms se llama a este callback y se actualiza el valor del comando de velocidad
    std::mutex mutex;

    i++;
    mutex.lock();
        v=msg;
        initPublish = true;
    mutex.unlock();

}

int main(int argc, char *argv[]){//Cada 10 ms se publica el comando de velocidad en el topic "cmd_vel" (hector_quadrotor)
    // Set up ROS.
    ros::init(argc, argv, "velocity_command");
    ros::NodeHandle n;


    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel"/*"/test_vel"*/, 10);
    ros::Subscriber sub = n.subscribe("/cmd_vel_control", 1, velCallback);

    const double T=0.01;
    ros::Rate loop_rate(1/T);

    // Main loop.
    while (ros::ok()){
      // Publish the message.
      if (initPublish) pub.publish(v);

      ros::spinOnce();
      loop_rate.sleep();
    }

        return 0;
}

