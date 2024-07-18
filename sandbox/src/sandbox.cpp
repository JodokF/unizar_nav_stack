#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>


class tf_intercom {
    private:

    ros::NodeHandle nh;
    geometry_msgs::PoseStamped tf_pose_org, tf_pose_new;
    geometry_msgs::TransformStamped tf_transformation;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener; 
    ros::Publisher read_tf;
    std::string target_frame, source_frame;

    public:
        tf_intercom() :  tf_listener(tf_buffer, nh){
            nh = ros::NodeHandle();
            
            read_tf = nh.advertise<geometry_msgs::PoseStamped>("/tf_read_pose",10);
            target_frame = "odom";
            source_frame = "base_link";

            // this is necessary otherwise the orientation will be n.a.n.:
            tf_pose_org.pose.orientation.w = 1.0;
        }

        void readtfTree();
};
        

void tf_intercom::readtfTree()
{
    // to evade some error msgs at the startup
    if (tf_buffer.canTransform(target_frame, source_frame, ros::Time(0))) {
        try{
            tf_transformation = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
        } catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        
        tf2::doTransform(tf_pose_org, tf_pose_new, tf_transformation);
        read_tf.publish(tf_pose_new);

    } 
    else {
    ROS_WARN("Transform not available yet, waiting...");
    ros::Duration(1).sleep();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_intercom");
    tf_intercom tf_intercom;
    ros::Rate loop_rate(40);

    while (ros::ok()){
      tf_intercom.readtfTree();
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}

