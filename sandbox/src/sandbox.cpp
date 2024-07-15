

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>




class poseTfBroadcaster {
    private:

    ros::NodeHandle nh;
    ros::Subscriber subs;
    ros::Publisher pub;
    geometry_msgs::PoseStamped transformed_pose, temp_pose;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_; 

    public:
        poseTfBroadcaster() :  tf_listener_(tf_buffer_, nh){
            nh = ros::NodeHandle();
            subs = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample_throttled",10,&poseTfBroadcaster::poseCallback,this);
 	        pub = nh.advertise<geometry_msgs::PoseStamped>("/walawalabumbum",10);
        }

        void poseCallback(nav_msgs::Odometry recieved_pose){

            temp_pose.header = recieved_pose.header;
            temp_pose.pose = recieved_pose.pose.pose;

            geometry_msgs::TransformStamped tf_odom_odomNED;
            try{
                tf_odom_odomNED = tf_buffer_.lookupTransform("odom", "camera_odom_frame", ros::Time(0));
            } catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
	    tf2::doTransform(temp_pose, transformed_pose, tf_odom_odomNED);
	    transformed_pose.header.frame_id="odom";
            transformed_pose.header.stamp=temp_pose.header.stamp;
        }

	void timedPubCallback(void){
	    pub.publish(transformed_pose);  
	}
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_tf_broadcaster");
    poseTfBroadcaster pose_tf_broadcaster;
    ros::Rate loop_rate(40);
    while (ros::ok()){
      pose_tf_broadcaster.timedPubCallback();
      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}

