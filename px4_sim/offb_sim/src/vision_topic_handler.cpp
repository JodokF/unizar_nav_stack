#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>



int check = 0;
class PoseConverterNode {
public:
    PoseConverterNode() {

        nh_ = ros::NodeHandle("~");

        vision_pose_sub = nh_.subscribe("/gazebo/ground_truth/pose", 10, &PoseConverterNode::visionPoseCallback, this);

        converted_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    }

    void visionPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::PoseStamped pose_converted;
        check++;
        if (check == 1) ROS_INFO("Converting messages...\n");

        pose_converted.header = msg->header;
        pose_converted.pose = msg->pose.pose;

        converted_pose_pub.publish(pose_converted);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber vision_pose_sub;
    ros::Publisher converted_pose_pub;
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "pose_converter_node");
    PoseConverterNode node;

    ros::spin();

    return 0;
}


