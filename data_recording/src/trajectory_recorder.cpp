#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


class trajectory_recorder{
    private:
        ros::NodeHandle nh;
        ros::Subscriber pose_subscriber;
        ros::Publisher path_publisher;
        
        std::string observed_pose_topic, recorded_path_topic;
        nav_msgs::Path recorded_path;
    public:
        trajectory_recorder(char** argv){
            //nh.param("observed_pose_topic", observed_pose_topic, 
            //        std::string("/optitrack/pose"));
            //nh.param("recorded_path_topic", recorded_path_topic, 
            //        std::string("/path_gt"));
            observed_pose_topic = argv[1];
            recorded_path_topic = argv[2];
            pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>(observed_pose_topic, 10, &trajectory_recorder::poseCallback, this);
            path_publisher = nh.advertise<nav_msgs::Path>(recorded_path_topic,10);
        }

        void poseCallback(geometry_msgs::PoseStamped msg){
            recorded_path.poses.push_back(msg);
            recorded_path.header.frame_id = msg.header.frame_id;
        }

        void publishPath(){
            recorded_path.header.stamp = ros::Time::now();
            path_publisher.publish(recorded_path);
        }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_recorder");
    if(argc > 1){
        trajectory_recorder recorder(argv);
        ros::Rate rate(1);
        while(ros::ok()){
            recorder.publishPath();
            rate.sleep();
            ros::spinOnce();
        }
    }
    return 0;
}