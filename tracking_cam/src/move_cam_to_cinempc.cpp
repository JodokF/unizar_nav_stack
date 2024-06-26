#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_cam_to_cinempc");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  std::string source_frame = "odom";  // replace with your frame
  std::string target_frame = "cine_mpc";  // replace with your frame

  ros::Rate rate(10.0); // Check at 10 Hz
  
  while (nh.ok()) {
    try {
      listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);

      // Set parameters based on the transform
      nh.setParam("/frame_offset/x", transform.getOrigin().x());
      nh.setParam("/frame_offset/y", transform.getOrigin().y());
      nh.setParam("/frame_offset/z", transform.getOrigin().z());
      nh.setParam("/frame_offset/roll", transform.getRotation().getX());
      nh.setParam("/frame_offset/pitch", transform.getRotation().getY());
      nh.setParam("/frame_offset/yaw", transform.getRotation().getZ());

      ROS_INFO("Transform set as parameters");
      break; // Exit once parameters are set

    } catch (tf::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }

  return 0;
}
