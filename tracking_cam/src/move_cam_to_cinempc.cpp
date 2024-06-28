#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_cam_to_cinempc");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  std::string source_frame = "odom";  // calculate the position from here
  std::string target_frame = "cine_mpc";  // to here

  ros::Rate rate(10.0); // Check at 10 Hz
  
  while (nh.ok()) {
    try {
      listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);

      double qx = transform.getRotation().getX();
      double qy = transform.getRotation().getY();
      double qz = transform.getRotation().getZ();
      double qw = transform.getRotation().getW();

      tf::Quaternion quaternion(qx, qy, qz, qw);
      double roll, pitch, yaw;
      tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

      // Set parameters based on the transform
      nh.setParam("/frame_offset/x", transform.getOrigin().x());
      nh.setParam("/frame_offset/y", transform.getOrigin().y());
      nh.setParam("/frame_offset/z", transform.getOrigin().z()); 
      nh.setParam("/frame_offset/roll", roll);
      nh.setParam("/frame_offset/pitch", pitch);
      nh.setParam("/frame_offset/yaw", yaw);

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
