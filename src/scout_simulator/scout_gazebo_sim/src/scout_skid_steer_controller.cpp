#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include "scout_gazebo/scout_skid_steer.hpp"

using namespace wescore;

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  // fetch parameters
  std::string robot_namespace;
  private_node.param<std::string>("robot_namespace", robot_namespace,
                                  std::string("scout_default"));

  ROS_INFO("Namespace: %s", robot_namespace.c_str());

  ScoutSkidSteer skid_steer_controller(&node, robot_namespace);
  skid_steer_controller.SetupSubscription();

  ros::spin();

  return 0;
}