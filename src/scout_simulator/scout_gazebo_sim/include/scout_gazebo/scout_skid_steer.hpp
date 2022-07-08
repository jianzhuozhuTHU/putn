/*
 * scout_skid_steer.hpp
 *
 * Created on: Mar 25, 2020 22:52
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SKID_STEER_HPP
#define SCOUT_SKID_STEER_HPP

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <string>

namespace wescore {
class ScoutSkidSteer {
 public:
  ScoutSkidSteer(ros::NodeHandle *nh, std::string robot_name = "");

  void SetupSubscription();

 private:
  std::string robot_name_;
  std::string motor_fr_topic_;
  std::string motor_fl_topic_;
  std::string motor_rl_topic_;
  std::string motor_rr_topic_;
  std::string cmd_topic_;

  const double SCOUT_WHEELBASE = 0.498;
  const double SCOUT_WHEEL_RADIUS = 0.16459;

  ros::NodeHandle *nh_;

  ros::Publisher motor_fr_pub_;
  ros::Publisher motor_fl_pub_;
  ros::Publisher motor_rl_pub_;
  ros::Publisher motor_rr_pub_;

  ros::Subscriber cmd_sub_;

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
};
}  // namespace wescore

#endif /* SCOUT_SKID_STEER_HPP */
