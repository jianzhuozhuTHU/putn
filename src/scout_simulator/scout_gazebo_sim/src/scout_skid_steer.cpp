/*
 * scout_skid_steer.cpp
 *
 * Created on: Mar 25, 2020 22:54
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_gazebo/scout_skid_steer.hpp"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace wescore {
ScoutSkidSteer::ScoutSkidSteer(ros::NodeHandle *nh, std::string robot_name)
    : nh_(nh), robot_name_(robot_name) {
  motor_fr_topic_ = robot_name_ + "/scout_motor_fr_controller/command";
  motor_fl_topic_ = robot_name_ + "/scout_motor_fl_controller/command";
  motor_rl_topic_ = robot_name_ + "/scout_motor_rl_controller/command";
  motor_rr_topic_ = robot_name_ + "/scout_motor_rr_controller/command";
  cmd_topic_ = robot_name_ + "/cmd_vel";
}

void ScoutSkidSteer::SetupSubscription() {
  // command subscriber
  cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      cmd_topic_, 5, &ScoutSkidSteer::TwistCmdCallback, this);

  // motor command publisher
  motor_fr_pub_ = nh_->advertise<std_msgs::Float64>(motor_fr_topic_, 50);
  motor_fl_pub_ = nh_->advertise<std_msgs::Float64>(motor_fl_topic_, 50);
  motor_rl_pub_ = nh_->advertise<std_msgs::Float64>(motor_rl_topic_, 50);
  motor_rr_pub_ = nh_->advertise<std_msgs::Float64>(motor_rr_topic_, 50);
}

void ScoutSkidSteer::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  std_msgs::Float64 motor_cmd[4];

  double driving_vel = msg->linear.x;
  double steering_vel = msg->angular.z;

  double left_side_velocity =
      (driving_vel - steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;
  double right_side_velocity =
      (driving_vel + steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;

  motor_cmd[0].data = right_side_velocity;
  motor_cmd[1].data = -left_side_velocity;
  motor_cmd[2].data = -left_side_velocity;
  motor_cmd[3].data = right_side_velocity;

  motor_fr_pub_.publish(motor_cmd[0]);
  motor_fl_pub_.publish(motor_cmd[1]);
  motor_rl_pub_.publish(motor_cmd[2]);
  motor_rr_pub_.publish(motor_cmd[3]);
}

}  // namespace wescore