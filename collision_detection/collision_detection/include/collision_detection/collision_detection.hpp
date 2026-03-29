#pragma once

#include <crp_msgs/msg/ego.hpp>
#include <crp_msgs/msg/target_space.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

autoware_planning_msgs::msg::Trajectory 
calculate_trajectory(
  const crp_msgs::msg::Ego& ego,
  const crp_msgs::msg::TargetSpace& target_space);