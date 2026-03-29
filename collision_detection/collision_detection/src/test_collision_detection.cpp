#include "collision_detection/collision_detection.hpp"
#include <iostream>
#include <iomanip>

int main() {
  // Mock Ego data
  crp_msgs::msg::Ego ego;
  ego.pose.pose.position.x = 0.0;
  ego.pose.pose.position.y = 0.0;
  ego.twist.twist.linear.x = 20.0;  // 20 m/s forward
  ego.twist.twist.linear.y = 0.0;
  ego.accel.accel.linear.x = 0.0;   // Constant speed
  ego.accel.accel.linear.y = 0.0;

  // Mock TargetSpace with one object
  crp_msgs::msg::TargetSpace target_space;
  target_space.header.stamp.sec = 0;
  target_space.header.stamp.nanosec = 0;

  autoware_perception_msgs::msg::PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 50.0;  // 50m ahead
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;   // Stationary
  obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;
  obj.kinematics.initial_acceleration_with_covariance.accel.linear.x = 0.0;
  obj.kinematics.initial_acceleration_with_covariance.accel.linear.y = 0.0;

  target_space.relevant_objects.push_back(obj);

  // Call the function
  auto trajectory = calculate_trajectory(ego, target_space);

  // Print results
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Trajectory Points: " << trajectory.points.size() << std::endl;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto& p = trajectory.points[i];
    std::cout << "Point " << i << ": "
              << "x=" << p.pose.position.x << ", "
              << "y=" << p.pose.position.y << ", "
              << "vel=" << p.longitudinal_velocity_mps << ", "
              << "acc=" << p.acceleration_mps2 << ", "
              << "time=" << p.time_from_start.sec + p.time_from_start.nanosec / 1e9 << "s"
              << std::endl;
  }

  return 0;
}