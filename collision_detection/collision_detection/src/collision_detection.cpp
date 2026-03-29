#include "collision_detection/collision_detection.hpp"
#include <cmath>
#include <limits>

double calculate_universal_ttc(double distance, double v_ego, double a_ego,
                               double v_target, double a_target,
                               double accel_threshold = 1e-5) {
  if (distance <= 0.0) return 0.0;

  double rel_v = v_ego - v_target;
  double rel_a = a_ego - a_target;

  if (std::abs(rel_a) < accel_threshold) {
    if (rel_v <= 0.0)
      return std::numeric_limits<double>::infinity();
    return distance / rel_v;
  }

  double disc = rel_v * rel_v + 2.0 * rel_a * distance;
  if (disc < 0.0) return std::numeric_limits<double>::infinity();

  double sq = std::sqrt(disc);
  double t1 = (-rel_v + sq) / rel_a;
  double t2 = (-rel_v - sq) / rel_a;

  double t = std::numeric_limits<double>::infinity();
  if (t1 > 0.0) t = std::min(t, t1);
  if (t2 > 0.0) t = std::min(t, t2);
  return t;
}

autoware_planning_msgs::msg::Trajectory 
calculate_trajectory(const crp_msgs::msg::Ego& ego,
                     const crp_msgs::msg::TargetSpace& target_space) {
  autoware_planning_msgs::msg::Trajectory traj;
  traj.header = target_space.header;

  double ego_x = ego.pose.pose.position.x;
  double ego_y = ego.pose.pose.position.y;
  double ego_v =
      std::hypot(ego.twist.twist.linear.x, ego.twist.twist.linear.y);
  double ego_a =
      std::hypot(ego.accel.accel.linear.x, ego.accel.accel.linear.y);

  for (const auto& obj : target_space.relevant_objects) {
    double tx = obj.kinematics.initial_pose_with_covariance.pose.position.x;
    double ty = obj.kinematics.initial_pose_with_covariance.pose.position.y;
    double distance = std::hypot(tx - ego_x, ty - ego_y);

    double v_target = std::hypot(
        obj.kinematics.initial_twist_with_covariance.twist.linear.x,
        obj.kinematics.initial_twist_with_covariance.twist.linear.y);
    double a_target = std::hypot(
        obj.kinematics.initial_acceleration_with_covariance.accel.linear.x,
        obj.kinematics.initial_acceleration_with_covariance.accel.linear.y);

    double ttc = calculate_universal_ttc(distance, ego_v, ego_a, v_target, a_target);

    if (ttc > 0.0 && ttc < std::numeric_limits<double>::infinity()) {
      const int N = 5;
      const double horizon = std::min(ttc, 5.0);
      for (int i = 0; i < N; ++i) {
        double t = horizon * (i / double(N - 1));
        autoware_planning_msgs::msg::TrajectoryPoint p;
        p.pose.position.x = ego_x + ego_v * t + 0.5 * ego_a * t * t;
        p.pose.position.y = ego_y;
        p.longitudinal_velocity_mps = ego_v + ego_a * t;
        p.acceleration_mps2 = ego_a;
        p.time_from_start.sec = static_cast<int32_t>(t);
        p.time_from_start.nanosec = static_cast<uint32_t>((t - p.time_from_start.sec) * 1e9);
        traj.points.push_back(p);
      }
    }
  }

  for (const auto& obj : target_space.relevant_obstacles) {
    double tx = obj.kinematics.initial_pose_with_covariance.pose.position.x;
    double ty = obj.kinematics.initial_pose_with_covariance.pose.position.y;
    double distance = std::hypot(tx - ego_x, ty - ego_y);

    double v_target = std::hypot(
        obj.kinematics.initial_twist_with_covariance.twist.linear.x,
        obj.kinematics.initial_twist_with_covariance.twist.linear.y);
    double a_target = std::hypot(
        obj.kinematics.initial_acceleration_with_covariance.accel.linear.x,
        obj.kinematics.initial_acceleration_with_covariance.accel.linear.y);

    double ttc = calculate_universal_ttc(distance, ego_v, ego_a, v_target, a_target);

    if (ttc > 0.0 && ttc < std::numeric_limits<double>::infinity()) {
      const int N = 5;
      const double horizon = std::min(ttc, 5.0);
      for (int i = 0; i < N; ++i) {
        double t = horizon * (i / double(N - 1));
        autoware_planning_msgs::msg::TrajectoryPoint p;
        p.pose.position.x = ego_x + ego_v * t + 0.5 * ego_a * t * t;
        p.pose.position.y = ego_y;
        p.longitudinal_velocity_mps = ego_v + ego_a * t;
        p.acceleration_mps2 = ego_a;
        p.time_from_start.sec = static_cast<int32_t>(t);
        p.time_from_start.nanosec = static_cast<uint32_t>((t - p.time_from_start.sec) * 1e9);
        traj.points.push_back(p);
      }
    }
  }

  if (traj.points.empty()) {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = ego_x;
    p.pose.position.y = ego_y;
    p.longitudinal_velocity_mps = ego_v;
    p.acceleration_mps2 = ego_a;
    p.time_from_start.sec = 0;
    p.time_from_start.nanosec = 0;
    traj.points.push_back(p);
  }

  return traj;
}