#ifndef BEHAVIOR_PLANNER_HPP
#define BEHAVIOR_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "crp_msgs/msg/ego.hpp"
#include "crp_msgs/msg/scenario.hpp"
#include "crp_msgs/msg/target_space.hpp"
#include "tier4_planning_msgs/msg/scenario.hpp"

class BehaviorPlanner : public rclcpp::Node
{
public:
    BehaviorPlanner();

private:
    void scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg);
    void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg);
    void timerCallback();

    autoware_perception_msgs::msg::PredictedObject calcMissingObject(const autoware_perception_msgs::msg::PredictedObject& obj,double dt_seconds);

    rclcpp::Subscription<crp_msgs::msg::Scenario>::SharedPtr sub_scenario;
    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr sub_ego;

    rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr pub_scenario;
    rclcpp::Publisher<crp_msgs::msg::TargetSpace>::SharedPtr pub_targetSpace;

    rclcpp::TimerBase::SharedPtr timer_pub;

    crp_msgs::msg::Ego::SharedPtr last_ego;
    crp_msgs::msg::Scenario::SharedPtr last_scenario;
    tier4_planning_msgs::msg::Scenario out_scenario;
    crp_msgs::msg::TargetSpace out_targetSpace;
    bool debugEnabled;

    crp_msgs::msg::Scenario::SharedPtr last_valid_scenario_;
    rclcpp::Time last_valid_scenario_time_;
    int missing_scenario_cycles_ = 0;
    static constexpr int MAX_MISSING_CYCLES = 5;

    std::vector<autoware_perception_msgs::msg::PredictedObject> relevant_obstacles;
    std::vector<autoware_perception_msgs::msg::PredictedObject> relevant_objects;
    std::vector<autoware_perception_msgs::msg::PredictedObject> obstacles;
    std::vector<autoware_perception_msgs::msg::PredictedObject> objects;

};

autoware_perception_msgs::msg::PredictedObject BehaviorPlanner::calcMissingObject(const autoware_perception_msgs::msg::PredictedObject& obj,double dt)
{
    auto ext = obj;
 
    double x0  = obj.kinematics.initial_pose_with_covariance.pose.position.x;
    double y0  = obj.kinematics.initial_pose_with_covariance.pose.position.y;
    double vx0 = obj.kinematics.initial_twist_with_covariance.twist.linear.x;
    double vy0 = obj.kinematics.initial_twist_with_covariance.twist.linear.y;
    double ax0 = obj.kinematics.initial_acceleration_with_covariance.accel.linear.x;
    double ay0 = obj.kinematics.initial_acceleration_with_covariance.accel.linear.y;
 
    double dt2 = dt * dt;
 
    ext.kinematics.initial_pose_with_covariance.pose.position.x = x0 + vx0*dt + 0.5*ax0*dt2;
    ext.kinematics.initial_pose_with_covariance.pose.position.y = y0 + vy0*dt + 0.5*ay0*dt2;
    ext.kinematics.initial_twist_with_covariance.twist.linear.x = vx0 + ax0*dt;
    ext.kinematics.initial_twist_with_covariance.twist.linear.y = vy0 + ay0*dt;
 
    return ext;
}

#endif
