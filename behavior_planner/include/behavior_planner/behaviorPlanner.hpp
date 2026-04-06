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

#endif
