#include "behavior_planner/behaviorPlanner.hpp"

BehaviorPlanner::BehaviorPlanner() : Node("behavior_planner")
{
    // declare and retrieve parameters
    this->declare_parameter<std::string>("scenario_topic", "/scenario");
    this->declare_parameter<std::string>("ego_topic", "/ego");
    this->declare_parameter<std::string>("strategy_topic", "/plan/strategy");
    this->declare_parameter<std::string>("targetSpace_topic", "/plan/target_space");
    this->declare_parameter<bool>("debugEnabled", false);

    std::string topicScenario, topicEgo, topicStrategy, topicTargetSpace;
    this->get_parameter<std::string>("scenario_topic", topicScenario);
    this->get_parameter<std::string>("ego_topic", topicEgo);
    this->get_parameter<std::string>("strategy_topic", topicStrategy);
    this->get_parameter<std::string>("targetSpace_topic", topicTargetSpace);

    // create subscriber and attach callbacks to the counterCallback method
    sub_scenario = this->create_subscription<crp_msgs::msg::Scenario>(
        topicScenario,
        3,
        std::bind(&BehaviorPlanner::scenarioCallback, this, std::placeholders::_1)
    );

    sub_ego = this->create_subscription<crp_msgs::msg::Ego>(
        topicEgo,
        3,
        std::bind(&BehaviorPlanner::egoCallback, this, std::placeholders::_1)
    );

    pub_scenario = this->create_publisher<tier4_planning_msgs::msg::Scenario>(
        topicStrategy,
        1
    );

    pub_targetSpace = this->create_publisher<crp_msgs::msg::TargetSpace>(
        topicTargetSpace,
        1
    );

    timer_pub = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&BehaviorPlanner::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "behavior_planner node has been started");
}


void BehaviorPlanner::scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg) {
    bool debugEnabled = this->get_parameter<bool>("debugEnabled", debugEnabled);
    last_scenario = msg;

    if (debugEnabled)
    {
        RCLCPP_INFO(this->get_logger(), "New Scenario received!");
    }
    
}

void BehaviorPlanner::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg) {
    bool debugEnabled = this->get_parameter<bool>("debugEnabled", debugEnabled);
    last_ego = msg;

    if (debugEnabled)
    {
        RCLCPP_INFO(this->get_logger(), "New Ego received!");
    }
}

void BehaviorPlanner::timerCallback() {
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorPlanner>());
    rclcpp::shutdown();
    return 0;
}
