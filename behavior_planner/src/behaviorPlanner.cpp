#include "behavior_planner/behaviorPlanner.hpp"

BehaviorPlanner::BehaviorPlanner() : Node("behavior_planner")
{
    // declare parameters
    this->declare_parameter<std::string>("scenario_topic", "/scenario");
    this->declare_parameter<std::string>("ego_topic", "/ego");
    this->declare_parameter<std::string>("strategy_topic", "/plan/strategy");
    this->declare_parameter<std::string>("targetSpace_topic", "/plan/target_space");
    this->declare_parameter("debugEnabled", false);
    debugEnabled = this->get_parameter("debugEnabled").as_bool();


    // create subscribers, publishers and attach callbacks
    std::string topic;
    this->get_parameter<std::string>("scenario_topic", topic);
    sub_scenario = this->create_subscription<crp_msgs::msg::Scenario>(
        topic,
        3,
        std::bind(&BehaviorPlanner::scenarioCallback, this, std::placeholders::_1)
    );

    this->get_parameter<std::string>("ego_topic", topic);
    sub_ego = this->create_subscription<crp_msgs::msg::Ego>(
        topic,
        3,
        std::bind(&BehaviorPlanner::egoCallback, this, std::placeholders::_1)
    );

    this->get_parameter<std::string>("strategy_topic", topic);
    pub_scenario = this->create_publisher<tier4_planning_msgs::msg::Scenario>(
        topic,
        1
    );

    this->get_parameter<std::string>("targetSpace_topic", topic);
    pub_targetSpace = this->create_publisher<crp_msgs::msg::TargetSpace>(
        topic,
        1
    );

    timer_pub = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&BehaviorPlanner::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "behavior_planner node has been started");
}


void BehaviorPlanner::scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg) {
    last_scenario = msg;

    if (debugEnabled)
    {
        RCLCPP_INFO(this->get_logger(), "New Scenario received!");
    }
    
}

void BehaviorPlanner::egoCallback(const crp_msgs::msg::Ego::SharedPtr msg) {
    last_ego = msg;

    if (debugEnabled)
    {
        RCLCPP_INFO(this->get_logger(), "New Ego received!");
    }
}

void BehaviorPlanner::timerCallback() {
    if (last_ego && last_scenario) {
        double x_now = (last_ego) ? last_ego->pose.pose.position.x : 0;
        obstacles  = last_scenario->local_obstacles.objects;
        objects = last_scenario->local_moving_objects.objects;
        relevant_obstacles.clear();
        relevant_objects.clear();

        out_targetSpace.free_space = last_scenario->free_space;
        out_targetSpace.header.stamp = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Obstacle count: %ld", obstacles.size());
        for (long unsigned int i = 0; i < obstacles.size(); i++)
        {
            auto current_obstacle = obstacles[i];
            double obstacle_x = current_obstacle.kinematics.initial_pose_with_covariance.pose.position.x;

            if ((obstacle_x >= x_now) && (obstacle_x-x_now <= 100)) {
                relevant_obstacles.push_back(current_obstacle);
                RCLCPP_INFO(this->get_logger(), "new obstacle added to relevants, distance: %f", obstacle_x-x_now);
                RCLCPP_INFO(this->get_logger(), "It's x is: %f and ego's is: %f", obstacle_x, x_now);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Object count: %ld", objects.size());
        for (long unsigned int i = 0; i < objects.size(); i++)
        {
            auto current_object = objects[i];
            double object_x = current_object.kinematics.initial_pose_with_covariance.pose.position.x;

            if ((object_x >= x_now) && (object_x-x_now <= 100)) {
                relevant_objects.push_back(current_object);
                RCLCPP_INFO(this->get_logger(), "new object added to relevants, distance: %f", object_x-x_now);
                RCLCPP_INFO(this->get_logger(), "It's x is: %f and ego's is: %f", object_x, x_now);
            }
        }
    
        out_targetSpace.relevant_obstacles = relevant_obstacles;
        out_targetSpace.relevant_objects = relevant_objects;

        pub_targetSpace->publish(out_targetSpace);
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorPlanner>());
    rclcpp::shutdown();
    return 0;
}
