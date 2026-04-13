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
    
    pub_emergency_brake = this->create_publisher<std_msgs::msg::Bool>(
    "/emergency_brake",
    1
    );
    
    timer_pub = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&BehaviorPlanner::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "behavior_planner node has been started");
}


void BehaviorPlanner::scenarioCallback(const crp_msgs::msg::Scenario::SharedPtr msg) {
    last_scenario = msg;

    last_valid_scenario_ = msg;
    last_valid_scenario_time_ = this->now();
    missing_scenario_cycles_ = 0;

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
    if (!last_ego)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "No ego data yet, waiting...");
        return;
    }
    crp_msgs::msg::Scenario::SharedPtr working_scenario;
 
    if (last_scenario)
    {
        working_scenario = last_scenario;
        last_scenario = nullptr;
    }
    else if (last_valid_scenario_ && (missing_scenario_cycles_ <= MAX_MISSING_CYCLES))
    {
        missing_scenario_cycles_++;
        double dt = (this->now() - last_valid_scenario_time_).seconds();
 
        RCLCPP_WARN(this->get_logger(),"[H-03] Scenario missing (cycle %d/%d), extrapolating objects by dt=%.3fs",
            missing_scenario_cycles_, MAX_MISSING_CYCLES, dt);
 
        auto calc_missing_value = std::make_shared<crp_msgs::msg::Scenario>(*last_valid_scenario_);
        for (auto& obj : calc_missing_value->local_moving_objects.objects)
            obj = calcMissingObject(obj, dt);
        for (auto& obj : calc_missing_value->local_obstacles.objects)
            obj = calcMissingObject(obj, dt);
 
        working_scenario = calc_missing_value;
    }
    else
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[H-03] Sensor data missing >%d cycles! Degraded mode: publishing empty target.",
            MAX_MISSING_CYCLES);
 
        crp_msgs::msg::TargetSpace empty_msg;
        empty_msg.header.stamp = this->get_clock()->now();
        pub_targetSpace->publish(empty_msg);
        return;
    }

 
    double ego_x = last_ego->pose.pose.position.x;
    double ego_y = last_ego->pose.pose.position.y;
 
    obstacles = working_scenario->local_obstacles.objects;
    objects   = working_scenario->local_moving_objects.objects;
    relevant_obstacles.clear();
    relevant_objects.clear();
 
    out_targetSpace.free_space   = working_scenario->free_space;
    out_targetSpace.header.stamp = this->get_clock()->now();

    bool emergency_brake = false;
    double limit = (double) 100 * 100;
    double aeb_limit = 10.0 * 10.0;
    
    for (long unsigned int i = 0; i < obstacles.size(); i++)
        {
            auto current_obstacle = obstacles[i];
            double obstacle_x = current_obstacle.kinematics.initial_pose_with_covariance.pose.position.x;
            double obstacle_y = current_obstacle.kinematics.initial_pose_with_covariance.pose.position.y;

            double dx = ego_x - obstacle_x;
            double dy = ego_y - obstacle_y;

            double distance = dx*dx + dy*dy;
            
            

            if ((obstacle_x >= ego_x) && (distance <= limit)) {
                relevant_obstacles.push_back(current_obstacle);
                
                if (distance <= aeb_limit) {
                    emergency_brake = true;
                    RCLCPP_WARN(this->get_logger(), "[AEB] Emergency brake activated! Obstacle distance: %.2f m", std::sqrt(distance));
                }
                
                RCLCPP_INFO(this->get_logger(), "New obstacle, x: %f y: %f distance: %f", obstacle_x, obstacle_y, distance);
                RCLCPP_INFO(this->get_logger(), "Ego's,        x: %f y: %f", ego_x, ego_y);
            }
        }

        for (long unsigned int i = 0; i < objects.size(); i++)
        {
            auto current_object = objects[i];
            double object_x = current_object.kinematics.initial_pose_with_covariance.pose.position.x;
            double object_y = current_object.kinematics.initial_pose_with_covariance.pose.position.y;

            double dx = ego_x - object_x;
            double dy = ego_y - object_y;

            double distance = dx*dx + dy*dy;
            double limit = (double) 100 * 100;

            if ((object_x >= ego_x) && (distance <= limit)) {
                relevant_objects.push_back(current_object);

                if (distance <= aeb_limit) {
                    emergency_brake = true;
                    RCLCPP_WARN(this->get_logger(), "[AEB] Emergency brake activated! Object distance: %.2f m", std::sqrt(distance));
                }
                
                RCLCPP_INFO(this->get_logger(), "New object,   x: %f y: %f distance: %f", object_x, object_y, distance);
                RCLCPP_INFO(this->get_logger(), "Ego's,        x: %f y: %f", ego_x, ego_y);
            }
        }
    
        out_targetSpace.relevant_obstacles = relevant_obstacles;
        out_targetSpace.relevant_objects = relevant_objects;

        auto brake_msg = std_msgs::msg::Bool();
        brake_msg.data = emergency_brake;
        pub_emergency_brake->publish(brake_msg);
    
        pub_targetSpace->publish(out_targetSpace);
    }
    

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorPlanner>());
    rclcpp::shutdown();
    return 0;
}
