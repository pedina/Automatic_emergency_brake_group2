#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "crp_msgs/msg/ego.hpp"
#include "crp_msgs/msg/target_space.hpp"
#include "crp_msgs/msg/behavior.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

class PlanLongEmergency : public rclcpp::Node
{
public:
    struct SafetyLimits
    {
        double max_accel;
        double max_jerk;
    };

    PlanLongEmergency() : Node("plan_long_emergency")
    {
        this->declare_parameter<double>("safety_distance", 5.0);
        this->declare_parameter<double>("prediction_horizon", 2.5);

        double safetyDistance;
        double predictionHorizon;
        this->get_parameter<double>("safety_distance", safetyDistance);
        this->get_parameter<double>("prediction_horizon", predictionHorizon);

        this->declare_parameter<std::string>("ego_topic", "/ego");
        this->declare_parameter<std::string>("target_topic", "plan/target_space");
        this->declare_parameter<std::string>("behavior_topic", "plan/strategy_behavior");
        this->declare_parameter<std::string>("output_topic", "plan/longEmergency/trajectory");
        this->declare_parameter<bool>("debug_enabled", false);

        std::string egoTopic;
        std::string targetTopic;
        std::string behaviorTopic;
        std::string outputTopic;
        this->get_parameter<std::string>("ego_topic", egoTopic);
        this->get_parameter<std::string>("target_topic", targetTopic);
        this->get_parameter<std::string>("behavior_topic", behaviorTopic);
        this->get_parameter<std::string>("output_topic", outputTopic);

        mode_limits_[0] = {1.5, 2.0};
        mode_limits_[1] = {2.5, 4.0};
        mode_limits_[2] = {4.0, 6.0};

        sub_ego_ = this->create_subscription<crp_msgs::msg::Ego>(
            egoTopic,
            10,
            std::bind(&PlanLongEmergency::egoCallback, this, std::placeholders::_1)
        );

        sub_target_ = this->create_subscription<crp_msgs::msg::TargetSpace>(
            targetTopic,
            10,
            std::bind(&PlanLongEmergency::targetCallback, this, std::placeholders::_1)
        );

        sub_behavior_ = this->create_subscription<crp_msgs::msg::Behavior>(
            behaviorTopic,
            10,
            std::bind(&PlanLongEmergency::behaviorCallback, this, std::placeholders::_1)
        );

        pub_trajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
            outputTopic, 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(1.0f / 50.0f),
            std::bind(&PlanLongEmergency::onTimer, this)
        );

        RCLCPP_INFO(this->get_logger(), "plan_long_emergency node has been started");
    }

private:
    void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg)
    {
        last_ego_ = msg;
    }

    void targetCallback(const crp_msgs::msg::TargetSpace::SharedPtr msg)
    {
        last_target_ = msg;
        last_target_time_ = this->now();
        missed_cycles_ = 0;
    }

    void behaviorCallback(const crp_msgs::msg::Behavior::SharedPtr msg)
    {
        current_behavior_ = msg;
    }

    void onTimer()
    {
        bool isDebugEnabled;
        this->get_parameter<bool>("debug_enabled", isDebugEnabled);

        if (!last_ego_ || !last_target_) return;

        if (!last_target_->relevant_objects.empty())
        {
            uint8_t mode = 1;
            if (current_behavior_)
            {
                mode = current_behavior_->deceleration_mode.data;
                if (mode_limits_.find(mode) == mode_limits_.end())
                {
                    mode = 1;
                }
            }

            auto trajectory = generateTrajectory(mode_limits_[mode]);
            pub_trajectory_->publish(trajectory);

            if (isDebugEnabled)
                RCLCPP_INFO(this->get_logger(), "Emergency planning active: object detected.");
        }
        else
        {
            if (isDebugEnabled)
                RCLCPP_INFO(this->get_logger(), "No critical object detected.");
        }
    }

    autoware_planning_msgs::msg::Trajectory generateTrajectory(SafetyLimits lims)
    {
        autoware_planning_msgs::msg::Trajectory traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "map";

        double safetyDistance;
        double predictionHorizon;
        this->get_parameter<double>("safety_distance", safetyDistance);
        this->get_parameter<double>("prediction_horizon", predictionHorizon);

        double v_ego = last_ego_->twist.twist.linear.x;
        double ego_x = last_ego_->pose.pose.position.x;
        double ego_y = last_ego_->pose.pose.position.y;

        if (last_target_->relevant_objects.empty())
        {
            return traj;
        }

        const auto& obj = last_target_->relevant_objects[0];
        double obj_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
        double obj_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;
        
        double distance = std::sqrt(std::pow(obj_x - ego_x, 2) + std::pow(obj_y - ego_y, 2));
        double s_stop = std::max(0.1, distance - safetyDistance);

        double a_req = -(v_ego * v_ego) / (2.0 * s_stop);
        if (a_req < -lims.max_accel)
        {
            a_req = -lims.max_accel;
        }

        double dt = 0.1;
        double curr_v = v_ego;
        double curr_s = 0.0;

        for (double t = 0; t <= predictionHorizon; t += dt)
        {
            autoware_planning_msgs::msg::TrajectoryPoint p;
            curr_v += a_req * dt;
            if (curr_v < 0) curr_v = 0;
            curr_s += curr_v * dt;

            p.longitudinal_velocity_mps = static_cast<float>(curr_v);
            p.acceleration_mps2        = static_cast<float>(a_req);
            p.pose.position.x = ego_x + curr_s;
            p.pose.position.y = ego_y;
            p.time_from_start.sec = static_cast<int32_t>(t);
            p.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
            traj.points.push_back(p);

            if (curr_v <= 0) break;
        }

        return traj;
    }

    // Subscriberek és publisher
    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr sub_ego_;
    rclcpp::Subscription<crp_msgs::msg::TargetSpace>::SharedPtr sub_target_;
    rclcpp::Subscription<crp_msgs::msg::Behavior>::SharedPtr sub_behavior_;
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Adattárolás
    crp_msgs::msg::Ego::SharedPtr last_ego_;
    crp_msgs::msg::TargetSpace::SharedPtr last_target_;
    crp_msgs::msg::Behavior::SharedPtr current_behavior_;

    std::map<uint8_t, SafetyLimits> mode_limits_;
    rclcpp::Time last_target_time_;
    int missed_cycles_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanLongEmergency>());
    rclcpp::shutdown();
    return 0;
}