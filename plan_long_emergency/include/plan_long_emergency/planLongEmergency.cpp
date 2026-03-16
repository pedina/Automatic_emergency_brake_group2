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

using namespace std::chrono_literals;

class PlanLongEmergency : public rclcpp::Node {
public:
    struct SafetyLimits {
        double max_accel; 
        double max_jerk; 
    };

    PlanLongEmergency() : Node("plan_long_emergency") {
        this->declare_parameter("safety_distance", 5.0); // 5 m
        this->declare_parameter("prediction_horizon", 2.5); // s
        
        mode_limits_[0] = {1.5, 2.0}; 
        mode_limits_[1] = {2.5, 4.0};
        mode_limits_[2] = {4.0, 6.0};

        sub_ego_ = this->create_subscription<crp_msgs::msg::Ego>(
            "/ego", 10, [this](const crp_msgs::msg::Ego::SharedPtr msg) { last_ego_ = msg; });

        sub_target_ = this->create_subscription<crp_msgs::msg::TargetSpace>(
            "plan/target_space", 10, [this](const crp_msgs::msg::TargetSpace::SharedPtr msg) { 
                last_target_ = msg; 
                last_target_time_ = this->now();
                missed_cycles_ = 0;
            });

        sub_behavior_ = this->create_subscription<crp_msgs::msg::Behavior>(
            "plan/strategy_behavior", 10, [this](const crp_msgs::msg::Behavior::SharedPtr msg) {
                current_behavior_ = msg;
            });

        pub_trajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
            "plan/longEmergency/trajectory", 10);

        // 20ms ciklusidő
        timer_ = this->create_wall_timer(20ms, std::bind(&PlanLongEmergency::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "Plan Long Emergency Node elindítva (ASIL D ready)");
    }

private:
    void onTimer() {
    if (!last_ego_ || !last_target_) return;

    if (!last_target_->relevant_objects.empty()) {
        
        uint8_t mode = (current_behavior_) ? current_behavior_->deceleration_mode.data : 1; 
        auto trajectory = generateTrajectory(mode_limits_[mode]);
        pub_trajectory_->publish(trajectory);
        
        RCLCPP_DEBUG(this->get_logger(), "Vészhelyzeti tervezés aktív: Objektum észlelve.");
    } 
    else {
        RCLCPP_DEBUG(this->get_logger(), "Nincs kritikus objektum");
    }
}

    autoware_planning_msgs::msg::Trajectory generateTrajectory(SafetyLimits lims) {
        autoware_planning_msgs::msg::Trajectory traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "map";

        double v_ego = last_ego_->twist.twist.linear.x;
        double ego_x = last_ego_->pose.pose.position.x;
        double ego_y = last_ego_->pose.pose.position.y;
        double d_safe = this->get_parameter("safety_distance").as_double();

        if (last_target_->relevant_objects.empty())
        { 
            return traj;
        }

        // Kritikus objektum távolsága
        const auto& obj = last_target_->relevant_objects[0];
        double obj_x = obj.kinematics.initial_pose_with_covariance.pose.position.x;
        double obj_y = obj.kinematics.initial_pose_with_covariance.pose.position.y;
        
        double distance = std::sqrt(std::pow(obj_x - ego_x, 2) + std::pow(obj_y - ego_y, 2));
        double s_stop = std::max(0.1, distance - d_safe);

        // Lassulás számítása (v^2 = 2as -> a = v^2 / 2s)
        double a_req = -(v_ego * v_ego) / (2.0 * s_stop);

        if (a_req < -lims.max_accel) {
            a_req = -lims.max_accel;
        }

        double horizon = this->get_parameter("prediction_horizon").as_double();
        double dt = 0.1; 
        double curr_v = v_ego;
        double curr_s = 0.0;

        for (double t = 0; t <= horizon; t += dt) {
            autoware_planning_msgs::msg::TrajectoryPoint p;
            
            curr_v += a_req * dt;
            if (curr_v < 0) curr_v = 0;
            curr_s += curr_v * dt;

            p.longitudinal_velocity_mps = static_cast<float>(curr_v);
            p.acceleration_fps2 = static_cast<float>(a_req);
            
            p.pose.position.x = ego_x + curr_s; 
            p.pose.position.y = ego_y;

            traj.points.push_back(p);
            if (curr_v <= 0) break;
        }

        return traj;
    }

    // ROS 2 
    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr sub_ego_;
    rclcpp::Subscription<crp_msgs::msg::TargetSpace>::SharedPtr sub_target_;
    rclcpp::Subscription<crp_msgs::msg::Behavior>::SharedPtr sub_behavior_;
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Adattarolas
    crp_msgs::msg::Ego::SharedPtr last_ego_;
    crp_msgs::msg::TargetSpace::SharedPtr last_target_;
    crp_msgs::msg::Behavior::SharedPtr current_behavior_;
    
    std::map<uint8_t, SafetyLimits> mode_limits_;
    rclcpp::Time last_target_time_;
    int missed_cycles_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanLongEmergency>());
    rclcpp::shutdown();
    return 0;
}