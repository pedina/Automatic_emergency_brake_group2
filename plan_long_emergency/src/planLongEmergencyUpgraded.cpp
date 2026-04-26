#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <map>
#include <algorithm>
#include <optional>
#include <limits>

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
        double max_accel; // Maximális lassulás (abszolút érték)
        double max_jerk;
    };

    PlanLongEmergency() : Node("plan_long_emergency")
    {
        // Paraméterek deklarálása
        this->declare_parameter<double>("safety_distance", 5.0);
        this->declare_parameter<double>("prediction_horizon", 5.0);
        this->declare_parameter<std::string>("ego_topic", "/ego");
        this->declare_parameter<std::string>("target_topic", "plan/target_space");
        this->declare_parameter<std::string>("behavior_topic", "plan/strategy_behavior");
        this->declare_parameter<std::string>("output_topic", "plan/longEmergency/trajectory");
        this->declare_parameter<bool>("debug_enabled", true);

        // Paraméterek beolvasása
        std::string egoTopic = this->get_parameter("ego_topic").as_string();
        std::string targetTopic = this->get_parameter("target_topic").as_string();
        std::string behaviorTopic = this->get_parameter("behavior_topic").as_string();
        std::string outputTopic = this->get_parameter("output_topic").as_string();

        // Viselkedési módok határai (Lassulási szintek)--hozzaadva
        this->declare_parameter<double>("high_max_accel",   1.5);
        this->declare_parameter<double>("high_max_jerk",    2.0);
        this->declare_parameter<double>("normal_max_accel", 2.5);
        this->declare_parameter<double>("normal_max_jerk",  4.0);
        this->declare_parameter<double>("low_max_accel",    4.0);
        this->declare_parameter<double>("low_max_jerk",     6.0);

        mode_limits_[0] = {
            this->get_parameter("high_max_accel").as_double(),
            this->get_parameter("high_max_jerk").as_double()
        };
        mode_limits_[1] = {
            this->get_parameter("normal_max_accel").as_double(),
            this->get_parameter("normal_max_jerk").as_double()
        };
        mode_limits_[2] = {
            this->get_parameter("low_max_accel").as_double(),
            this->get_parameter("low_max_jerk").as_double()
        };

        // Feliratkozások
        sub_ego_ = this->create_subscription<crp_msgs::msg::Ego>(
            egoTopic, 10, std::bind(&PlanLongEmergency::egoCallback, this, std::placeholders::_1));

        sub_target_ = this->create_subscription<crp_msgs::msg::TargetSpace>(
            targetTopic, 10, std::bind(&PlanLongEmergency::targetCallback, this, std::placeholders::_1));

        sub_behavior_ = this->create_subscription<crp_msgs::msg::Behavior>(
            behaviorTopic, 10, std::bind(&PlanLongEmergency::behaviorCallback, this, std::placeholders::_1));

        // Publikáló
        pub_trajectory_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(outputTopic, 10);

        // Időzítő (50 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::duration<float>(1.0f / 50.0f), std::bind(&PlanLongEmergency::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "PlanLongEmergency node sikeresen elindult az uj TTC logikaval!");
    }

private:
    std::optional<double> calculate_universal_ttc(double distance, double v_ego, double a_ego,
                                                  double v_target, double a_target,
                                                  double accel_threshold = 1e-5) {
        if (distance <= 0.0) return 0.0;

        double rel_v = v_ego - v_target;
        double rel_a = a_ego - a_target;

        if (std::abs(rel_a) < accel_threshold) {
            if (rel_v <= 0.0) return std::nullopt; // Távolodik, nincs ütközés
            return distance / rel_v;
        }

        double disc = rel_v * rel_v + 2.0 * rel_a * distance;
        if (disc < 0.0) return std::nullopt; // Nincs valós megoldás, elkerülik egymást

        double sq = std::sqrt(disc);
        double t1 = (-rel_v + sq) / rel_a;
        double t2 = (-rel_v - sq) / rel_a;

        double t = std::numeric_limits<double>::infinity();
        if (t1 > 0.0) t = std::min(t, t1);
        if (t2 > 0.0) t = std::min(t, t2);

        if (t == std::numeric_limits<double>::infinity()) return std::nullopt;
        return t;
    }

    // --- CALLBACKEK ---
    void egoCallback(const crp_msgs::msg::Ego::SharedPtr msg) { last_ego_ = msg; }
    void targetCallback(const crp_msgs::msg::TargetSpace::SharedPtr msg) { last_target_ = msg; }
    void behaviorCallback(const crp_msgs::msg::Behavior::SharedPtr msg) { current_behavior_ = msg; }

    // --- FŐ LOGIKAI CIKLUS ---
    void onTimer()
    {
        if (!last_ego_ || !last_target_) return; // Védvonal: Ha nincs adat, ne csináljunk semmit

        uint8_t mode = 1; // Alapértelmezett mód
        if (current_behavior_ && mode_limits_.find(current_behavior_->deceleration_mode.data) != mode_limits_.end()) {
            mode = current_behavior_->deceleration_mode.data;
        }

        // Generáljuk és publikáljuk a trajektóriát a jelenlegi limit alapján
        auto trajectory = generateTrajectory(mode_limits_[mode]);
        if (trajectory.points[0].time_from_start.sec >= 0)
            pub_trajectory_->publish(trajectory);
    }

    // --- TRAJEKTÓRIA GENERÁLÁS (Kombinált logika) ---
    autoware_planning_msgs::msg::Trajectory generateTrajectory(SafetyLimits lims)
    {
        autoware_planning_msgs::msg::Trajectory traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "map";

        double predictionHorizon = this->get_parameter("prediction_horizon").as_double();

        double v_ego = last_ego_->twist.twist.linear.x;
        double a_ego = last_ego_->accel.accel.linear.x;

        double min_ttc = std::numeric_limits<double>::infinity();
        double target_distance_at_min_ttc = 0.0;

        // 1. Legkisebb TTC-jű objektum megkeresése!
        for (const auto& obj : last_target_->relevant_objects) {
            double tx = obj.kinematics.initial_pose_with_covariance.pose.position.x;
            double ty = obj.kinematics.initial_pose_with_covariance.pose.position.y;
            double distance = std::hypot(tx, ty); // Vektoros távolság

            // Csak X irányú (hosszirányú) mozgást nézünk a ráfutásos balesetekhez
            double v_target = obj.kinematics.initial_twist_with_covariance.twist.linear.x;
            double a_target = obj.kinematics.initial_acceleration_with_covariance.accel.linear.x;

            
            RCLCPP_INFO(this->get_logger(), "Pozíció: (%.2f, %.2f), Sebesség: %.2f m/s, Gyorsulás: %.2f m/s2",
                tx, ty, v_target, a_target);

            auto ttc_opt = calculate_universal_ttc(distance, v_ego, a_ego, v_target, a_target);

            if (ttc_opt.has_value() && ttc_opt.value() < min_ttc) {
                min_ttc = ttc_opt.value();
                target_distance_at_min_ttc = distance;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Legkisebb TTC: %.2fs, Távolság ennél: %.2fm", min_ttc, target_distance_at_min_ttc);
        
        // Trajektória felépítése (1 pont)
        double targetSpeed = 0.0;

        autoware_planning_msgs::msg::TrajectoryPoint p;

        p.longitudinal_velocity_mps = targetSpeed;
        p.time_from_start.sec = static_cast<int32_t>(min_ttc);
        p.time_from_start.nanosec = static_cast<uint32_t>((min_ttc - std::floor(min_ttc)) * 1e9);

        traj.points.push_back(p);

        return traj;
    }

    rclcpp::Subscription<crp_msgs::msg::Ego>::SharedPtr sub_ego_;
    rclcpp::Subscription<crp_msgs::msg::TargetSpace>::SharedPtr sub_target_;
    rclcpp::Subscription<crp_msgs::msg::Behavior>::SharedPtr sub_behavior_;
    rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
    rclcpp::TimerBase::SharedPtr timer_;

    crp_msgs::msg::Ego::SharedPtr last_ego_;
    crp_msgs::msg::TargetSpace::SharedPtr last_target_;
    crp_msgs::msg::Behavior::SharedPtr current_behavior_;

    std::map<uint8_t, SafetyLimits> mode_limits_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanLongEmergency>());
    rclcpp::shutdown();
    return 0;
}